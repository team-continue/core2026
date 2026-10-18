#!/usr/bin/env python3
r"""
砲塔1軸の「位置指令 → 実角度」モデルをオフラインで同定する.

turret_sysid_logger が保存した実行ディレクトリ（command.csv / joint.csv / meta.yaml）
または rosbag2 を読み、次のモデルを最小二乗で推定する。

  first_order  : K e^{-Ls} / (T s + 1)
  second_order : K wn^2 e^{-Ls} / (s^2 + 2 zeta wn s + wn^2)

むだ時間 L は格子探索（粗 5ms → 細 1ms）、それ以外のパラメータは各 L で
scipy.optimize.least_squares により推定する。シミュレーションは記録した指令を
ゼロ次ホールドで入力し、実測時刻での角度と比べる（出力誤差法）。

学習データ（--train）で推定し、別に取った検証データ（--valid）での再現度で合否を判定する。
結果は C++ の core_shooter::loadTurretAxisModel() がそのまま読める YAML で保存する。

依存: numpy, scipy, PyYAML（matplotlib は --plot 指定時のみ, rosbag2_py は --bag 指定時のみ）
ROS 環境がなくても CSV からの同定は動く。

例:
  turret_sysid_identify --train sysid_data/left_yaw_step_* sysid_data/left_yaw_prbs_* \\
      --valid sysid_data/left_yaw_multistep_* --out left_yaw_model.yaml --plot left_yaw.png
"""

import argparse
import csv
import datetime
import math
import os
import sys

import numpy as np

try:
    from scipy.optimize import least_squares
    from scipy.signal import cont2discrete, lfilter
except ImportError:  # pragma: no cover - 実行環境の案内
    sys.exit('scipy is required: sudo apt install python3-scipy  (or pip install scipy)')
import yaml

SIM_DT = 0.001          # シミュレーション刻み [s]
FIT_PASS = 85.0         # 検証データの適合度 [%] 合格
FIT_MARGINAL = 70.0     # 検証データの適合度 [%] 条件付き


# ---------------------------------------------------------------------------
# データ読み込み
# ---------------------------------------------------------------------------

class Run:
    """1回分の記録. 時刻は全て同じ時間軸 [s]."""

    def __init__(self, name, t_cmd, u, t_meas, y, meta):
        order = np.argsort(t_cmd, kind='stable')
        self.t_cmd = np.asarray(t_cmd, dtype=float)[order]
        self.u = np.asarray(u, dtype=float)[order]
        order = np.argsort(t_meas, kind='stable')
        self.t_meas = np.asarray(t_meas, dtype=float)[order]
        self.y = np.asarray(y, dtype=float)[order]
        self.name = name
        self.meta = meta or {}
        if len(self.t_cmd) < 10 or len(self.t_meas) < 10:
            raise ValueError(f'{name}: too few samples (command={len(self.t_cmd)}, '
                             f'joint={len(self.t_meas)})')
        # 指令区間内の実測だけを使う
        mask = (self.t_meas >= self.t_cmd[0]) & (self.t_meas <= self.t_cmd[-1])
        if mask.sum() < 10:
            raise ValueError(f'{name}: command and joint timestamps do not overlap '
                             '(clock offset between PCs? try --time-source recv)')
        self.t_meas = self.t_meas[mask]
        self.y = self.y[mask]
        gap = float(np.max(np.diff(self.t_meas)))
        if gap > 0.05:
            print(f'warning: {name}: joint_states has a gap of {gap * 1e3:.0f} ms '
                  '(CPU/network stall?). Re-record if the fit looks poor.', file=sys.stderr)


def _read_csv(path):
    with open(path, newline='') as f:
        return list(csv.DictReader(f))


def load_run_dir(path, time_source):
    """turret_sysid_logger の出力ディレクトリを読む."""
    cmd_rows = _read_csv(os.path.join(path, 'command.csv'))
    joint_rows = _read_csv(os.path.join(path, 'joint.csv'))
    meta = {}
    meta_path = os.path.join(path, 'meta.yaml')
    if os.path.exists(meta_path):
        with open(meta_path) as f:
            meta = (yaml.safe_load(f) or {}).get('turret_sysid_run', {})
    if meta.get('dry_run'):
        raise ValueError(f'{path}: this run was a dry run (no command was sent)')
    if str(meta.get('status', 'completed')) != 'completed':
        print(f'warning: {path}: status is "{meta.get("status")}" (aborted run)', file=sys.stderr)

    t_cmd = np.array([float(r['send_time']) for r in cmd_rows])
    u = np.array([float(r['command']) for r in cmd_rows])
    recv = np.array([float(r['recv_time']) for r in joint_rows])
    stamp = np.array([float(r['stamp']) for r in joint_rows])
    y = np.array([float(r['position']) for r in joint_rows])

    offset = float(np.median(recv - stamp))
    source = time_source
    if source == 'auto':
        # 同じ PC で記録していれば recv - stamp は数 ms。大きければ時計がずれている
        source = 'stamp' if -0.005 <= offset <= 0.05 else 'recv'
        if source == 'recv':
            print(f'warning: {path}: median(recv - stamp) = {offset * 1e3:.1f} ms. '
                  'Clocks of logger PC and robot differ; using receive time. '
                  'The identified delay then includes network latency.', file=sys.stderr)
    t_meas = stamp if source == 'stamp' else recv
    meta = dict(meta)
    meta['time_source'] = source
    meta['recv_minus_stamp_ms'] = round(offset * 1e3, 3)
    return Run(os.path.basename(os.path.normpath(path)), t_cmd, u, t_meas, y, meta)


def load_bag(path, motor_id, can_topic, joint_topic, window):
    """rosbag2 から指令（CANArray の該当 id）と実測角を取り出す. 時刻はバッグの受信時刻."""
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError:
        sys.exit('--bag requires a sourced ROS 2 environment (rosbag2_py)')

    storage_id = 'sqlite3'
    meta_path = os.path.join(path, 'metadata.yaml')
    if os.path.exists(meta_path):
        with open(meta_path) as f:
            info = yaml.safe_load(f).get('rosbag2_bagfile_information', {})
            storage_id = info.get('storage_identifier', storage_id)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage_id),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for topic in (can_topic, joint_topic):
        if topic not in types:
            sys.exit(f'{path}: topic {topic} not found. topics: {sorted(types)}')
    can_type = get_message(types[can_topic])
    joint_type = get_message(types[joint_topic])

    t_cmd, u, t_meas, y = [], [], [], []
    t_first = None
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        t = t_ns * 1e-9
        if t_first is None:
            t_first = t
        rel = t - t_first
        if window and not (window[0] <= rel <= window[1]):
            continue
        if topic == can_topic:
            msg = deserialize_message(data, can_type)
            for can in msg.array:
                if can.id == motor_id and len(can.data) > 0:
                    t_cmd.append(t)
                    u.append(float(can.data[0]))
        elif topic == joint_topic:
            msg = deserialize_message(data, joint_type)
            if motor_id < len(msg.position):
                t_meas.append(t)
                y.append(float(msg.position[motor_id]))
    if not t_cmd:
        sys.exit(f'{path}: no command for motor_id={motor_id} on {can_topic}')
    name = os.path.basename(os.path.normpath(path))
    if window:
        name += f'[{window[0]:g}-{window[1]:g}s]'
    return Run(name, t_cmd, u, t_meas, y, {'time_source': 'bag_receive', 'motor_id': motor_id})


# ---------------------------------------------------------------------------
# モデルとシミュレーション
# ---------------------------------------------------------------------------

MODEL_TYPES = ('first_order', 'second_order')


def theta_to_params(model_type, theta):
    if model_type == 'first_order':
        return {'gain': float(theta[0]), 'time_constant_sec': float(math.exp(theta[1]))}
    return {'gain': float(theta[0]), 'natural_frequency_rad_per_sec': float(math.exp(theta[1])),
            'damping_ratio': float(theta[2])}


def discrete_tf(model_type, params, dt):
    k = params['gain']
    if model_type == 'first_order':
        num, den = [k], [params['time_constant_sec'], 1.0]
    else:
        wn = params['natural_frequency_rad_per_sec']
        z = params['damping_ratio']
        num, den = [k * wn * wn], [1.0, 2.0 * z * wn, wn * wn]
    numd, dend, _ = cont2discrete((num, den), dt, method='zoh')
    return np.ravel(numd), np.ravel(dend)


class SimGrid:
    """1回分の記録を固定刻みのシミュレーション格子に載せたもの（遅延ごとに入力を作り直す）."""

    def __init__(self, run, dt=SIM_DT):
        self.run = run
        self.dt = dt
        self.t = np.arange(run.t_cmd[0], run.t_cmd[-1], dt)
        self.y0 = float(np.interp(run.t_cmd[0], run.t_meas, run.y))
        self.u0 = float(run.u[0])
        self._cache = {}

    def delayed_input(self, delay):
        key = round(delay, 6)
        if key not in self._cache:
            idx = np.searchsorted(self.run.t_cmd, self.t - delay, side='right') - 1
            self._cache[key] = self.run.u[np.clip(idx, 0, None)] - self.u0
        return self._cache[key]

    def simulate(self, model_type, params, delay):
        """実測時刻での角度予測. 開始時は指令 u0 で静止していたとみなす（偏差系で計算）."""
        b, a = discrete_tf(model_type, params, self.dt)
        y_grid = self.y0 + lfilter(b, a, self.delayed_input(delay))
        return np.interp(self.run.t_meas, self.t, y_grid)


def fit_metrics(y, y_hat):
    err = y - y_hat
    denom = np.linalg.norm(y - np.mean(y))
    fit = 100.0 * (1.0 - np.linalg.norm(err) / denom) if denom > 0 else float('nan')
    return {'fit_percent': float(fit), 'rmse_rad': float(np.sqrt(np.mean(err ** 2))),
            'max_abs_error_rad': float(np.max(np.abs(err)))}


def fit_model(model_type, grids, max_delay):
    """むだ時間の格子探索 + 各むだ時間での非線形最小二乗."""
    if model_type == 'first_order':
        starts = [np.array([1.0, math.log(0.05)])]
        lower, upper = [0.3, math.log(1e-3)], [3.0, math.log(5.0)]
    else:
        starts = [np.array([1.0, math.log(w), z]) for w in (10.0, 30.0) for z in (0.5, 1.0)]
        lower, upper = [0.3, math.log(0.5), 0.05], [3.0, math.log(500.0), 5.0]

    def residual(theta, delay):
        params = theta_to_params(model_type, theta)
        return np.concatenate([g.simulate(model_type, params, delay) - g.run.y for g in grids])

    def solve(delay, init):
        best = None
        for x0 in init:
            x0 = np.clip(x0, np.array(lower) + 1e-9, np.array(upper) - 1e-9)
            res = least_squares(residual, x0, bounds=(lower, upper), args=(delay,),
                                x_scale='jac', max_nfev=200)
            if best is None or res.cost < best.cost:
                best = res
        return best

    results = {}
    coarse = np.arange(0.0, max_delay + 1e-9, 0.005)
    warm = starts
    for d in coarse:
        res = solve(float(d), warm)
        results[round(float(d), 4)] = res
        warm = [res.x] + starts[:1]
    best_d = min(results, key=lambda d: results[d].cost)
    fine = np.arange(max(0.0, best_d - 0.005), min(max_delay, best_d + 0.005) + 1e-9, 0.001)
    for d in fine:
        key = round(float(d), 4)
        if key not in results:
            results[key] = solve(float(d), [results[best_d].x])
    best_d = min(results, key=lambda d: results[d].cost)
    params = theta_to_params(model_type, results[best_d].x)
    params['delay_sec'] = float(best_d)
    warnings = []
    if best_d >= max_delay - 1e-9:
        warnings.append('delay hit the search limit (--max-delay)')
    theta = results[best_d].x
    for i, (lo, hi) in enumerate(zip(lower, upper)):
        if abs(theta[i] - lo) < 1e-6 or abs(theta[i] - hi) < 1e-6:
            warnings.append(f'parameter #{i} hit its bound; data may not excite the dynamics')
    return params, warnings


def evaluate(model_type, params, grids):
    out = []
    for g in grids:
        y_hat = g.simulate(model_type, params, params['delay_sec'])
        m = fit_metrics(g.run.y, y_hat)
        m['run'] = g.run.name
        out.append(m)
    return out


def combined_fit(grids, model_type, params):
    if not grids:
        return float('nan')
    y = np.concatenate([g.run.y - np.mean(g.run.y) for g in grids])
    y_hat = np.concatenate([
        g.simulate(model_type, params, params['delay_sec']) - np.mean(g.run.y) for g in grids])
    return fit_metrics(y, y_hat)['fit_percent']


def step_characteristics(model_type, params, amplitude=0.1, horizon=2.0, dt=SIM_DT):
    """同定モデルのステップ応答指標（むだ時間込み）."""
    t = np.arange(0.0, horizon, dt)
    b, a = discrete_tf(model_type, params, dt)
    u = np.where(t >= params['delay_sec'], amplitude, 0.0)
    y = lfilter(b, a, u)
    final = params['gain'] * amplitude

    def first_cross(level):
        idx = np.nonzero(y >= level)[0]
        return float(t[idx[0]]) if len(idx) else float('nan')

    outside = np.nonzero(np.abs(y - final) > 0.02 * abs(final))[0]
    settle = float(t[outside[-1]] + dt) if len(outside) and outside[-1] < len(t) - 1 \
        else float('nan')
    return {
        'rise_time_10_90_sec': first_cross(0.9 * final) - first_cross(0.1 * final),
        'overshoot_percent': float(max(0.0, (np.max(y) - final) / final * 100.0)),
        'settling_time_2pct_sec': settle,
    }


def observed_max_rate(runs):
    """実測角の角速度の 99 パーセンタイル [rad/s]（大振幅ステップでの速度飽和の目安）."""
    rates = []
    for r in runs:
        if len(r.t_meas) < 5:
            continue
        dt = np.diff(r.t_meas)
        ok = dt > 1e-4
        v = np.abs(np.diff(r.y)[ok] / dt[ok])
        if len(v) >= 5:
            # 単発のノイズを避けるため 5 点移動中央値
            v = np.median(np.lib.stride_tricks.sliding_window_view(v, 5), axis=1)
        rates.append(v)
    if not rates:
        return float('nan')
    return float(np.percentile(np.concatenate(rates), 99))


def verdict_of(valid_fit):
    if not math.isfinite(valid_fit):
        return 'NO_VALIDATION'
    if valid_fit >= FIT_PASS:
        return 'PASS'
    if valid_fit >= FIT_MARGINAL:
        return 'MARGINAL'
    return 'FAIL'


# ---------------------------------------------------------------------------
# 出力
# ---------------------------------------------------------------------------

def plot_runs(path, model_type, params, train, valid):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('warning: matplotlib not found; --plot skipped', file=sys.stderr)
        return
    grids = [('train', g) for g in train] + [('valid', g) for g in valid]
    fig, axes = plt.subplots(len(grids), 1, figsize=(12, 3.2 * len(grids)), squeeze=False)
    for ax, (kind, g) in zip(axes[:, 0], grids):
        r = g.run
        y_hat = g.simulate(model_type, params, params['delay_sec'])
        t0 = r.t_cmd[0]
        ax.step(r.t_cmd - t0, r.u, where='post', label='command', color='0.6', linewidth=1)
        ax.plot(r.t_meas - t0, r.y, label='measured', linewidth=1.2)
        ax.plot(r.t_meas - t0, y_hat, label='model', linewidth=1.2, linestyle='--')
        m = fit_metrics(r.y, y_hat)
        ax.set_title(f'[{kind}] {r.name}  fit={m["fit_percent"]:.1f}%  '
                     f'rmse={m["rmse_rad"] * 1e3:.2f} mrad')
        ax.set_xlabel('time [s]')
        ax.set_ylabel('angle [rad]')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right')
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    print(f'plot saved: {path}')


def main(argv=None):
    ap = argparse.ArgumentParser(
        description='Identify turret axis model (position command -> measured angle).')
    ap.add_argument('--train', nargs='+', default=[], help='run directories for estimation')
    ap.add_argument('--valid', nargs='*', default=[], help='run directories for validation')
    ap.add_argument('--bag', help='rosbag2 directory (instead of --train/--valid run dirs)')
    ap.add_argument('--bag-motor-id', type=int, help='motor id to extract from --bag')
    ap.add_argument('--bag-train-window', nargs=2, type=float, metavar=('START', 'END'),
                    help='seconds from bag start used for training')
    ap.add_argument('--bag-valid-window', nargs=2, type=float, metavar=('START', 'END'),
                    help='seconds from bag start used for validation')
    ap.add_argument('--can-topic', default='/hardware/can/tx')
    ap.add_argument('--joint-topic', default='/joint_states')
    ap.add_argument('--model', choices=MODEL_TYPES + ('auto',), default='auto')
    ap.add_argument('--max-delay', type=float, default=0.25, help='delay search limit [s]')
    ap.add_argument('--time-source', choices=('auto', 'stamp', 'recv'), default='auto')
    ap.add_argument('--mpc-rate', type=float, default=30.0,
                    help='control rate [Hz] used to report delay in control steps')
    ap.add_argument('--side', help='override side in output (default: from meta.yaml)')
    ap.add_argument('--axis', help='override axis in output (default: from meta.yaml)')
    ap.add_argument('--out', required=True, help='output YAML path')
    ap.add_argument('--plot', help='save comparison plot (PNG) to this path')
    args = ap.parse_args(argv)

    if args.bag:
        if args.bag_motor_id is None:
            ap.error('--bag requires --bag-motor-id')
        train_runs = [load_bag(args.bag, args.bag_motor_id, args.can_topic, args.joint_topic,
                               args.bag_train_window)]
        valid_runs = [load_bag(args.bag, args.bag_motor_id, args.can_topic, args.joint_topic,
                               args.bag_valid_window)] if args.bag_valid_window else []
    else:
        if not args.train:
            ap.error('--train (or --bag) is required')
        train_runs = [load_run_dir(p, args.time_source) for p in args.train]
        valid_runs = [load_run_dir(p, args.time_source) for p in args.valid]

    metas = [r.meta for r in train_runs + valid_runs]
    side = args.side or next((m.get('side') for m in metas if m.get('side')), 'unknown')
    axis = args.axis or next((m.get('axis') for m in metas if m.get('axis')), 'unknown')
    motor_ids = sorted({m.get('motor_id') for m in metas if m.get('motor_id') is not None})
    if len(motor_ids) > 1:
        sys.exit(f'error: runs from different motors were mixed: motor_id={motor_ids}')
    if len({(m.get('side'), m.get('axis')) for m in metas if m.get('side')}) > 1:
        sys.exit('error: runs from different side/axis were mixed')

    train = [SimGrid(r) for r in train_runs]
    valid = [SimGrid(r) for r in valid_runs]
    if not valid:
        print('warning: no validation data (--valid). The verdict cannot be decided; '
              'record another run for validation.', file=sys.stderr)

    candidates = {}
    types = MODEL_TYPES if args.model == 'auto' else (args.model,)
    for mt in types:
        print(f'fitting {mt} ...', flush=True)
        params, warns = fit_model(mt, train, args.max_delay)
        candidates[mt] = {
            'params': params,
            'warnings': warns,
            'train_fit_percent': combined_fit(train, mt, params),
            'valid_fit_percent': combined_fit(valid, mt, params),
        }

    def score(mt):
        c = candidates[mt]
        return c['valid_fit_percent'] if math.isfinite(c['valid_fit_percent']) \
            else c['train_fit_percent']

    chosen = types[0]
    if len(types) == 2:
        # 2次の改善が 1 ポイント未満なら簡単な 1次を採用する
        chosen = 'second_order' if score('second_order') > score('first_order') + 1.0 \
            else 'first_order'
    best = candidates[chosen]
    params = best['params']
    train_eval = evaluate(chosen, params, train)
    valid_eval = evaluate(chosen, params, valid)
    verdict = verdict_of(best['valid_fit_percent'])
    step = step_characteristics(chosen, params)
    max_rate = observed_max_rate(train_runs + valid_runs)
    ts = 1.0 / args.mpc_rate
    delay_steps = int(math.floor(params['delay_sec'] / ts + 1e-9))

    notes = list(best['warnings'])
    if not 0.9 <= params['gain'] <= 1.1:
        notes.append(f'gain {params["gain"]:.3f} is far from 1 (offset/gravity? '
                     'MPC disturbance estimation compensates steady-state error)')
    if params['delay_sec'] > 0.15:
        notes.append('delay > 150 ms; check that the logger ran on the robot PC')

    model_yaml = {'model_type': chosen, 'gain': round(params['gain'], 6)}
    for key in ('time_constant_sec', 'natural_frequency_rad_per_sec', 'damping_ratio'):
        if key in params:
            model_yaml[key] = round(params[key], 6)
    model_yaml['delay_sec'] = round(params['delay_sec'], 4)
    model_yaml.update({
        # 以下は参照情報（C++ 側は読まない）
        'side': side,
        'axis': axis,
        'motor_id': motor_ids[0] if motor_ids else None,
        'identified_at': datetime.datetime.now().isoformat(timespec='seconds'),
        'time_source': sorted({str(m.get('time_source')) for m in metas}),
        'verdict': verdict,
        'notes': notes,
        'observed_max_rate_rad_per_sec': round(max_rate, 4),
        'step_response_0p1rad': {k: round(v, 4) for k, v in step.items()},
        'delay_at_control_rate': {'rate_hz': args.mpc_rate, 'delay_steps': delay_steps,
                                  'fractional_delay_sec':
                                      round(params['delay_sec'] - delay_steps * ts, 4)},
        'fit': {
            'train_fit_percent': round(best['train_fit_percent'], 2),
            'valid_fit_percent': (round(best['valid_fit_percent'], 2)
                                  if math.isfinite(best['valid_fit_percent']) else None),
            'train': [{k: (round(v, 6) if isinstance(v, float) else v) for k, v in e.items()}
                      for e in train_eval],
            'valid': [{k: (round(v, 6) if isinstance(v, float) else v) for k, v in e.items()}
                      for e in valid_eval],
        },
        'candidates': {
            mt: {
                **{k: round(v, 6) for k, v in c['params'].items()},
                'train_fit_percent': round(c['train_fit_percent'], 2),
                'valid_fit_percent': (round(c['valid_fit_percent'], 2)
                                      if math.isfinite(c['valid_fit_percent']) else None),
            } for mt, c in candidates.items()
        },
        'data': {'train': [g.run.name for g in train], 'valid': [g.run.name for g in valid]},
    })
    out_dir = os.path.dirname(os.path.abspath(args.out))
    os.makedirs(out_dir, exist_ok=True)
    with open(args.out, 'w') as f:
        f.write(f'# turret_sysid_identify の出力（{side} {axis}）。'
                'turret_axis_model の先頭 6 項目だけを MPC が使う。\n')
        yaml.safe_dump({'turret_axis_model': model_yaml}, f, sort_keys=False,
                       allow_unicode=True)

    # ---- 結果表示 ----
    print()
    print(f'=== {side} {axis} (motor_id={model_yaml["motor_id"]}) ===')
    for mt, c in candidates.items():
        mark = '*' if mt == chosen else ' '
        p = ', '.join(f'{k}={v:.4g}' for k, v in c['params'].items())
        vf = c['valid_fit_percent']
        print(f' {mark} {mt:12s} {p}')
        print(f'     train fit {c["train_fit_percent"]:6.2f}%   valid fit '
              f'{vf:6.2f}%' if math.isfinite(vf) else
              f'     train fit {c["train_fit_percent"]:6.2f}%   valid fit   n/a')
    print()
    for kind, evals in (('train', train_eval), ('valid', valid_eval)):
        for e in evals:
            print(f'  [{kind}] {e["run"]}: fit {e["fit_percent"]:.2f}%  '
                  f'rmse {e["rmse_rad"] * 1e3:.2f} mrad  '
                  f'max {e["max_abs_error_rad"] * 1e3:.2f} mrad')
    print()
    print(f'  step(0.1rad): rise {step["rise_time_10_90_sec"] * 1e3:.1f} ms, '
          f'overshoot {step["overshoot_percent"]:.1f} %, '
          f'settling {step["settling_time_2pct_sec"] * 1e3:.1f} ms')
    print(f'  observed max rate: {max_rate:.3f} rad/s')
    print(f'  delay at {args.mpc_rate:g} Hz: {delay_steps} steps + '
          f'{(params["delay_sec"] - delay_steps * ts) * 1e3:.1f} ms')
    for n in notes:
        print(f'  note: {n}')
    print(f'\n  VERDICT: {verdict}  (PASS >= {FIT_PASS:.0f}%, MARGINAL >= {FIT_MARGINAL:.0f}% '
          'on validation data)')
    print(f'  saved: {args.out}')
    if args.plot:
        plot_runs(args.plot, chosen, params, train, valid)
    return 0 if verdict in ('PASS', 'MARGINAL') else 1


if __name__ == '__main__':
    sys.exit(main())
