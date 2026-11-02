"""Estimate a delayed speed response from independent ROS 2 train/validation bags."""
import argparse
import json
from pathlib import Path

import matplotlib
import numpy as np
from scipy.optimize import minimize_scalar
from scipy.signal import lfilter

from .base_sysid_probe import CAN, HAZARD, yaw


AXES = ('waist', 'vx', 'vy', 'wz', 'wheel0', 'wheel1', 'wheel2', 'wheel3')
DT = 0.01
RADIUS = 0.13 / 2
WIDTH = 0.5304


def bag_messages(path, topics, window=None):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {m.name: m.type for m in reader.get_all_topics_and_types()}
    missing = set(topics) - types.keys()
    if missing:
        raise ValueError(f'{path}: missing topics {sorted(missing)}')
    cls = {key: get_message(types[key]) for key in topics}
    if not reader.has_next():
        raise ValueError(f'{path}: empty bag')
    first = reader.read_next()
    origin = first[2]
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    record = first
    while record is not None:
        topic, raw, ns = record
        t = (ns-origin)*1e-9
        if topic in cls and (window is None or window[0] <= t <= window[1]):
            yield topic, t, deserialize_message(raw, cls[topic])
        record = reader.read_next() if reader.has_next() else None


def body_speed(refs, axis):
    # Inverse of the four-wheel map in body_control_node (lower-body coordinates).
    w0, w1, w2, w3 = (refs[i] for i in range(4))
    if axis == 'wz':
        return (w0 + w1 + w2 + w3) * RADIUS / (2 * np.sqrt(2) * WIDTH)
    component = -w0 - w1 + w2 + w3 if axis == 'vx' else w0 - w1 - w2 + w3
    return component * RADIUS / (2 * np.sqrt(2))


def read_run(path, axis, odom_topic, base_frame, window=None):
    wheel_axis = axis.startswith('wheel')
    joint_axis = axis == 'waist' or wheel_axis
    index = 4 if axis == 'waist' else int(axis[-1]) if wheel_axis else None
    feedback = '/hardware/joint_states' if joint_axis else odom_topic
    cmds, poses, hazards = [], [], []
    frame = None
    for topic, t, msg in bag_messages(path, [CAN, HAZARD, feedback], window):
        if topic == HAZARD:
            hazards.append((t, msg.data))
        elif topic == CAN:
            refs = {c.id: c.data[-1] for c in msg.array if c.data}
            if joint_axis:
                if index in refs:
                    cmds.append((t, refs[index]))
            elif all(i in refs for i in range(4)):
                cmds.append((t, body_speed(refs, axis)))
        elif joint_axis:
            if index >= len(msg.position):
                raise ValueError(f'missing encoder ID {index}')
            poses.append((t, msg.position[index]))
        else:
            if msg.child_frame_id != base_frame or not msg.header.frame_id:
                raise ValueError(f'expected floor odometry for {base_frame}, got '
                                 f'{msg.header.frame_id} -> {msg.child_frame_id}')
            if frame is not None and frame != msg.header.frame_id:
                raise ValueError('odometry frame changed within a run')
            frame = msg.header.frame_id
            p = msg.pose.pose
            poses.append((t, p.position.x, p.position.y, yaw(p.orientation)))
    if len(cmds) < 10 or len(poses) < 10 or not hazards:
        raise ValueError(f'{path}: insufficient command, feedback or hazard samples')
    tc, u = np.array(cmds).T
    pose = np.array(poses)
    if not np.isfinite(pose).all() or not np.isfinite(u).all():
        raise ValueError(f'{path}: non-finite measurement')
    if joint_axis:
        position = np.unwrap(pose[:, 1])
    else:
        heading = np.unwrap(pose[:, 3])
        mid = (heading[1:] + heading[:-1]) / 2
        dx, dy = np.diff(pose[:, 1]), np.diff(pose[:, 2])
        if axis == 'wz':
            position = heading
        else:
            increment = (np.cos(mid)*dx + np.sin(mid)*dy if axis == 'vx' else
                         -np.sin(mid)*dx + np.cos(mid)*dy)
            position = np.r_[0, np.cumsum(increment)]
    return prepare_run(tc, u, pose[:, 0], position, hazards)


def prepare_run(tc, u, ty, y, hazards):
    for t in (tc, ty):
        if np.any(np.diff(t) < 0):
            raise ValueError('timestamps went backwards')
    active = np.flatnonzero(abs(u) > max(1e-4, 0.015 * np.max(abs(u))))
    if len(active) < 2 or min(u) >= -1e-4 or max(u) <= 1e-4:
        raise ValueError('need positive and negative excitation')
    start = max(tc[0], ty[0], tc[active[0]] - 0.5)
    stop = min(tc[-1], ty[-1], tc[active[-1]] + 1.0)
    if tc[active[0]] - start < 0.25 or stop - tc[active[-1]] < 0.45:
        raise ValueError('need a stationary lead-in and a recorded stop after excitation')
    for times, limit in [(tc, 0.20), (ty, 0.35)]:
        points = np.r_[start, times[(times > start) & (times < stop)], stop]
        if np.max(np.diff(points)) > limit:
            raise ValueError('gap in command or measurement within excitation')
    ht, h = np.array(hazards).T
    before = np.flatnonzero(ht <= start)
    inside = (ht >= start) & (ht <= stop)
    if (not len(before) or h[before[-1]] or np.any(h[inside]) or
            np.max(np.diff(np.r_[ht[before[-1]], ht[inside], stop])) > 0.35):
        raise ValueError('hazard active or feedback missing during trial')
    grid = np.arange(start, stop, DT)
    # Inputs are held between command updates. Poses remain on their native sample times.
    indices = np.searchsorted(tc, grid, side='right') - 1
    keep = (ty >= grid[0]) & (ty <= grid[-1])
    if keep.sum() < 10:
        raise ValueError('too few actual feedback samples in excitation')
    return grid, u[indices], ty[keep], y[keep] - y[keep][0]


def response(run, tau, delay_steps):
    grid, u, t, _ = run
    delayed = np.zeros_like(u)
    if delay_steps:
        delayed[delay_steps:] = u[:-delay_steps]
    else:
        delayed[:] = u
    a = np.exp(-DT/tau)
    v = lfilter([0, 1-a], [1, -a], delayed)
    # Exact integration of one first-order ZOH interval at unit DC gain.
    increments = delayed*DT + (v-delayed)*tau*(1-a)
    pos = np.interp(t, grid, np.r_[0, np.cumsum(increments[:-1])])
    return pos - pos[0]


def fit(run):
    y = run[3]

    def solve(tau, delay):
        unit = response(run, tau, delay)
        norm = np.dot(unit, unit)
        if norm < 1e-12:
            return float('inf'), 0.0
        gain = np.dot(unit, y)/norm
        return np.mean((gain*unit-y)**2), gain

    best = (float('inf'),)
    for delay in range(31):
        result = minimize_scalar(lambda tau: solve(tau, delay)[0], bounds=(0.01, 2.0),
                                 method='bounded')
        cost, gain = solve(result.x, delay)
        if cost < best[0]:
            best = (cost, gain, result.x, delay)
    if len(best) != 4 or np.ptp(y) < 1e-5:
        raise ValueError('no measurable response')
    return best[1:]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('axis', choices=AXES)
    p.add_argument('--train', type=Path, required=True)
    p.add_argument('--valid', type=Path, required=True)
    p.add_argument('--out', type=Path, required=True, help='new output directory')
    p.add_argument('--odom-topic', default='/localization/odom')
    p.add_argument('--base-frame', default='base_link')
    for name in ('train', 'valid'):
        p.add_argument(f'--{name}-window', type=float, nargs=2, metavar=('START', 'STOP'))
    args = p.parse_args()
    if args.train.resolve() == args.valid.resolve():
        p.error('use independently recorded train and validation bags')
    try:
        runs = [read_run(path, args.axis, args.odom_topic, args.base_frame, window)
                for path, window in [(args.train, args.train_window),
                                     (args.valid, args.valid_window)]]
        gain, tau, delay = fit(runs[0])
        args.out.mkdir(parents=True, exist_ok=False)
    except (ValueError, RuntimeError, OSError) as exc:
        p.exit(1, f'{exc}\n')
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 1, figsize=(9, 6))
    unit = 'm' if args.axis in ('vx', 'vy') else 'rad'
    model = {'axis': args.axis, 'gain': gain, 'tau_s': tau, 'delay_s': delay*DT,
             'position_unit': unit, 'clock': 'bag receipt time', 'train_bag': str(args.train),
             'valid_bag': str(args.valid)}
    for label, run, ax in zip(('train', 'valid'), runs, axes):
        grid, u, t, y = run
        pred = gain*response(run, tau, delay)
        err = pred-y
        variation = np.linalg.norm(y-y.mean())
        model[label] = {'rmse': float(np.sqrt(np.mean(err**2))),
                        'fit_pct': float(100*(1-np.linalg.norm(err)/variation)) if variation else None}
        read_u = u[np.clip(np.searchsorted(grid, t, side='right')-1, 0, len(u)-1)]
        np.savetxt(args.out / f'{label}.csv', np.column_stack((t-grid[0], read_u, y, pred)),
                   delimiter=',', header='time_s,speed_command,measured_position,predicted_position',
                   comments='')
        ax.plot(t-grid[0], y, label='measured')
        ax.plot(t-grid[0], pred, '--', label='predicted')
        ax.set(title=f'{args.axis} / {label}', xlabel='time [s]', ylabel=f'displacement [{unit}]')
        ax.grid(alpha=0.3)
        ax.legend()
    fig.tight_layout()
    fig.savefig(args.out / 'response.png')
    plt.close(fig)
    payload = json.dumps(model, indent=2, allow_nan=False)
    (args.out / 'model.json').write_text(payload + '\n', encoding='utf-8')
    print(payload)


def hold_main():
    """Evaluate upper-body heading drift while the lower body turns."""
    p = argparse.ArgumentParser(description=hold_main.__doc__)
    p.add_argument('bag', type=Path)
    p.add_argument('--out', type=Path, required=True)
    p.add_argument('--odom-topic', default='/localization/odom')
    p.add_argument('--base-frame', default='base_link')
    p.add_argument('--band-deg', type=float, default=2.0)
    args = p.parse_args()
    if not np.isfinite(args.band_deg) or args.band_deg <= 0:
        p.error('--band-deg must be positive')
    imu = '/sensing/imu'
    try:
        grid, u, t_base, base = read_run(args.bag, 'wz', args.odom_topic, args.base_frame)
        angles = [(t, yaw(m.orientation)) for _, t, m in bag_messages(args.bag, [imu])
                  if grid[0] <= t <= grid[-1]]
        if len(angles) < 10:
            raise ValueError('no IMU heading during rotation')
        t, heading = np.array(angles).T
        if np.any(np.diff(t) < 0) or max(np.diff(np.r_[grid[0], t, grid[-1]])) > 0.2:
            raise ValueError('invalid or missing upper-body IMU timing')
        heading = np.rad2deg(np.unwrap(heading))
        active = np.flatnonzero(abs(u) > max(1e-4, 0.015*np.max(abs(u))))
        heading -= np.median(heading[t < grid[active[0]]])
        last = grid[active[-1]+1]
        outside = (t >= last) & (abs(heading) > args.band_deg)
        settled = t[np.flatnonzero(outside)[-1]+1:] if np.any(outside) else t[t >= last]
        recovery = None
        if len(settled) and t[-1]-settled[0] >= 0.5:
            recovery = float(settled[0]-last) if np.any(outside) else 0.0
        result = {'upper_peak_deg': float(max(abs(heading))),
                  'upper_rms_deg': float(np.sqrt(np.mean(heading**2))),
                  'band_deg': args.band_deg, 'recovery_after_stop_s': recovery}
        args.out.mkdir(parents=True, exist_ok=False)
    except (ValueError, RuntimeError, OSError) as exc:
        p.exit(1, f'{exc}\n')
    np.savetxt(args.out/'hold.csv', np.column_stack((t-grid[0], heading,
               np.rad2deg(np.interp(t, t_base, base)))), delimiter=',',
               header='time_s,upper_heading_deg,lower_heading_deg', comments='')
    payload = json.dumps(result, indent=2, allow_nan=False)
    (args.out/'hold.json').write_text(payload+'\n', encoding='utf-8')
    print(payload)
