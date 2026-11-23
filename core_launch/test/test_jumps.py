import math

import numpy as np
import pytest

from core_nav_check.jumps import (
    Alignment,
    Band,
    compose,
    Crowd,
    JumpError,
    Level,
    map_pose_errors,
    Motion,
    parse_alignment,
    PlanarPose,
    Record,
    steps,
    summarize,
    time_base_gap,
)


def test_parse_alignment_reads_values():
    values = {
        'has_converged': 'true', 'fitness_score': '0.12', 'correction_translation_m': '0.03',
        'correction_yaw_deg': '0.4', 'consecutive_rejected_updates': '2',
    }
    assert parse_alignment(1.0, values) == Alignment(1.0, True, 0.12, 0.03, 0.4, 2)


def test_parse_alignment_reads_motion_gate():
    values = {
        'has_converged': 'true', 'fitness_score': '0.12', 'correction_translation_m': '0.3',
        'correction_yaw_deg': '4.0', 'consecutive_rejected_updates': '0', 'motion_gate_active': 'true',
    }
    alignment = parse_alignment(1.0, values)
    assert alignment is not None and alignment.motion_gated


def test_parse_alignment_skips_status_without_result():
    assert parse_alignment(1.0, {'map_received': 'true'}) is None


def test_parse_alignment_reports_bad_value():
    values = {
        'has_converged': 'true', 'fitness_score': 'x', 'correction_translation_m': '0',
        'correction_yaw_deg': '0', 'consecutive_rejected_updates': '0',
    }
    with pytest.raises(JumpError):
        parse_alignment(1.0, values)


def test_steps_measure_changes_between_updates():
    samples = [PlanarPose(0.0, 0.0, 0.0, 0.0), PlanarPose(0.1, 0.03, 0.04, 0.0),
               PlanarPose(0.1, 0.03, 0.04, 0.0), PlanarPose(0.2, 0.03, 0.04, math.radians(2.0))]
    t, step_m, step_rad = steps(samples)
    assert t.tolist() == [0.1, 0.2]
    assert step_m == pytest.approx([0.05, 0.0])
    assert np.degrees(step_rad) == pytest.approx([0.0, 2.0])


def test_steps_measure_robot_jump_not_map_odom_translation():
    # 機体が odom の原点から 10m 先にいるとき、map→odom を原点まわりに 0.5 度回すと、map→odom の並進は
    # 変わらないが、機体の地図上の位置は約 8.7cm 動く
    yaw = math.radians(0.5)
    samples = [PlanarPose(0.0, 0.0, 0.0, 0.0), PlanarPose(0.1, 0.0, 0.0, yaw)]
    robot = [PlanarPose(0.05, 10.0, 0.0, 0.0)]
    _, step_m, _ = steps(samples, robot)
    assert step_m == pytest.approx([10.0 * math.hypot(math.cos(yaw) - 1.0, math.sin(yaw))])
    # 逆に、原点から離れた機体の位置を保つように map→odom の並進が大きく変わっても、機体は跳ばない
    shifted = [PlanarPose(0.0, 0.0, 0.0, 0.0),
               PlanarPose(0.1, 10.0 - 10.0 * math.cos(yaw), -10.0 * math.sin(yaw), yaw)]
    _, step_m, _ = steps(shifted, robot)
    assert step_m == pytest.approx([0.0], abs=1e-9)


def test_steps_wrap_yaw_across_pi():
    samples = [PlanarPose(0.0, 0.0, 0.0, math.pi - 0.01), PlanarPose(0.1, 0.0, 0.0, -math.pi + 0.01)]
    _, _, step_rad = steps(samples)
    assert step_rad == pytest.approx([0.02])


def test_compose_rotates_then_translates():
    pose = compose(PlanarPose(0.0, 1.0, 0.0, math.pi / 2), PlanarPose(0.0, 1.0, 0.0, 0.0), 0.0)
    assert (pose.x, pose.y, pose.yaw) == pytest.approx((1.0, 1.0, math.pi / 2))


def test_map_pose_errors_use_latest_correction():
    map_odom = [PlanarPose(0.0, 0.0, 0.0, 0.0), PlanarPose(1.0, 0.1, 0.0, 0.0)]
    odom_base = [PlanarPose(0.5, 1.0, 0.0, 0.0), PlanarPose(1.5, 2.0, 0.0, 0.0)]
    truth = [PlanarPose(0.0, 0.0, 0.0, 0.0), PlanarPose(2.0, 2.0, 0.0, 0.0)]
    t, errors = map_pose_errors(map_odom, odom_base, truth)
    # 0.5 秒: 真値 0.5、推定 1.0。1.5 秒: 真値 1.5、推定 2.1
    assert t.tolist() == [0.5, 1.5]
    assert errors == pytest.approx([0.5, 0.6])


def test_summarize_splits_by_motion_and_counts_rejections():
    record = Record(
        map_odom=[PlanarPose(0.0, 0.0, 0.0, 0.0), PlanarPose(1.0, 0.02, 0.0, 0.0),
                  PlanarPose(2.0, 0.10, 0.0, 0.0)],
        odom_base=[],
        motion=[Motion(0.0, 0.0, 0.0), Motion(1.5, 0.0, 4.0)],
        alignments=[Alignment(1.0, True, 0.1, 0.02, 0.1, 0), Alignment(2.0, True, 0.1, 0.08, 0.1, 3)],
        levels=[Level(0.0, 0), Level(1.8, 1)],
        crowd=[Crowd(0.0, 0)],
        truth=[],
        end=3.0,
    )
    bands = (Band('全体'), Band('静止', max_speed=0.1, max_yaw_rate=0.2), Band('速い旋回', min_yaw_rate=3.0))
    summary = summarize(record, bands)
    assert summary['全体']['段差_最大_cm'] == pytest.approx(8.0)
    assert summary['静止']['段差_最大_cm'] == pytest.approx(2.0)
    assert summary['速い旋回']['段差_最大_cm'] == pytest.approx(8.0)
    assert summary['速い旋回']['捨てた照合_回'] == 1.0
    assert summary['速い旋回']['続けて捨てた_最大_回'] == 3.0
    assert summary['速い旋回']['DEGRADED_s'] == pytest.approx(1.2)
    assert summary['静止']['時間_s'] == pytest.approx(1.5)
    assert math.isnan(summary['全体']['位置の誤差_最大_cm'])


def test_time_base_gap_compares_last_stamps():
    motion = [Motion(10.0, 0.0, 0.0), Motion(20.0, 0.0, 0.0)]
    assert time_base_gap([Level(19.8, 0)], motion) == pytest.approx(-0.2)
    # シミュレーションの rosbag を --use-sim-time なしで録って再生したとき
    assert time_base_gap([Level(1794445702.7, 0)], motion) > 1e9
    assert time_base_gap([], motion) == 0.0
