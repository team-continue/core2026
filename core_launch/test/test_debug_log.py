"""Tests for debug_log_recorder.py and debug_log_timeline.py."""

import datetime
import os
from pathlib import Path
import re
import sys

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import pytest
from rcl_interfaces.msg import Log
from rclpy.serialization import serialize_message
import rosbag2_py
from std_msgs.msg import Bool, Int8MultiArray, String

PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR / 'scripts'))

import debug_log_recorder as recorder  # noqa: E402
import debug_log_timeline as timeline  # noqa: E402

MB = recorder.MB
NOW = datetime.datetime(2026, 11, 24, 0, 51, 13)


@pytest.fixture(scope='module')
def config():
    return recorder.load_config(PACKAGE_DIR / 'config' / 'debug_log.yaml')


def is_recorded(config, topic):
    # ros2 bag と同じく、include のどれかに部分一致し、exclude のどれにも当たらないもの
    include = re.compile(recorder.combine_patterns(config['include']))
    exclude = re.compile(recorder.combine_patterns(config['exclude']))
    return bool(include.search(topic)) and not exclude.search(topic)


@pytest.mark.parametrize('topic', [
    '/rosout',
    '/tf_static',
    '/system/emergency/hazard_status',
    '/system/emergency/hazard_label',
    '/system/health/diagnostics',
    '/bridge/connected',
    '/ui/manual_mode',
    '/ui/auto/route',
    '/behavior/state_name',
    '/behavior/waypoints',
    '/control/cmd_vel',
    '/hardware/can/tx',
    '/hardware/can/rx',
    '/hardware/joint_states',
    '/hardware/motor_status',
    '/hardware/wireless',
    '/mecha/shooter/left/aim_status',
    '/planning/planned_path',
    '/planning/waypoints',
    '/localization/odom',
    '/perception/enemy_detection/left/target_pose',
])
def test_default_config_records_key_topics(config, topic):
    assert is_recorded(config, topic)


@pytest.mark.parametrize('topic', [
    '/tf',
    '/imu',
    '/map',
    '/map/costmap/global',
    '/planning/costmap/local',
    '/localization/aligned_cloud',
    '/localization/global_map',
    '/livox/lidar',
    '/lidar/points_filtered',
    '/turret_camera_left/color/image',
    '/perception/enemy_detection/left/damage_panels_infomation',
    '/hardware/debug_image/compressed',
])
def test_default_config_skips_heavy_topics(config, topic):
    assert not is_recorded(config, topic)


def test_load_config_rejects_unknown_keys(tmp_path):
    path = tmp_path / 'bad.yaml'
    path.write_text("include: ['^/rosout$']\nmax_total_gb: 4\n")
    with pytest.raises(ValueError, match='max_total_gb'):
        recorder.load_config(path)


def make_session_dir(log_dir, name, size_bytes):
    path = log_dir / name
    path.mkdir(parents=True)
    (path / f'{name}_0.db3').write_bytes(b'\0' * size_bytes)
    return path


def test_list_sessions_orders_by_sequence_and_ignores_other_dirs(tmp_path):
    make_session_dir(tmp_path, '0010_20260101-000000', 3)
    make_session_dir(tmp_path, '0002_20991231-235959', 2)
    make_session_dir(tmp_path, 'my_bag', 1)
    (tmp_path / '0003_20260101-000000.txt').write_text('not a dir')
    os.symlink(tmp_path / 'my_bag', tmp_path / '0001_20260101-000000')

    sessions = recorder.list_sessions(tmp_path)

    assert [s.name for s in sessions] == ['0002_20991231-235959', '0010_20260101-000000']
    assert [s.size_bytes for s in sessions] == [2, 3]


def test_next_session_name():
    assert recorder.next_session_name([], NOW) == '0001_20261124-005113'
    sessions = [recorder.Session(3, 'a', 'a', 0), recorder.Session(12, 'b', 'b', 0)]
    assert recorder.next_session_name(sessions, NOW) == '0013_20261124-005113'


def sessions_of(sizes):
    return [recorder.Session(i + 1, str(i + 1), str(i + 1), size) for i, size in enumerate(sizes)]


def test_plan_pruning_keeps_everything_within_limits():
    to_delete, free = recorder.plan_pruning(sessions_of([10, 10]), 20, 5, 100)
    assert to_delete == []
    assert free == 100


def test_plan_pruning_deletes_oldest_until_total_fits():
    sessions = sessions_of([10, 20, 30])
    to_delete, free = recorder.plan_pruning(sessions, 35, 0, 100)
    assert to_delete == sessions[:2]
    assert free == 130


def test_plan_pruning_deletes_until_enough_free_space():
    sessions = sessions_of([10, 20, 30])
    to_delete, free = recorder.plan_pruning(sessions, 1000, 25, 0)
    assert to_delete == sessions[:2]
    assert free == 30


def test_plan_pruning_reports_when_free_space_cannot_be_reached():
    sessions = sessions_of([10])
    to_delete, free = recorder.plan_pruning(sessions, 1000, 50, 5)
    assert to_delete == sessions
    assert free == 15


def test_prepare_session_deletes_only_old_sessions(tmp_path, config):
    config = dict(config, max_total_mb=2, min_free_mb=0)
    make_session_dir(tmp_path, '0001_20261120-100000', MB)
    make_session_dir(tmp_path, '0002_20261121-100000', MB)
    make_session_dir(tmp_path, '0003_20261122-100000', MB)
    make_session_dir(tmp_path, 'keep_me', 5 * MB)
    logs = []

    path = recorder.prepare_session(
        config, str(tmp_path), NOW, free_bytes_fn=lambda _: 100 * MB, log=logs.append)

    assert path == str(tmp_path / '0004_20261124-005113')
    assert sorted(p.name for p in tmp_path.iterdir()) == [
        '0002_20261121-100000', '0003_20261122-100000', 'keep_me']
    assert len(logs) == 1 and '0001_20261120-100000' in logs[0]


def test_prepare_session_keeps_counting_after_deleting_every_session(tmp_path, config):
    config = dict(config, max_total_mb=0, min_free_mb=0)
    make_session_dir(tmp_path, '0007_20261120-100000', MB)

    path = recorder.prepare_session(
        config, str(tmp_path), NOW, free_bytes_fn=lambda _: 100 * MB, log=lambda _: None)

    assert path == str(tmp_path / '0008_20261124-005113')
    assert list(tmp_path.iterdir()) == []


def test_prepare_session_skips_recording_without_free_space(tmp_path, config):
    config = dict(config, min_free_mb=100)
    make_session_dir(tmp_path, '0001_20261120-100000', MB)
    logs = []

    path = recorder.prepare_session(
        config, str(tmp_path), NOW, free_bytes_fn=lambda _: 10 * MB, log=logs.append)

    assert path is None
    # 消しても足りないので、記録は消さずに残す
    assert [p.name for p in tmp_path.iterdir()] == ['0001_20261120-100000']
    assert len(logs) == 1 and 'not recording' in logs[0]


def test_build_record_command(config):
    cmd = recorder.build_record_command(config, '/logs/0001_x')

    assert cmd[:3] == ['ros2', 'bag', 'record']

    def value(flag):
        return cmd[cmd.index(flag) + 1]

    assert value('--output') == '/logs/0001_x'
    assert value('--storage-preset-profile') == 'resilient'
    assert value('--max-cache-size') == str(MB)
    assert value('--max-bag-duration') == '300'
    assert value('--regex') == recorder.combine_patterns(config['include'])
    assert value('--exclude') == recorder.combine_patterns(config['exclude'])

    no_split = recorder.build_record_command(dict(config, split_duration_s=0, exclude=[]), 'x')
    assert '--max-bag-duration' not in no_split
    assert '--exclude' not in no_split


def test_format_value():
    assert timeline.format_value(Bool(data=True)) == 'true'
    assert timeline.format_value(String(data='BRIDGE 切断')) == '"BRIDGE 切断"'
    assert timeline.format_value(Int8MultiArray(data=[0, 1, -1])) == '[0, 1, -1]'
    assert timeline.format_value(String(data='x' * 500)).endswith('...')


S = 1_000_000_000


def rosout(level, name, text):
    return Log(level=level, name=name, msg=text)


def diagnostics(*statuses):
    msg = DiagnosticArray()
    for name, level, text in statuses:
        msg.status.append(DiagnosticStatus(name=name, level=bytes([level]), message=text))
    return msg


def write_bag(path, messages):
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {}
    for topic, msg, _ in messages:
        if topic not in types:
            types[topic] = f'{type(msg).__module__.split(".")[0]}/msg/{type(msg).__name__}'
            writer.create_topic(rosbag2_py.TopicMetadata(
                name=topic, type=types[topic], serialization_format='cdr'))
    for topic, msg, stamp_ns in messages:
        writer.write(topic, serialize_message(msg), stamp_ns)
    del writer


BASE = 1_795_449_073 * S
BAG_MESSAGES = [
    ('/rosout', rosout(20, 'system.emergency.emergency_handler', 'started'), BASE),
    ('/system/emergency/hazard_label', String(data='NONE'), BASE + S),
    ('/system/emergency/hazard_label', String(data='NONE'), BASE + 2 * S),
    ('/control/cmd_vel_raw', String(data='not watched'), BASE + 2 * S),
    ('/system/health/diagnostics',
     diagnostics(('motor/waist', 0, ''), ('motor/wheel', 0, '')), BASE + 3 * S),
    ('/rosout', rosout(30, 'hardware.core_hardware', 'CAN timeout'), BASE + 4 * S),
    ('/system/health/diagnostics',
     diagnostics(('motor/waist', 0, ''), ('motor/wheel', 2, 'no response')), BASE + 5 * S),
    ('/system/emergency/hazard_label', String(data='BRIDGE DISCONNECTED'), BASE + 6 * S),
]


def test_collect_events_shows_warnings_and_changes_only(tmp_path):
    bag = tmp_path / 'bag'
    write_bag(bag, BAG_MESSAGES)

    start_ns, events = timeline.collect_events(str(bag), log=lambda _: None)

    assert start_ns == BASE
    assert events == [
        (BASE + S, '/system/emergency/hazard_label = "NONE"'),
        (BASE + 3 * S, '/system/health/diagnostics motor/waist = OK'),
        (BASE + 3 * S, '/system/health/diagnostics motor/wheel = OK'),
        (BASE + 4 * S, '[WARN] hardware.core_hardware: CAN timeout'),
        (BASE + 5 * S, '/system/health/diagnostics motor/wheel = ERROR "no response"'),
        (BASE + 6 * S, '/system/emergency/hazard_label = "BRIDGE DISCONNECTED"'),
    ]


def test_collect_events_level_and_extra_watch(tmp_path):
    bag = tmp_path / 'bag'
    write_bag(bag, BAG_MESSAGES)

    _, events = timeline.collect_events(
        str(bag), min_level=timeline.LEVELS['info'], watch=[r'^/control/'], log=lambda _: None)

    assert events == [
        (BASE, '[INFO] system.emergency.emergency_handler: started'),
        (BASE + 2 * S, '/control/cmd_vel_raw = "not watched"'),
        (BASE + 4 * S, '[WARN] hardware.core_hardware: CAN timeout'),
    ]


def test_collect_events_rebuilds_missing_metadata(tmp_path):
    # 急に電源を切ると metadata.yaml が書かれない
    bag = tmp_path / 'bag'
    write_bag(bag, BAG_MESSAGES)
    os.remove(bag / 'metadata.yaml')
    logs = []

    _, events = timeline.collect_events(str(bag), log=logs.append)

    assert (bag / 'metadata.yaml').exists()
    assert 'Reindexing' in logs[0]
    assert len(events) == 6


def test_main_reads_latest_session(tmp_path, capsys):
    write_bag(tmp_path / '0001_20261123-000000', BAG_MESSAGES[:2])
    write_bag(tmp_path / '0002_20261124-000000', BAG_MESSAGES)

    assert timeline.main(['--log-dir', str(tmp_path)]) == 0

    out = capsys.readouterr().out.splitlines()
    assert out[0].startswith(f'# {tmp_path / "0002_20261124-000000"}')
    assert len(out) == 7
    assert out[-1].endswith('+6.000s  /system/emergency/hazard_label = "BRIDGE DISCONNECTED"')
