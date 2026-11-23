#!/usr/bin/env python3
"""
Print what happened in a robot debug log, in time order.

Shows /rosout messages at or above --level, and the value of the watched
topics only when it changes. Without a bag path, reads the latest session in
the debug log directory.
"""

import argparse
import array
import datetime
import json
import os
import re
import sys

from debug_log_recorder import combine_patterns, default_log_dir, list_sessions, MB
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

ROSOUT_TOPIC = '/rosout'
LEVELS = {'debug': 10, 'info': 20, 'warn': 30, 'error': 40, 'fatal': 50}
LEVEL_NAMES = {value: key.upper() for key, value in LEVELS.items()}
DIAG_LEVEL_NAMES = {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}
# 変わったときに値を出すトピック
DEFAULT_WATCH = [
    r'^/system/emergency/',
    r'^/system/health/diagnostics$',
    r'^/bridge/',
    r'^/ui/(manual_mode|test_mode)$',
    r'^/behavior/(state_name|goal_reached)$',
    r'^/hardware/(destroy|hardware_emergency|hp)$',
]
MAX_VALUE_CHARS = 160


def truncate(text, limit=MAX_VALUE_CHARS):
    return text if len(text) <= limit else text[:limit - 3] + '...'


def format_scalar(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, float):
        return f'{value:.6g}'
    if isinstance(value, str):
        return json.dumps(value, ensure_ascii=False)
    if isinstance(value, (bytes, bytearray)):
        return format_scalar(list(value))
    if isinstance(value, (list, tuple, array.array)) or hasattr(value, 'tolist'):
        items = value.tolist() if hasattr(value, 'tolist') else list(value)
        return '[' + ', '.join(format_scalar(v) for v in items) + ']'
    return str(value)


def format_value(msg):
    fields = set(msg.get_fields_and_field_types())
    # std_msgs の Bool・String・*MultiArray などは data だけを出す
    if fields == {'data'} or fields == {'layout', 'data'}:
        text = format_scalar(msg.data)
    else:
        text = json.dumps(message_to_ordereddict(msg), ensure_ascii=False,
                          separators=(',', ':'), default=str)
    return truncate(text)


def diagnostic_level(level):
    if isinstance(level, (bytes, bytearray)):
        level = level[0]
    return DIAG_LEVEL_NAMES.get(int(level), str(level))


def watched_values(topic, msg):
    """Yield (key, value) pairs whose changes are reported."""
    if type(msg).__name__ == 'DiagnosticArray':
        # 項目ごとに、レベルかメッセージが変わったときだけ出す
        for status in msg.status:
            value = diagnostic_level(status.level)
            if status.message:
                value += ' ' + json.dumps(status.message, ensure_ascii=False)
            yield f'{topic} {status.name}', truncate(value)
        return
    yield topic, format_value(msg)


def format_rosout(msg):
    level = LEVEL_NAMES.get(msg.level, str(msg.level))
    return f'[{level}] {msg.name}: {msg.msg}'


def ensure_metadata(bag_path, log):
    """Rebuild metadata.yaml when the recorder did not stop cleanly."""
    if not os.path.isdir(bag_path) or os.path.exists(os.path.join(bag_path, 'metadata.yaml')):
        return
    log(f'metadata.yaml is missing (power cut or still recording). Reindexing {bag_path}')
    rosbag2_py.Reindexer().reindex(rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'))


def open_reader(bag_path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    return reader


def collect_events(bag_path, min_level=LEVELS['warn'], watch=None, log=None):
    """Return (start_ns, [(stamp_ns, text), ...]) for the bag."""
    if log is None:
        def log(message):
            print(message, file=sys.stderr)
    watch = DEFAULT_WATCH if watch is None else watch
    ensure_metadata(bag_path, log)
    reader = open_reader(bag_path)
    start_ns = reader.get_metadata().starting_time.nanoseconds

    watch_re = re.compile(combine_patterns(watch)) if watch else None
    msg_types = {}
    for topic in reader.get_all_topics_and_types():
        if topic.name != ROSOUT_TOPIC and not (watch_re and watch_re.search(topic.name)):
            continue
        try:
            msg_types[topic.name] = get_message(topic.type)
        except (AttributeError, ModuleNotFoundError, ValueError):
            log(f'skip {topic.name}: message type {topic.type} is not available')
    if not msg_types:
        return start_ns, []
    reader.set_filter(rosbag2_py.StorageFilter(topics=sorted(msg_types)))

    events = []
    last_values = {}
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        msg = deserialize_message(data, msg_types[topic])
        if topic == ROSOUT_TOPIC:
            if msg.level >= min_level:
                events.append((stamp_ns, format_rosout(msg)))
            continue
        for key, value in watched_values(topic, msg):
            if last_values.get(key) != value:
                last_values[key] = value
                events.append((stamp_ns, f'{key} = {value}'))
    return start_ns, events


def format_line(stamp_ns, start_ns, text):
    clock = datetime.datetime.fromtimestamp(stamp_ns / 1e9).strftime('%H:%M:%S.%f')[:-3]
    return f'{clock} {(stamp_ns - start_ns) / 1e9:+9.3f}s  {text}'


def print_sessions(log_dir):
    sessions = list_sessions(log_dir)
    if not sessions:
        print(f'no sessions in {log_dir}')
    for session in sessions:
        has_metadata = os.path.exists(os.path.join(session.path, 'metadata.yaml'))
        note = '' if has_metadata else '  (no metadata.yaml)'
        print(f'{session.name}  {session.size_bytes / MB:8.1f} MB{note}')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('bag', nargs='?', help='bag directory (default: the latest session)')
    parser.add_argument('--log-dir', default='', help='default: $CORE_DEBUG_LOG_DIR '
                        'or ~/rosbag/core_debug_log')
    parser.add_argument('--level', choices=sorted(LEVELS, key=LEVELS.get), default='warn',
                        help='lowest /rosout level to show (default: warn)')
    parser.add_argument('--watch', action='append', default=[], metavar='REGEX',
                        help='also show changes of topics matching REGEX (repeatable)')
    parser.add_argument('--list', action='store_true', help='list the sessions and exit')
    args = parser.parse_args(argv)

    log_dir = os.path.abspath(os.path.expanduser(args.log_dir or default_log_dir()))
    if args.list:
        print_sessions(log_dir)
        return 0

    bag_path = args.bag
    if bag_path is None:
        sessions = list_sessions(log_dir)
        if not sessions:
            print(f'no sessions in {log_dir}', file=sys.stderr)
            return 1
        bag_path = sessions[-1].path

    start_ns, events = collect_events(
        bag_path, min_level=LEVELS[args.level], watch=DEFAULT_WATCH + args.watch)
    start = datetime.datetime.fromtimestamp(start_ns / 1e9)
    print(f'# {bag_path}  (start {start:%Y-%m-%d %H:%M:%S})')
    for stamp_ns, text in events:
        print(format_line(stamp_ns, start_ns, text))
    return 0


if __name__ == '__main__':
    sys.exit(main())
