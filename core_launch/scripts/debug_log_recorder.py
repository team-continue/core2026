#!/usr/bin/env python3
"""
Start ros2 bag record for the robot debug log after pruning old sessions.

The log directory holds one sub directory per launch, named
``<seq>_<YYYYmmdd-HHMMSS>``. The sequence number keeps the order even when
the robot clock is wrong. Only directories with this name are ever deleted.
"""

import argparse
from dataclasses import dataclass
import datetime
import os
import re
import shutil
import sys

import yaml

MB = 1024 * 1024
SESSION_DIR_RE = re.compile(r'^(\d{4,})_\d{8}-\d{6}$')
LOG_PREFIX = '[debug_log]'
DEFAULTS = {
    'include': [],
    'exclude': [],
    'max_total_mb': 4000,
    'min_free_mb': 2000,
    'split_duration_s': 300,
    'max_cache_mb': 1,
}


@dataclass(frozen=True)
class Session:
    seq: int
    name: str
    path: str
    size_bytes: int


def default_log_dir():
    return os.environ.get(
        'CORE_DEBUG_LOG_DIR',
        os.path.join(os.path.expanduser('~'), 'rosbag', 'core_debug_log'))


def load_config(path):
    with open(path, encoding='utf-8') as f:
        loaded = yaml.safe_load(f) or {}
    unknown = set(loaded) - set(DEFAULTS)
    if unknown:
        raise ValueError(f'unknown keys in {path}: {sorted(unknown)}')
    config = dict(DEFAULTS)
    config.update(loaded)
    if not config['include']:
        raise ValueError(f'{path}: include must not be empty')
    for pattern in config['include'] + config['exclude']:
        re.compile(pattern)
    return config


def combine_patterns(patterns):
    return '|'.join(f'(?:{p})' for p in patterns)


def dir_size_bytes(path):
    total = 0
    for root, _dirs, files in os.walk(path):
        for name in files:
            try:
                total += os.lstat(os.path.join(root, name)).st_size
            except OSError:
                pass
    return total


def list_sessions(log_dir):
    """Return the session directories in log_dir, oldest first."""
    sessions = []
    if not os.path.isdir(log_dir):
        return sessions
    for name in os.listdir(log_dir):
        match = SESSION_DIR_RE.match(name)
        path = os.path.join(log_dir, name)
        if match is None or os.path.islink(path) or not os.path.isdir(path):
            continue
        sessions.append(Session(int(match.group(1)), name, path, dir_size_bytes(path)))
    sessions.sort(key=lambda s: (s.seq, s.name))
    return sessions


def plan_pruning(sessions, max_total_bytes, min_free_bytes, free_bytes):
    """
    Pick the oldest sessions to delete.

    Deletes until the total is within max_total_bytes and the free space is at
    least min_free_bytes. Returns the sessions to delete and the free space
    expected afterwards.
    """
    total = sum(s.size_bytes for s in sessions)
    to_delete = []
    for session in sessions:
        if total <= max_total_bytes and free_bytes >= min_free_bytes:
            break
        to_delete.append(session)
        total -= session.size_bytes
        free_bytes += session.size_bytes
    return to_delete, free_bytes


def next_session_name(sessions, now):
    seq = max((s.seq for s in sessions), default=0) + 1
    return f'{seq:04d}_{now:%Y%m%d-%H%M%S}'


def build_record_command(config, output_dir):
    cmd = [
        'ros2', 'bag', 'record',
        '--output', output_dir,
        '--storage', 'sqlite3',
        # WAL で書き、急に電源を切っても直前まで読めるようにする
        '--storage-preset-profile', 'resilient',
        '--max-cache-size', str(int(config['max_cache_mb'] * MB)),
        '--node-name', 'debug_log_recorder',
        '--regex', combine_patterns(config['include']),
    ]
    if config['exclude']:
        cmd += ['--exclude', combine_patterns(config['exclude'])]
    if config['split_duration_s'] > 0:
        cmd += ['--max-bag-duration', str(int(config['split_duration_s']))]
    return cmd


def _log(message):
    print(message, flush=True)


def prepare_session(config, log_dir, now, free_bytes_fn=None, log=_log):
    """Delete old sessions and return the new session path, or None to skip."""
    if free_bytes_fn is None:
        def free_bytes_fn(path):
            return shutil.disk_usage(path).free

    os.makedirs(log_dir, exist_ok=True)
    sessions = list_sessions(log_dir)
    min_free_bytes = int(config['min_free_mb'] * MB)
    to_delete, expected_free = plan_pruning(
        sessions, int(config['max_total_mb'] * MB), min_free_bytes, free_bytes_fn(log_dir))
    if expected_free < min_free_bytes:
        # 全部消しても空きが足りないなら、調べたい記録を無駄に消さないよう何も消さない
        log(f'{LOG_PREFIX} not recording: {log_dir} would have only '
            f'{expected_free / MB:.0f} MB free even after deleting every old session '
            f'(min_free_mb: {config["min_free_mb"]})')
        return None
    for session in to_delete:
        log(f'{LOG_PREFIX} delete old session {session.name} '
            f'({session.size_bytes / MB:.1f} MB)')
        shutil.rmtree(session.path)
    # 消した記録も数に入れ、全部消したときに番号が 0001 に戻らないようにする
    return os.path.join(log_dir, next_session_name(sessions, now))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    parser.add_argument('--config', required=True, help='debug_log.yaml')
    parser.add_argument('--log-dir', default='', help='default: $CORE_DEBUG_LOG_DIR '
                        'or ~/rosbag/core_debug_log')
    args = parser.parse_args(argv)

    config = load_config(args.config)
    log_dir = os.path.abspath(os.path.expanduser(args.log_dir or default_log_dir()))
    output_dir = prepare_session(config, log_dir, datetime.datetime.now())
    if output_dir is None:
        # 記録できなくても機体の起動は止めない
        return 0

    cmd = build_record_command(config, output_dir)
    _log(f'{LOG_PREFIX} recording to {output_dir}')
    # 自分を ros2 bag に置き換え、launch からの SIGINT がそのまま届くようにする。
    # SIGINT で終われば metadata.yaml が書かれる
    os.execvp(cmd[0], cmd)
    return 1


if __name__ == '__main__':
    sys.exit(main())
