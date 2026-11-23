#!/usr/bin/env python3
"""地図の PCD（lio_map_saver_node.py が保存したもの）から、navigation.launch.py で選べる地図のファイルをそろえる.

作るもの（--maps に、--name の名前で置く）:
- <name>.pcd: 入力の PCD の写し（入力が --maps/<name>.pcd ならそのまま）
- <name>.png・<name>.yaml: UI に出す占有地図（map_server の trinary）。床から --min-height〜--max-height に点のある
  セルが占有、走って見た床が空き、ほかは不明。地図照合が LIO を失ったときに捕まえ直す範囲にも使う
- <name>_field.yaml: フィールドの範囲（空きのセルの外接の長方形。--field で手で決められる）

PCD は床を z=0 にした座標を前提にする。作った後は core_launch をビルドし直してから、
navigation.launch.py environment:=real map_name:=<name> で起動する。
"""

from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import sys

from core_nav_launch.custom_map import custom_map_files, CustomMapError
from core_nav_map.map_files import (
    Bounds,
    free_bounds,
    MapFileError,
    occupancy_image,
    read_pcd,
    write_field_config,
    write_occupancy,
)
from core_nav_map.voxel_map import floor_height, floor_plane

# 床の傾きがこれより大きければ知らせる（10m 先で約 5cm。計画は床から 0.15m 以上の点を静止物にする）
TILT_WARN_DEG = 0.3


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('pcd', type=Path, help='地図の PCD（床が z=0）')
    parser.add_argument('--name', required=True, help='地図の名前（英数字と _ -）。map_name:=<name> で選ぶ')
    parser.add_argument('--maps', type=Path, required=True, help='置く場所（ソースの core2026/core_launch/maps）')
    parser.add_argument('--resolution', type=float, default=0.05, help='占有地図のセルの大きさ [m]')
    parser.add_argument('--min-height', type=float, default=0.15, help='占有にする高さの下限 [m]（床の凹凸を拾わない）')
    parser.add_argument('--max-height', type=float, default=1.5, help='占有にする高さの上限 [m]（天井を拾わない）')
    parser.add_argument('--floor-tolerance', type=float, default=0.08, help='床とみなす |z| の上限 [m]')
    parser.add_argument('--margin', type=float, default=1.0, help='占有地図の画像の、点の外側の余白 [m]')
    parser.add_argument('--free-fill', type=float, default=0.15,
                        help='床の点から空きにする距離 [m]（地図のボクセルより少し大きく）')
    parser.add_argument('--field', type=float, nargs=4, metavar=('X_MIN', 'X_MAX', 'Y_MIN', 'Y_MAX'), default=None,
                        help='フィールドの範囲 [m]（既定は空きのセルの外接の長方形。窓の外の床なども入るときに狭める）')
    parser.add_argument('--force', action='store_true', help='同じ名前のファイルがあれば書き換える')
    args = parser.parse_args()

    try:
        name = args.name
        maps = args.maps.resolve()
        # 名前とファイルの場所は navigation.launch.py が探すものと同じにする
        pcd_out, yaml_out, field_out = custom_map_files(maps, name)
        outputs = [pcd_out, maps / f'{name}.png', yaml_out, field_out]
        same_pcd = args.pcd.resolve() == pcd_out
        existing = [path.name for path in outputs if path.exists() and not (same_pcd and path == pcd_out)]
        if existing and not args.force:
            raise MapFileError(f'{maps} に同じ名前のファイルがあります（--force で書き換え）: {", ".join(existing)}')

        points = read_pcd(args.pcd)
        floor = floor_height(points)
        if floor is None or abs(floor) > args.floor_tolerance:
            raise MapFileError(
                f'床が z=0 にありません（床の高さの目安: {floor}）。lio_map_saver_node.py で保存した地図か、'
                '作り始めに機体が水平だったかを確かめてください')
        plane = floor_plane(points)
        image = occupancy_image(
            points, args.resolution, args.min_height, args.max_height, args.floor_tolerance, args.margin,
            args.free_fill)
        if args.field is None:
            bounds = free_bounds(image)
        else:
            x_min, x_max, y_min, y_max = args.field
            if not (x_min < x_max and y_min < y_max):
                raise MapFileError(f'--field は X_MIN < X_MAX、Y_MIN < Y_MAX にしてください: {args.field}')
            bounds = Bounds(x_min, x_max, y_min, y_max)
        if not same_pcd:
            maps.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(args.pcd, pcd_out)
        source = args.pcd.name
        write_occupancy(image, name, maps, source)
        write_field_config(bounds, field_out, source)
    except (MapFileError, CustomMapError, OSError) as error:
        print(f'[NG] {error}', file=sys.stderr)
        return 1
    height, width = image.pixels.shape
    print(f'点 {points.shape[0]} 個、床の高さの目安 {floor:+.3f}m')
    if plane is not None:
        print(f'床の傾き {plane.tilt_deg:.2f}°（原点の 15m 以内の床に当てはめた面）')
        if plane.tilt_deg > TILT_WARN_DEG:
            print(f'[注意] 床が {TILT_WARN_DEG}° より傾いています。遠くの床が静止物になるおそれがあります。'
                  '水平な所から作り直すか、占有地図（png）で床が黒くなっていないかを確かめてください')
    print(f'占有地図 {width} x {height} 画素（{args.resolution}m）、原点 ({image.origin_x:.2f}, {image.origin_y:.2f})')
    print(f'フィールドの範囲 x {bounds.x_min:.2f}〜{bounds.x_max:.2f}m、y {bounds.y_min:.2f}〜{bounds.y_max:.2f}m')
    print(f'作ったもの: {maps}/{name}(.pcd|.png|.yaml|_field.yaml)')
    print(f'次に core_launch をビルドし直し、navigation.launch.py environment:=real map_name:={name} で起動する')
    return 0


if __name__ == '__main__':
    sys.exit(main())
