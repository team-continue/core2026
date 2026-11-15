#!/usr/bin/env python3
"""CoRE-1 のフィールドの形（fields/core1_2027_top.json）から、試験のフィールドを作る.

作るもの:
- Gazebo のワールド（構造物を高さで押し出した立体。会場の壁や天井などは --venue のワールドから写す）
- 地図照合の PCD（構造物・会場・床の面に点を並べたもの）
- UI に出す占有地図（PNG と map_server の yaml）
- フィールドの設定（範囲、静止物の高さの下限、入らない範囲＝進入禁止ゾーン）

座標: 図は mm で y が下向き（左下が赤サイド）。map・world は m で、フィールドの中心を原点、x は図の右向き、
y は図の上向き（X = x / 1000 - 12、Y = 7.5 - y / 1000）。赤サイドは map の左下（x < 0, y < 0）になる。

高さは本文と図面の値。置き場の台と陣地の羽根は図面から読んだおおよその値で、形は上から見た外形を
そのまま押し出す（台の脚の間や棚の段は埋まった立体になる）。動かせる競技用品（コンテナ・ブロック・
ブロックキャリアー）と床の線は置かない。
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
import struct
import sys
from typing import Final
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw
import yaml

FIELD_LENGTH_M: Final = 24.0
FIELD_WIDTH_M: Final = 15.0

# 高さ [m]（本文と図面の値）
FENCE_HEIGHT_M: Final = 0.14       # 3.2
HIGHLAND_HEIGHT_M: Final = 0.20    # 3.6
BUNKER_HEIGHT_M: Final = 0.15      # 3.6
WALL_HIGH_M: Final = 1.0           # 表3-3: スポットと置き場 A の後ろ
WALL_LOW_M: Final = 0.84           # 表3-3: 置き場 B・C の後ろ、共通置き場の中央
SPOT_HEIGHT_M: Final = 0.95        # 3.4: 上の段 850mm と棚の枠（図面からのおおよその値）
HOLDER_HEIGHT_M: Final = 0.45      # 図3-3: 台の高さ 300〜450mm
BASE_TOWER_HEIGHT_M: Final = 1.085  # 図3-11
BASE_FLAP_HEIGHT_M: Final = 0.08   # 図3-11: 陣地の周りの羽根（おおよその値）
# 壁の部品が、スポットか置き場 A からこの距離 [m] 以内なら高い壁（1000mm）
HIGH_WALL_NEAR_M: Final = 1.5

# 地図照合の PCD の点の間隔 [m]
SURFACE_STEP_M: Final = 0.05
TOP_STEP_M: Final = 0.1
FLOOR_STEP_M: Final = 0.2
# 静止物として使う点の高さの下限 [m]（フェンス 0.14m・バンカー 0.15m を拾う。床の点は拾わない）
STATIC_MIN_HEIGHT_M: Final = 0.08
# UI の占有地図
MAP_RESOLUTION_M: Final = 0.05
MAP_MARGIN_M: Final = 1.0

Polygon = list[tuple[float, float]]


@dataclass(frozen=True)
class Prism:
    name: str
    polygon: Polygon  # map 座標
    height: float


class FieldError(Exception):
    pass


def subpaths(path: str) -> list[Polygon]:
    """SVG のパス（M・L・Z だけ）を部品ごとの頂点の並びにする（図の mm 座標）."""
    result: list[Polygon] = []
    current: Polygon = []
    for command, x, y in re.findall(r'([MLZ])\s*(-?[\d.]+)?\s*(-?[\d.]+)?', path):
        if command == 'M':
            if len(current) >= 3:
                result.append(current)
            current = [(float(x), float(y))]
        elif command == 'L':
            current.append((float(x), float(y)))
        else:
            if len(current) >= 3:
                result.append(current)
            current = []
    if len(current) >= 3:
        result.append(current)
    return result


def to_map(polygon: Polygon) -> Polygon:
    return [(x / 1000.0 - FIELD_LENGTH_M / 2.0, FIELD_WIDTH_M / 2.0 - y / 1000.0) for x, y in polygon]


def centroid(polygon: Polygon) -> tuple[float, float]:
    xs = [p[0] for p in polygon]
    ys = [p[1] for p in polygon]
    return (sum(xs) / len(xs), sum(ys) / len(ys))


def load_field(path: Path) -> tuple[list[Prism], list[Polygon]]:
    """押し出す構造物と、入らない範囲（進入禁止ゾーン）を作る."""
    document = json.loads(path.read_text(encoding='utf-8'))
    elements = document['elements']
    near_high_wall = [
        centroid(to_map(polygon))
        for element in elements if element['class'] in ('m-sp', 'm-ha')
        for polygon in subpaths(element['path'])
    ]
    prisms: list[Prism] = []
    keep_out: list[Polygon] = []
    counts: dict[str, int] = {}

    def add(kind: str, polygon: Polygon, height: float) -> None:
        counts[kind] = counts.get(kind, 0) + 1
        prisms.append(Prism(f'{kind}_{counts[kind]}', polygon, height))

    for element in elements:
        parts = [to_map(polygon) for polygon in subpaths(element['path'])]
        cls = element['class']
        if cls == 'm-ne':
            keep_out.extend(parts)
        elif cls == 'm-fe':
            for polygon in parts:
                add('fence', polygon, FENCE_HEIGHT_M)
        elif cls == 'm-hl':
            for polygon in parts:
                add('highland', polygon, HIGHLAND_HEIGHT_M)
        elif cls == 'm-bk':
            for polygon in parts:
                add('bunker', polygon, BUNKER_HEIGHT_M)
        elif cls == 'm-wl':
            for polygon in parts:
                cx, cy = centroid(polygon)
                high = any(math.hypot(cx - x, cy - y) <= HIGH_WALL_NEAR_M for x, y in near_high_wall)
                add('wall', polygon, WALL_HIGH_M if high else WALL_LOW_M)
        elif cls == 'm-sp':
            for polygon in parts:
                add('spot', polygon, SPOT_HEIGHT_M)
        elif cls in ('m-ha', 'm-hbc', 'm-cm'):
            for polygon in parts:
                add('holder', polygon, HOLDER_HEIGHT_M)
        elif cls == 'm-ba':
            # いちばん頂点の多い外形が羽根、八角形が中央の塔。ほかはダメージパネルの取付部で省く
            largest = max(len(polygon) for polygon in parts)
            for polygon in parts:
                if len(polygon) == largest:
                    add('base_flap', polygon, BASE_FLAP_HEIGHT_M)
                elif len(polygon) == 8:
                    add('base_tower', polygon, BASE_TOWER_HEIGHT_M)
    if not prisms or not keep_out:
        raise FieldError(f'{path} から構造物か進入禁止ゾーンを作れませんでした')
    return prisms, keep_out


# ---------------------------------------------------------------- Gazebo のワールド

def prism_model(prism: Prism) -> str:
    points = ''.join(f'<point>{x:.4f} {y:.4f}</point>' for x, y in prism.polygon)
    geometry = f'<geometry><polyline>{points}<height>{prism.height:.3f}</height></polyline></geometry>'
    return (
        f'    <model name="{prism.name}">\n'
        f'      <static>true</static>\n'
        f'      <link name="link">\n'
        f'        <collision name="collision">{geometry}</collision>\n'
        f'        <visual name="visual">{geometry}</visual>\n'
        f'      </link>\n'
        f'    </model>\n'
    )


def venue_models(venue_world: Path) -> tuple[str, list[ET.Element]]:
    """会場のワールドから、柵と試験用の箱を除いた部分（文字列）と、残した模型を返す."""
    text = venue_world.read_text(encoding='utf-8')
    removed = re.compile(r'\s*<model name="(fence_[a-z]+|static_box_[a-z])">.*?</model>', re.S)
    text = removed.sub('', text)
    root = ET.fromstring(text)
    models = [m for m in root.iter('model') if m.get('name') != 'ground']
    return text, models


def write_world(venue_world: Path, prisms: list[Prism], out: Path) -> None:
    text, _ = venue_models(venue_world)
    generated = (
        '\n    <!-- 以下は make_core1_field.py が CoRE-1 のフィールドの形から作ったもの。'
        '手で直さず、スクリプトで作り直す -->\n' + ''.join(prism_model(p) for p in prisms)
    )
    if '</world>' not in text:
        raise FieldError(f'{venue_world} に </world> がありません')
    text = text.replace('</world>', generated + '  </world>', 1)
    text = text.replace(
        '<world name="field">',
        '<!-- CoRE-1 の試験のフィールド（make_core1_field.py で作る） -->\n  <world name="field">', 1)
    out.write_text(text, encoding='utf-8')


# ---------------------------------------------------------------- 地図照合の PCD

def inside(polygon: Polygon, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    result = np.zeros(x.shape, dtype=bool)
    count = len(polygon)
    for i in range(count):
        ax, ay = polygon[i]
        bx, by = polygon[i - 1]
        crosses = (ay > y) != (by > y)
        with np.errstate(divide='ignore', invalid='ignore'):
            at = ax + (y - ay) * (bx - ax) / (by - ay)
        result ^= crosses & (x < at)
    return result


def sample_prism(prism: Prism) -> np.ndarray:
    points = []
    heights = np.arange(0.0, prism.height + 1e-9, SURFACE_STEP_M)
    for i in range(len(prism.polygon)):
        ax, ay = prism.polygon[i - 1]
        bx, by = prism.polygon[i]
        length = math.hypot(bx - ax, by - ay)
        steps = np.linspace(0.0, 1.0, max(int(length / SURFACE_STEP_M), 1) + 1)
        xs = ax + (bx - ax) * steps
        ys = ay + (by - ay) * steps
        gx, gz = np.meshgrid(np.arange(len(xs)), heights)
        points.append(np.stack([xs[gx.ravel()], ys[gx.ravel()], gz.ravel()], axis=1))
    xs = [p[0] for p in prism.polygon]
    ys = [p[1] for p in prism.polygon]
    gx, gy = np.meshgrid(
        np.arange(min(xs), max(xs), TOP_STEP_M), np.arange(min(ys), max(ys), TOP_STEP_M))
    gx = gx.ravel()
    gy = gy.ravel()
    mask = inside(prism.polygon, gx, gy)
    points.append(np.stack([gx[mask], gy[mask], np.full(mask.sum(), prism.height)], axis=1))
    return np.concatenate(points)


def box_surface(center: np.ndarray, size: np.ndarray) -> np.ndarray:
    points = []
    half = size / 2.0
    for axis in range(3):
        others = [a for a in range(3) if a != axis]
        u = np.arange(-half[others[0]], half[others[0]] + 1e-9, TOP_STEP_M * 2)
        v = np.arange(-half[others[1]], half[others[1]] + 1e-9, TOP_STEP_M * 2)
        gu, gv = np.meshgrid(u, v)
        for sign in (-1.0, 1.0):
            face = np.zeros((gu.size, 3))
            face[:, axis] = sign * half[axis]
            face[:, others[0]] = gu.ravel()
            face[:, others[1]] = gv.ravel()
            points.append(face + center)
    return np.concatenate(points)


def venue_points(venue_world: Path) -> np.ndarray:
    _, models = venue_models(venue_world)
    points = []
    for model in models:
        pose = [float(v) for v in (model.findtext('pose') or '0 0 0 0 0 0').split()]
        if any(abs(v) > 1e-9 for v in pose[3:5]):
            raise FieldError(f'会場の {model.get("name")} は水平でないので扱えません')
        center = np.array(pose[:3])
        yaw = pose[5]
        box = model.find('.//visual/geometry/box/size')
        cylinder = model.find('.//visual/geometry/cylinder')
        if box is not None:
            local = box_surface(np.zeros(3), np.array([float(v) for v in box.text.split()]))
            c, s_ = math.cos(yaw), math.sin(yaw)
            rotated = np.stack([
                c * local[:, 0] - s_ * local[:, 1], s_ * local[:, 0] + c * local[:, 1], local[:, 2]], axis=1)
            points.append(rotated + center)
        elif cylinder is not None:
            radius = float(cylinder.findtext('radius'))
            length = float(cylinder.findtext('length'))
            angles = np.arange(0.0, 2.0 * math.pi, TOP_STEP_M * 2 / radius)
            zs = np.arange(-length / 2.0, length / 2.0, TOP_STEP_M * 2)
            ga, gz = np.meshgrid(angles, zs)
            points.append(np.stack([
                center[0] + radius * np.cos(ga.ravel()), center[1] + radius * np.sin(ga.ravel()),
                center[2] + gz.ravel()], axis=1))
    return np.concatenate(points) if points else np.zeros((0, 3))


def floor_points(xmin: float, xmax: float, ymin: float, ymax: float) -> np.ndarray:
    gx, gy = np.meshgrid(np.arange(xmin, xmax, FLOOR_STEP_M), np.arange(ymin, ymax, FLOOR_STEP_M))
    return np.stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)], axis=1)


def write_pcd(points: np.ndarray, out: Path) -> None:
    data = np.zeros((len(points), 4), dtype=np.float32)
    data[:, :3] = points
    header = (
        '# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z intensity\n'
        'SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n'
        f'WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(points)}\nDATA binary\n'
    )
    out.write_bytes(header.encode('ascii') + data.tobytes())


# ---------------------------------------------------------------- UI の占有地図

def write_occupancy(prisms: list[Prism], keep_out: list[Polygon], name: str, out_dir: Path) -> None:
    xmin = -FIELD_LENGTH_M / 2.0 - MAP_MARGIN_M
    ymin = -FIELD_WIDTH_M / 2.0 - MAP_MARGIN_M
    width = int(round((FIELD_LENGTH_M + 2 * MAP_MARGIN_M) / MAP_RESOLUTION_M))
    height = int(round((FIELD_WIDTH_M + 2 * MAP_MARGIN_M) / MAP_RESOLUTION_M))

    def pixels(polygon: Polygon) -> list[tuple[float, float]]:
        return [((x - xmin) / MAP_RESOLUTION_M, height - (y - ymin) / MAP_RESOLUTION_M) for x, y in polygon]

    image = Image.new('L', (width, height), 205)  # 不明
    draw = ImageDraw.Draw(image)
    field = [
        (-FIELD_LENGTH_M / 2, -FIELD_WIDTH_M / 2), (FIELD_LENGTH_M / 2, -FIELD_WIDTH_M / 2),
        (FIELD_LENGTH_M / 2, FIELD_WIDTH_M / 2), (-FIELD_LENGTH_M / 2, FIELD_WIDTH_M / 2)]
    draw.polygon(pixels(field), fill=254)  # 空き
    # 進入禁止ゾーンも、UI で目標を置かない所として占有にする
    for polygon in [p.polygon for p in prisms] + keep_out:
        draw.polygon(pixels(polygon), fill=0)
    image.save(out_dir / f'{name}.png')
    (out_dir / f'{name}.yaml').write_text(
        f'# make_core1_field.py が作った、UI に出す占有地図（構造物と進入禁止ゾーンが占有）\n'
        f'image: {name}.png\nresolution: {MAP_RESOLUTION_M}\norigin: [{xmin}, {ymin}, 0.0]\n'
        f'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n', encoding='utf-8')


# ---------------------------------------------------------------- フィールドの設定

def write_field_config(keep_out: list[Polygon], out: Path) -> None:
    xs = [round(x, 4) for polygon in keep_out for x, _ in polygon]
    ys = [round(y, 4) for polygon in keep_out for _, y in polygon]
    document = {'/**': {'ros__parameters': {
        'field': {
            'x_min': -FIELD_LENGTH_M / 2, 'x_max': FIELD_LENGTH_M / 2,
            'y_min': -FIELD_WIDTH_M / 2, 'y_max': FIELD_WIDTH_M / 2},
        'static_map': {'min_height_m': STATIC_MIN_HEIGHT_M},
        'keep_out': {'x': xs, 'y': ys, 'counts': [len(p) for p in keep_out]},
    }}}
    out.write_text(
        '# CoRE-1 の試験のフィールド（make_core1_field.py が作る。手で直さず、スクリプトで作り直す）。\n'
        '# navigation.launch.py の map_name:=core1_2027 と sim_nav.launch.py の field_file で config/field.yaml に重ねる。\n'
        '# - static_map.min_height_m: フェンス 0.14m・バンカー 0.15m を静止物として拾う\n'
        '# - keep_out: 進入禁止ゾーン（3.2）。計画はここを壁として扱う\n'
        + yaml.safe_dump(document, sort_keys=False, allow_unicode=True), encoding='utf-8')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--top', type=Path, required=True, help='fields/core1_2027_top.json')
    parser.add_argument('--venue', type=Path, required=True, help='会場を写すワールド（worlds/field_static.sdf）')
    parser.add_argument('--world', type=Path, required=True, help='作るワールド')
    parser.add_argument('--maps', type=Path, required=True, help='PCD と占有地図を置く場所（core_launch/maps）')
    parser.add_argument('--config', type=Path, required=True, help='作るフィールドの設定')
    parser.add_argument('--name', default='core1_2027')
    args = parser.parse_args()
    try:
        prisms, keep_out = load_field(args.top)
        write_world(args.venue, prisms, args.world)
        cloud = np.concatenate(
            [sample_prism(p) for p in prisms] + [venue_points(args.venue), floor_points(-29.5, 29.5, -19.0, 19.0)])
        write_pcd(cloud, args.maps / f'{args.name}.pcd')
        write_occupancy(prisms, keep_out, args.name, args.maps)
        write_field_config(keep_out, args.config)
    except (FieldError, OSError, ET.ParseError, KeyError, ValueError) as error:
        print(f'[NG] {error}', file=sys.stderr)
        return 1
    kinds: dict[str, int] = {}
    for prism in prisms:
        kind = prism.name.rsplit('_', 1)[0]
        kinds[kind] = kinds.get(kind, 0) + 1
    print(f'構造物 {len(prisms)} 個 {kinds}、進入禁止ゾーン {len(keep_out)} 個、PCD {len(cloud)} 点')
    print(f'作ったもの: {args.world}, {args.maps}/{args.name}.(pcd|png|yaml), {args.config}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
