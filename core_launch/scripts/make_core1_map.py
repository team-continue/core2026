#!/usr/bin/env python3
"""CoRE-1 のフィールドの形（fields/core1_2027_top.json）から、試験のフィールドの地図を作る.

作るもの（実機と sim で同じものを使う）:
- 構造物の外形（maps/<name>_outlines.yaml。名前・高さ・上から見た多角形）。sim の接触の判定と、sim（core_field_sim）の
  make_core1_field_model.py（外形を押し出した Gazebo のフィールドのモデル）が使う
- 地図照合の PCD（構造物・会場・床の面に点を並べたもの。会場の壁や天井などは --venue のワールドから写す）
- UI に出す占有地図（PNG と map_server の yaml）
- フィールドの設定（範囲、静止物の高さの下限、入らない範囲＝進入禁止ゾーン）

--mesh にフィールドの 3D データ（fields/core1_2027_field.stl。m、z が上、フィールドの角が原点で
x 0〜24・y −15〜0）を渡すと、構造物は押し出しの代わりにそのメッシュを使う（壁の厚さ、高台の坂、
置き場の段などが図面の近似より正確）。PCD はメッシュの面に点を並べ、占有地図と構造物の外形は、静止物の
高さの下限より高い所を輪切りにして作る。進入禁止ゾーンとフィールドの設定は、どちらでも --top から作る。
STL は公式の配布で配布の条件が分からないので git に入れず、メッシュ版は別の名前で作る（git の管理外）:
  --mesh fields/core1_2027_field.stl --name core1_2027_mesh
（navigation.launch.py の map_name:=core1_2027_mesh で選べる）

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
# 静止物として使う点の高さの下限 [m]（陣地の羽根 0.08m・フェンス 0.14m・バンカー 0.15m を拾う。床の点は拾わない）。
# PCD の高さは float なので、羽根の上の面の点は 0.08 より僅かに低い（0.0799…）。羽根の高さより低くしておく
STATIC_MIN_HEIGHT_M: Final = 0.07
# 3D データ（STL）の原点（フィールドの角）を map の座標へ移す量 [m]
MESH_OFFSET_M: Final = (-FIELD_LENGTH_M / 2.0, FIELD_WIDTH_M / 2.0)
# メッシュの面に並べる点の間隔、外形を作る格子 [m]
MESH_STEP_M: Final = 0.025
OUTLINE_RESOLUTION_M: Final = 0.025
# 外形を作る範囲（フィールドの外の置き場なども入れる）と、外形とみなす一番高い点 [m]
OUTLINE_BOUNDS_M: Final = (-13.0, 13.0, -9.0, 9.0)
OUTLINE_MAX_HEIGHT_M: Final = 2.0
# 外形を切る区画の大きさ [m]
OUTLINE_TILE_M: Final = 1.0
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


# ---------------------------------------------------------------- 3D データ（STL）

def load_stl(path: Path) -> np.ndarray:
    """binary STL の三角形（n × 3 頂点 × xyz）を map の座標で返す."""
    data = path.read_bytes()
    if len(data) < 84:
        raise FieldError(f'{path} は STL として短すぎます')
    count = struct.unpack('<I', data[80:84])[0]
    if 84 + count * 50 != len(data):
        raise FieldError(f'{path} は binary STL ではありません（三角形 {count} 個と長さ {len(data)} が合わない）')
    record = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    triangles = np.frombuffer(data, dtype=record, count=count, offset=84)['vertices'].astype(np.float64)
    triangles[:, :, 0] += MESH_OFFSET_M[0]
    triangles[:, :, 1] += MESH_OFFSET_M[1]
    return triangles


def sample_mesh(triangles: np.ndarray, step: float) -> np.ndarray:
    """三角形の面に、間隔 step 以下の格子で点を並べる（step の立方体ごとに 1 点にまとめる）.

    床の板（上の面が z = 0 以下）は除く。床の点は floor_points で並べる。
    """
    triangles = triangles[triangles[:, :, 2].max(axis=1) > 1e-3]
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    longest = np.max(np.stack([
        np.linalg.norm(b - a, axis=1), np.linalg.norm(c - a, axis=1), np.linalg.norm(c - b, axis=1)]), axis=0)
    divisions = np.maximum(np.ceil(longest / step).astype(int), 1)
    points = []
    for k in np.unique(divisions):
        chosen = divisions == k
        i, j = np.meshgrid(np.arange(k + 1), np.arange(k + 1))
        keep = i + j <= k
        u = i[keep] / k
        v = j[keep] / k
        ta, tb, tc = a[chosen], b[chosen], c[chosen]
        grid = ta[:, None, :] + u[None, :, None] * (tb - ta)[:, None, :] + v[None, :, None] * (tc - ta)[:, None, :]
        points.append(grid.reshape(-1, 3))
    cloud = np.concatenate(points)
    _, first = np.unique(np.floor(cloud / step).astype(np.int64), axis=0, return_index=True)
    return cloud[np.sort(first)]


def outline_name(
    centers: list[tuple[str, tuple[float, float], Polygon]], cx: float, cy: float, max_distance: float = 1.5
) -> str:
    """点 (cx, cy) を含む図面の構造物の名前。なければ max_distance 以内で重心が一番近いもの、それもなければ mesh."""
    for name, _, polygon in centers:
        if inside(polygon, np.array([cx]), np.array([cy]))[0]:
            return name
    nearest = min(centers, key=lambda item: math.hypot(item[1][0] - cx, item[1][1] - cy), default=None)
    if nearest is not None and math.hypot(nearest[1][0] - cx, nearest[1][1] - cy) <= max_distance:
        return nearest[0]
    return 'mesh'


def mesh_outlines(cloud: np.ndarray, prisms: list[Prism]) -> list[Prism]:
    """静止物の高さの下限より高い点がある所を輪切りにした外形.

    柵と壁がつながって床を囲むと、外形の穴（囲まれた床）を埋められないので、OUTLINE_TILE_M 四方の区画ごとに
    切って外形を作る（区画の中に収まる小さな穴だけが残る。埋めて、その面積を知らせる）。
    名前は、外形の重心を含む（なければ近い）図面の構造物から付ける（接触の報告で場所が分かるように）。
    高さは外形の中の一番高い点。
    """
    # 外形を作るときだけ使う。rosdep のキーがないので package.xml には書かず、使う手元で
    # sudo apt install python3-contourpy で入れる
    try:
        import contourpy
    except ImportError as error:
        raise FieldError('--mesh には contourpy が要ります（sudo apt install python3-contourpy）') from error

    xmin, xmax, ymin, ymax = OUTLINE_BOUNDS_M
    res = OUTLINE_RESOLUTION_M
    nx = int(round((xmax - xmin) / res))
    ny = int(round((ymax - ymin) / res))
    high = cloud[(cloud[:, 2] >= STATIC_MIN_HEIGHT_M) & (cloud[:, 2] <= OUTLINE_MAX_HEIGHT_M)]
    ix = np.floor((high[:, 0] - xmin) / res).astype(int)
    iy = np.floor((high[:, 1] - ymin) / res).astype(int)
    ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    occupied = np.zeros((ny, nx))
    occupied[iy[ok], ix[ok]] = 1.0
    tile = int(round(OUTLINE_TILE_M / res))
    centers = [(p.name, centroid(p.polygon), p.polygon) for p in prisms]
    used: dict[str, int] = {}
    result: list[Prism] = []
    filled_area = 0.0
    for ty in range(0, ny, tile):
        for tx in range(0, nx, tile):
            block = occupied[ty:ty + tile, tx:tx + tile]
            if not block.any():
                continue
            padded = np.pad(block, 1)
            xs = xmin + (tx + np.arange(padded.shape[1]) - 0.5) * res
            ys = ymin + (ty + np.arange(padded.shape[0]) - 0.5) * res
            generator = contourpy.contour_generator(xs, ys, padded, fill_type=contourpy.FillType.OuterOffset)
            boundaries, offsets = generator.filled(0.5, 2.0)
            for points, offset in zip(boundaries, offsets):
                rings = [points[offset[i]:offset[i + 1]] for i in range(len(offset) - 1)]
                outer = drop_collinear([(float(x), float(y)) for x, y in rings[0][:-1]])
                if len(outer) < 3:
                    continue
                filled_area += sum(polygon_area(ring) for ring in rings[1:])
                cx, cy = centroid(outer)
                name = outline_name(centers, cx, cy)
                used[name] = used.get(name, 0) + 1
                unique = name if used[name] == 1 else f'{name}_part{used[name]}'
                mask = inside(outer, high[:, 0], high[:, 1])
                height = float(high[mask, 2].max()) if mask.any() else STATIC_MIN_HEIGHT_M
                result.append(Prism(unique, outer, height))
    if not result:
        raise FieldError('メッシュから構造物の外形を作れませんでした')
    if filled_area > 0.0:
        print(f'[注意] 外形の中に囲まれた床 {filled_area:.3f} m² を構造物として埋めました')
    return result


def drop_collinear(polygon: Polygon, tolerance: float = 1e-9) -> Polygon:
    """一直線に並んだ頂点を除く（格子の外形は同じ向きの辺が続くので、頂点が多くなる）."""
    result = list(polygon)
    changed = True
    while changed and len(result) > 3:
        changed = False
        kept: Polygon = []
        count = len(result)
        for i in range(count):
            ax, ay = kept[-1] if kept else result[i - 1]
            bx, by = result[i]
            cx, cy = result[(i + 1) % count]
            if abs((bx - ax) * (cy - by) - (by - ay) * (cx - bx)) <= tolerance:
                changed = True
                continue
            kept.append((bx, by))
        result = kept if len(kept) >= 3 else result
    return result


def polygon_area(ring: np.ndarray) -> float:
    x, y = ring[:, 0], ring[:, 1]
    return 0.5 * abs(float(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1))))


def write_outlines(outlines: list[Prism], out: Path) -> None:
    document = {'outlines': [
        {'name': p.name, 'height_m': round(p.height, 3),
         'points': [[round(x, 4), round(y, 4)] for x, y in p.polygon]} for p in outlines]}
    out.write_text(
        '# make_core1_map.py が作った構造物の外形（--mesh のときは 3D データの、静止物の高さの下限より高い所の輪切り）。\n'
        '# run_scenario.py の接触の判定と、sim（core_field_sim）の make_core1_field_model.py が使う。手で直さず、スクリプトで作り直す\n'
        + yaml.safe_dump(document, sort_keys=False, allow_unicode=True, default_flow_style=None), encoding='utf-8')


# ---------------------------------------------------------------- 会場

def venue_models(venue_world: Path) -> tuple[str, list[ET.Element]]:
    """会場のワールドから、柵と試験用の箱を除いた部分（文字列）と、残した模型を返す."""
    text = venue_world.read_text(encoding='utf-8')
    removed = re.compile(r'\s*<model name="(fence_[a-z]+|static_box_[a-z])">.*?</model>', re.S)
    text = removed.sub('', text)
    root = ET.fromstring(text)
    models = [m for m in root.iter('model') if m.get('name') != 'ground']
    return text, models


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
    # 側面の点は上の縁（高さ prism.height）にも置く。間隔で刻むだけだと、陣地の羽根（0.08m）は 0・0.05m の
    # 点しかなく、静止物の高さの下限（0.07m）より低いので外形が地図に入らず、腕の先端が約 0.1m 欠けていた
    heights = np.arange(0.0, prism.height + 1e-9, SURFACE_STEP_M)
    if heights[-1] < prism.height - 1e-9:
        heights = np.append(heights, prism.height)
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
        f'# make_core1_map.py が作った、UI に出す占有地図（構造物と進入禁止ゾーンが占有）\n'
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
        '# CoRE-1 の試験のフィールド（make_core1_map.py が作る。手で直さず、スクリプトで作り直す）。\n'
        '# navigation.launch.py の map_name:=core1_2027 と sim_nav.launch.py の field_file で config/field.yaml に重ねる。\n'
        '# - static_map.min_height_m: 陣地の羽根 0.08m・フェンス 0.14m・バンカー 0.15m を静止物として拾う\n'
        '# - keep_out: 進入禁止ゾーン（3.2）。計画はここを壁として扱う\n'
        + yaml.safe_dump(document, sort_keys=False, allow_unicode=True), encoding='utf-8')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--top', type=Path, required=True, help='fields/core1_2027_top.json')
    parser.add_argument('--venue', type=Path, required=True, help='会場を写すワールド（worlds/field_static.sdf）')
    parser.add_argument('--maps', type=Path, required=True, help='外形・PCD・占有地図を置く場所（core_launch/maps）')
    parser.add_argument('--config', type=Path, required=True, help='作るフィールドの設定')
    parser.add_argument('--name', default='core1_2027')
    parser.add_argument(
        '--mesh', type=Path, default=None,
        help='フィールドの 3D データ（fields/core1_2027_field.stl）。渡すと構造物はこのメッシュにする')
    args = parser.parse_args()
    try:
        prisms, keep_out = load_field(args.top)
        if args.mesh is not None:
            structure = sample_mesh(load_stl(args.mesh), MESH_STEP_M)
            outlines = mesh_outlines(structure, prisms)
        else:
            structure = np.concatenate([sample_prism(p) for p in prisms])
            outlines = prisms
        write_outlines(outlines, args.maps / f'{args.name}_outlines.yaml')
        cloud = np.concatenate([structure, venue_points(args.venue), floor_points(-29.5, 29.5, -19.0, 19.0)])
        write_pcd(cloud, args.maps / f'{args.name}.pcd')
        write_occupancy(outlines, keep_out, args.name, args.maps)
        write_field_config(keep_out, args.config)
    except (FieldError, OSError, ET.ParseError, KeyError, ValueError) as error:
        print(f'[NG] {error}', file=sys.stderr)
        return 1
    kinds: dict[str, int] = {}
    for prism in prisms:
        kind = prism.name.rsplit('_', 1)[0]
        kinds[kind] = kinds.get(kind, 0) + 1
    source = f'メッシュ {args.mesh}（外形 {len(outlines)} 個）' if args.mesh is not None else '押し出し'
    print(f'構造物 {len(prisms)} 個 {kinds}（{source}）、進入禁止ゾーン {len(keep_out)} 個、PCD {len(cloud)} 点')
    print(f'作ったもの: {args.maps}/{args.name}(_outlines.yaml|.pcd|.png|.yaml), {args.config}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
