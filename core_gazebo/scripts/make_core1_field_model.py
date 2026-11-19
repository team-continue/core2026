#!/usr/bin/env python3
"""CoRE-1 の試験のフィールドを、Gazebo のモデル（URDF）とワールドにする.

作るもの:
- フィールドのモデル（--models/<name>_field/: model.config と model.urdf）。構造物を 1 つのリンクにまとめた
  静止物で、見た目と当たり判定は同じメッシュ
  - --outlines: core_launch の make_core1_map.py が作る構造物の外形（core_launch/maps/<name>_outlines.yaml）を
    高さで押し出したメッシュ（meshes/<name>_field.stl）を作って使う
  - --mesh-uri: フィールドの 3D データ（STL）をそのまま使う。原点（フィールドの角）を map の座標へ移して置く
- ワールド（--world）。会場のワールド（--venue）から柵と試験用の箱を除き、フィールドのモデルを読み込む
  （<include>。sim.launch.py が core_gazebo の models を GZ_SIM_RESOURCE_PATH に足す）

メッシュの URI は package://（RViz でも同じ URDF を表示できる）。sim.launch.py が core_gazebo と core_launch の
share の親を GZ_SIM_RESOURCE_PATH に足すので、Gazebo も見つけられる。

使い方（core2026 の直下で）:
  押し出し: --outlines core_launch/maps/core1_2027_outlines.yaml --name core1_2027
  3D データ（git の管理外）: --mesh-uri package://core_launch/fields/core1_2027_field.stl --name core1_2027_mesh
  共通: --venue core_gazebo/worlds/field_static.sdf --world core_gazebo/worlds/<name>.sdf --models core_gazebo/models
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import struct
import sys
from typing import Final
import xml.etree.ElementTree as ET

import numpy as np
import yaml

# 3D データ（STL）の原点（フィールドの角）を map の座標へ移す量 [m]（core_launch の make_core1_map.py の
# MESH_OFFSET_M と同じ。フィールドの長さ 24m・幅 15m の半分）
MESH_OFFSET_M: Final = (-12.0, 7.5)
# 静止物の質量と慣性（<inertial> がないリンクは URDF から SDF にするとき消えるので付ける。static なので値は使われない）
STATIC_MASS_KG: Final = 1000.0

Polygon = list[tuple[float, float]]


class FieldError(Exception):
    pass


# ---------------------------------------------------------------- 外形の押し出し

def load_outlines(path: Path) -> list[tuple[str, Polygon, float]]:
    """make_core1_map.py の外形（outlines: [{name, height_m, points}]）を読む."""
    try:
        outlines = yaml.safe_load(path.read_text(encoding='utf-8'))['outlines']
    except (OSError, yaml.YAMLError, KeyError, TypeError) as error:
        raise FieldError(f'構造物の外形 {path} を読めません: {error}') from error
    result = []
    for outline in outlines:
        points = [(float(x), float(y)) for x, y in outline['points']]
        height = float(outline['height_m'])
        if len(points) < 3 or height <= 0.0:
            raise FieldError(f'{path}: {outline["name"]} の外形か高さが正しくありません')
        result.append((str(outline['name']), points, height))
    if not result:
        raise FieldError(f'{path} に外形がありません')
    return result


def signed_area(polygon: Polygon) -> float:
    return 0.5 * sum(
        ax * by - bx * ay for (ax, ay), (bx, by) in zip(polygon, polygon[1:] + polygon[:1]))


def triangulate(polygon: Polygon, name: str) -> list[tuple[int, int, int]]:
    """穴のない多角形（反時計回り）を耳の切り落としで三角形にする（頂点の番号の組）."""
    def cross(o: tuple[float, float], a: tuple[float, float], b: tuple[float, float]) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    def contains(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float],
                 p: tuple[float, float]) -> bool:
        return cross(a, b, p) >= 0.0 and cross(b, c, p) >= 0.0 and cross(c, a, p) >= 0.0

    remaining = list(range(len(polygon)))
    triangles: list[tuple[int, int, int]] = []
    while len(remaining) > 3:
        for k in range(len(remaining)):
            i, j, m = remaining[k - 1], remaining[k], remaining[(k + 1) % len(remaining)]
            a, b, c = polygon[i], polygon[j], polygon[m]
            if cross(a, b, c) <= 1e-12:
                continue  # 凹んだ角か一直線
            others = (polygon[n] for n in remaining if n not in (i, j, m))
            if any(contains(a, b, c, p) for p in others):
                continue
            triangles.append((i, j, m))
            remaining.pop(k)
            break
        else:
            # 耳がないのは、一直線に並んだ頂点が残ったとき。面積がないので除いて続ける
            flat = [k for k in range(len(remaining)) if abs(cross(
                polygon[remaining[k - 1]], polygon[remaining[k]],
                polygon[remaining[(k + 1) % len(remaining)]])) <= 1e-12]
            if not flat:
                raise FieldError(f'{name} の外形を三角形に分けられません（辺が交わっていないか確かめてください）')
            remaining.pop(flat[0])
    triangles.append((remaining[0], remaining[1], remaining[2]))
    return triangles


def extrude(polygon: Polygon, height: float, name: str) -> np.ndarray:
    """外形を床（z = 0）から height まで押し出した閉じた立体の三角形（n × 3 頂点 × xyz。外向きが表）."""
    ring = polygon[:-1] if polygon[0] == polygon[-1] else list(polygon)
    if signed_area(ring) < 0.0:
        ring = ring[::-1]
    faces = []
    for i, j, m in triangulate(ring, name):
        a, b, c = ring[i], ring[j], ring[m]
        faces.append([(*a, height), (*b, height), (*c, height)])  # 上の面
        faces.append([(*a, 0.0), (*c, 0.0), (*b, 0.0)])  # 下の面
    for (ax, ay), (bx, by) in zip(ring, ring[1:] + ring[:1]):
        faces.append([(ax, ay, 0.0), (bx, by, 0.0), (bx, by, height)])
        faces.append([(ax, ay, 0.0), (bx, by, height), (ax, ay, height)])
    return np.array(faces, dtype=np.float64)


def write_stl(triangles: np.ndarray, out: Path) -> None:
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = np.divide(normals, lengths, out=np.zeros_like(normals), where=lengths > 0.0)
    record = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    data = np.zeros(len(triangles), dtype=record)
    data['normal'] = normals
    data['vertices'] = triangles
    header = b'make_core1_field_model.py: CoRE-1 field structures extruded from outlines'.ljust(80, b' ')
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_bytes(header + struct.pack('<I', len(triangles)) + data.tobytes())


# ---------------------------------------------------------------- フィールドのモデル（URDF）

def field_urdf(model: str, mesh_uri: str, offset: tuple[float, float], source: str) -> str:
    geometry = f'<geometry><mesh filename="{mesh_uri}"/></geometry>'
    origin = f'<origin xyz="{offset[0]:.3f} {offset[1]:.3f} 0" rpy="0 0 0"/>'
    return (
        '<?xml version="1.0"?>\n'
        f'<!-- CoRE-1 の試験のフィールドの構造物（{source}）。make_core1_field_model.py が作る。'
        '手で直さず、スクリプトで作り直す -->\n'
        f'<robot name="{model}">\n'
        '  <link name="field">\n'
        f'    <inertial><mass value="{STATIC_MASS_KG:.1f}"/>'
        '<inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>\n'
        f'    <visual>{origin}{geometry}'
        '<material name="field_grey"><color rgba="0.7 0.7 0.72 1"/></material></visual>\n'
        f'    <collision>{origin}{geometry}</collision>\n'
        '  </link>\n'
        '  <gazebo><static>true</static></gazebo>\n'
        '</robot>\n'
    )


def write_model(models: Path, model: str, urdf: str) -> Path:
    directory = models / model
    directory.mkdir(parents=True, exist_ok=True)
    (directory / 'model.urdf').write_text(urdf, encoding='utf-8')
    (directory / 'model.config').write_text(
        '<?xml version="1.0"?>\n'
        f'<model>\n  <name>{model}</name>\n  <version>1.0</version>\n'
        '  <sdf version="1.9">model.urdf</sdf>\n'
        '  <description>CoRE-1 の試験のフィールドの構造物（make_core1_field_model.py が作る）</description>\n'
        '</model>\n', encoding='utf-8')
    return directory


# ---------------------------------------------------------------- ワールド

def write_world(venue_world: Path, model: str, out: Path) -> None:
    """会場のワールドから柵と試験用の箱を除き、フィールドのモデルを読み込む."""
    text = venue_world.read_text(encoding='utf-8')
    text = re.sub(r'\s*<model name="(fence_[a-z]+|static_box_[a-z])">.*?</model>', '', text, flags=re.S)
    ET.fromstring(text)  # 壊れていないか
    if '</world>' not in text:
        raise FieldError(f'{venue_world} に </world> がありません')
    generated = (
        '\n    <!-- 以下は make_core1_field_model.py が CoRE-1 のフィールドから作ったもの。'
        '手で直さず、スクリプトで作り直す -->\n'
        f'    <include>\n      <uri>model://{model}</uri>\n      <name>core1_field</name>\n    </include>\n'
    )
    text = text.replace('</world>', generated + '  </world>', 1)
    text = text.replace(
        '<world name="field">',
        '<!-- CoRE-1 の試験のフィールド（make_core1_field_model.py で作る） -->\n  <world name="field">', 1)
    out.write_text(text, encoding='utf-8')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--outlines', type=Path, help='押し出す構造物の外形（core_launch/maps/<name>_outlines.yaml）')
    source.add_argument('--mesh-uri', help='フィールドの 3D データの URI（package://core_launch/fields/core1_2027_field.stl）')
    parser.add_argument('--venue', type=Path, required=True, help='会場のワールド（worlds/field_static.sdf）')
    parser.add_argument('--world', type=Path, required=True, help='作るワールド')
    parser.add_argument('--models', type=Path, required=True, help='フィールドのモデルを置く場所（core_gazebo/models）')
    parser.add_argument('--name', default='core1_2027')
    args = parser.parse_args()
    model = f'{args.name}_field'
    try:
        if args.outlines is not None:
            outlines = load_outlines(args.outlines)
            triangles = np.concatenate([extrude(points, height, name) for name, points, height in outlines])
            write_stl(triangles, args.models / model / 'meshes' / f'{model}.stl')
            urdf = field_urdf(
                model, f'package://core_gazebo/models/{model}/meshes/{model}.stl', (0.0, 0.0),
                f'{args.outlines.name} の外形 {len(outlines)} 個を押し出したもの')
            summary = f'外形 {len(outlines)} 個を押し出し（三角形 {len(triangles)} 個）'
        else:
            urdf = field_urdf(model, args.mesh_uri, MESH_OFFSET_M, f'3D データ {args.mesh_uri}')
            summary = f'3D データ {args.mesh_uri}'
        directory = write_model(args.models, model, urdf)
        write_world(args.venue, model, args.world)
    except (FieldError, OSError, ET.ParseError, KeyError, ValueError) as error:
        print(f'[NG] {error}', file=sys.stderr)
        return 1
    print(f'フィールドのモデル {model}（{summary}）')
    print(f'作ったもの: {directory}/, {args.world}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
