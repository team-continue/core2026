import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace

# ノード固有のパラメータはすべて config/shooter.params.yaml で管理する。
# 左右差分は同ファイル内の /**/<side>/<node> セクションで指定するため、
# ここではトピックの remap のみを行う。

HAZARD_REMAP = ("hazard_status", "/system/emergency/hazard_status")
CAN_REMAP = ("/can/tx", "/hardware/can/tx")
TEST_MODE_REMAP = ("/test_mode", "/ui/test_mode")


def _params_file():
    return os.path.join(
        get_package_share_directory("core_shooter"),
        "config",
        "shooter.params.yaml",
    )


# 曲射弾道の実測テーブル。左右で弾道が異なるので砲塔ごとに用意し、
# aim_bot へ絶対パスで渡す。
def _ballistics_file(side):
    return os.path.join(
        get_package_share_directory("core_shooter"),
        "config",
        f"ballistics_{side}.yaml",
    )


def _shooter_cmd_gate(params):
    return Node(
        package="core_shooter",
        executable="shooter_cmd_gate",
        name="shooter_cmd_gate",
        output="screen",
        parameters=[params],
        remappings=[
            ("manual_mode", "/ui/manual_mode"),
            ("manual_pitch", "/ui/manual_pitch"),
            ("shoot_motor_state", "/ui/shoot_motor_state"),
            ("left_manual_mode", "left/manual_mode"),
            ("left_manual_pitch_angle", "left/manual_pitch_angle"),
            ("right_manual_mode", "right/manual_mode"),
            ("right_manual_pitch_angle", "right/manual_pitch_angle"),
            ("left_shoot_cmd", "left/shoot_cmd"),
            ("right_shoot_cmd", "right/shoot_cmd"),
            ("/left/shoot_motor", "/mecha/shooter/left/shoot_motor"),
            ("/right/shoot_motor", "/mecha/shooter/right/shoot_motor"),
        ],
    )


def _shooter_controller(params):
    return Node(
        package="core_shooter",
        executable="shooter_controller",
        name="shooter_controller",
        output="screen",
        parameters=[params],
        remappings=[
            HAZARD_REMAP,
            CAN_REMAP,
            TEST_MODE_REMAP,
        ],
    )


def _magazine_manager(params):
    return Node(
        package="core_shooter",
        executable="magazine_manager",
        name="magazine_manager",
        output="screen",
        parameters=[params],
        remappings=[
            ("disk_distance_sensor", "distance"),
            HAZARD_REMAP,
            CAN_REMAP,
        ],
    )


def _aim_bot(params, side):
    return Node(
        package="core_shooter",
        executable="aim_bot",
        name="aim_bot",
        output="screen",
        parameters=[
            params,
            {
                "point3d.ballistic.table_path": _ballistics_file(side),
                "envelope.table_path": _envelope_file(side),
            },
        ],
        remappings=[
            HAZARD_REMAP,
            CAN_REMAP,
            TEST_MODE_REMAP,
            (
                "target_image_position",
                f"/perception/enemy_detection/{side}/target_pose",
            ),
            # target_input_mode="point3d" のときに購読する砲塔座標系の3次元ターゲット。
            # image モードと同じ target_pose トピックで届く（中身の解釈だけが違う）。
            # 購読は排他なので、同じトピックを二重に受けることはない。
            (
                "target_point_3d",
                f"/perception/enemy_detection/{side}/target_pose",
            ),
            # 自動射撃の有効/無効と、その引き金。
            # shoot_fullauto は shooter_cmd_gate が shoot_cmd へ変換する。
            ("turret_auto", f"/ui/{side}/turret_auto"),
            ("shoot_fullauto", f"/mecha/shooter/{side}/shoot_fullauto"),
        ],
    )


# 砲塔の可動包絡線。左右で機構制約が異なるので砲塔ごとに用意し、
# aim_bot へ絶対パスで渡す。
def _envelope_file(side):
    return os.path.join(
        get_package_share_directory("core_shooter"),
        "config",
        f"turret_envelope_{side}.yaml",
    )


def _side_group(params, side):
    return GroupAction([
        PushRosNamespace(side),
        _shooter_controller(params),
        _magazine_manager(params),
        _aim_bot(params, side),
    ])


def generate_launch_description():
    params = _params_file()

    return LaunchDescription([
        GroupAction([
            PushRosNamespace("mecha"),
            GroupAction([
                PushRosNamespace("shooter"),
                _shooter_cmd_gate(params),
                _side_group(params, "left"),
                _side_group(params, "right"),
            ]),
        ]),
    ])
