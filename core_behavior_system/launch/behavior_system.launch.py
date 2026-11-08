from launch import LaunchDescription

# attack_shoot_manager は core_shooter へ移設したため、この launch から起動するノードはない。
# 砲塔の引き金判断は照準・弾道と同じパッケージで完結させる方針による。
# 起動対象は core_shooter/launch/shooter.launch.py を参照。
#
# behavior_system / waypoint_selector / enemy_detection_coordinator は
# 以前からこの launch の起動対象ではない。


def generate_launch_description():
    return LaunchDescription([])
