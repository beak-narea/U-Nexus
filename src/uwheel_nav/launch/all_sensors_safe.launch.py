#!/usr/bin/env python3
"""all_sensors_launch.py (원본 수정 없이 그대로 포함) + 충돌 방지.

원본 대비 달라지는 점은 딱 하나:
  motor_driver_node 만 /cmd_vel 대신 /cmd_vel_safe 를 구독 (노드 한정 remap)

  아이패드 Foxglove ─/cmd_vel─▶ teleop_avoidance ─▶ collision_monitor ─/cmd_vel_safe─▶ motor_driver_node
  (avoid:=false 면 회피 없이 감속·정지만)

foxglove_bridge 등 다른 노드는 remap 영향 없음 (아이패드는 기존대로 /cmd_vel 발행).
collision_monitor 가 꺼져 있으면 모터에 명령이 안 가므로 로봇이 멈춘 상태로 안전하게 실패한다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_launch_description():
    all_sensors = LaunchConfiguration('all_sensors_launch')
    motor_node = LaunchConfiguration('motor_node_name')

    sensors_with_safe_motor = GroupAction(
        actions=[
            SetRemap(src=[motor_node, ':/cmd_vel'], dst='/cmd_vel_safe'),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(all_sensors)),
        ],
        scoped=True,
    )

    teleop_safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('uwheel_nav'), 'launch', 'teleop_safety.launch.py')),
        launch_arguments={'use_joy': LaunchConfiguration('use_joy'),
                          'avoid': LaunchConfiguration('avoid'),
                          'angular_sign': LaunchConfiguration('angular_sign')}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'all_sensors_launch',
            default_value=os.path.expanduser('~/U-Wheel_Nexus/all_sensors_launch.py')),
        DeclareLaunchArgument('motor_node_name', default_value='motor_driver_node'),
        DeclareLaunchArgument('use_joy', default_value='false'),
        DeclareLaunchArgument('avoid', default_value='true'),
        DeclareLaunchArgument('angular_sign', default_value='-1.0'),
        sensors_with_safe_motor,
        teleop_safety,
    ])
