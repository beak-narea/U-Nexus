#!/usr/bin/env python3
"""조종 명령 충돌 방지 (start_all_sensors_safe.sh 에서 all_sensors_safe.launch.py 가 포함).

  아이패드 /cmd_vel ─[sign_in]─▶ /cmd_vel_ros ─▶ teleop_avoidance ─/cmd_vel_avoid─▶ collision_monitor
      ─/cmd_vel_safe_ros─[sign_out]─▶ /cmd_vel_safe ─▶ motor_driver
  /scan_g2b, /scan2, /lidar0|1/pointcloud ─▶ scan_merger ─/scan_merged─▶ (avoidance, collision_monitor)

sign_in / sign_out: 이 로봇은 모터에 angular.z>0 을 주면 오른쪽으로 돈다(ROS 규칙과 반대, 2026-09-14 실측).
  계산(회피/충돌)은 ROS 규칙으로 하고, 아이패드·모터 쪽 부호는 그대로 두기 위해 들어올 때/나갈 때 뒤집는다.
  배선이 ROS 규칙으로 고쳐지면 angular_sign:=1.0

avoid:=false 면 teleop_avoidance 없이 /cmd_vel_ros → collision_monitor (감속/정지만).
use_joy:=true 로 켜면 Orin 직결 조이스틱(teleop_twist_joy, ROS 규칙)이 /cmd_vel_ros 로 바로 발행.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('uwheel_nav'), 'config')
    joy_cfg = os.path.join(cfg, 'joy_teleop.yaml')
    use_joy = IfCondition(LaunchConfiguration('use_joy'))
    avoid = LaunchConfiguration('avoid')
    sign = PythonExpression(['float(', LaunchConfiguration('angular_sign'), ')'])
    cm_input = PythonExpression(["'/cmd_vel_avoid' if '", avoid, "'.lower() == 'true' else '/cmd_vel_ros'"])

    return LaunchDescription([
        DeclareLaunchArgument(
            'avoid', default_value='true',
            description='true: 장애물 앞에서 방향을 틀어 회피 / false: 감속·정지만'),
        DeclareLaunchArgument(
            'angular_sign', default_value='-1.0',
            description='모터 회전 부호 (-1: 이 로봇처럼 angular.z>0 이 오른쪽 회전, 1: ROS 규칙)'),
        DeclareLaunchArgument(
            'use_joy', default_value='false',
            description='Orin 에 직접 연결한 조이스틱 사용 (아이패드 Foxglove 조종이면 false)'),
        Node(
            package='joy',
            executable='game_controller_node',
            name='joy_node',
            parameters=[joy_cfg],
            output='screen',
            condition=use_joy,
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[joy_cfg],
            remappings=[('/cmd_vel', '/cmd_vel_ros')],
            output='screen',
            condition=use_joy,
        ),
        Node(
            package='uwheel_nav',
            executable='twist_sign_fix',
            name='cmd_sign_in',
            parameters=[{'input_topic': '/cmd_vel', 'output_topic': '/cmd_vel_ros', 'angular_sign': sign}],
            output='screen',
        ),
        Node(
            package='uwheel_nav',
            executable='scan_merger',
            name='scan_merger',
            output='screen',
        ),
        Node(
            package='uwheel_nav',
            executable='teleop_avoidance',
            name='teleop_avoidance',
            parameters=[{'input_topic': '/cmd_vel_ros'}],
            output='screen',
            condition=IfCondition(avoid),
        ),
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            parameters=[os.path.join(cfg, 'collision_monitor.yaml'),
                        {'cmd_vel_in_topic': cm_input, 'cmd_vel_out_topic': '/cmd_vel_safe_ros'}],
            output='screen',
        ),
        Node(
            package='uwheel_nav',
            executable='twist_sign_fix',
            name='cmd_sign_out',
            parameters=[{'input_topic': '/cmd_vel_safe_ros', 'output_topic': '/cmd_vel_safe', 'angular_sign': sign}],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_collision_monitor',
            parameters=[{'autostart': True, 'node_names': ['collision_monitor']}],
            output='screen',
        ),
    ])
