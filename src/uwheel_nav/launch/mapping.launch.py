#!/usr/bin/env python3
"""2D 지도 작성. start_all_sensors_safe.sh(센서+충돌 방지)가 켜진 상태에서 실행.

    FAST-LIO (fastlio_nav.yaml, frame 이름 fastlio_*) ─/Odometry─▶ fastlio_odom_bridge ─TF odom->base_link
  + slam_toolbox (/scan_merged) ─TF map->odom, /map

지도 저장: ros2 run nav2_map_server map_saver_cli -f ~/maps/<이름>
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('uwheel_nav')
    cfg = os.path.join(share, 'config')

    fast_lio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='laser_mapping',
        parameters=[os.path.join(cfg, 'fastlio_nav.yaml')],
        output='screen',
    )

    odom_bridge = Node(
        package='uwheel_nav',
        executable='fastlio_odom_bridge',
        name='fastlio_odom_bridge',
        output='screen',
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={
            'slam_params_file': os.path.join(cfg, 'slam_toolbox.yaml'),
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([fast_lio, odom_bridge, slam])
