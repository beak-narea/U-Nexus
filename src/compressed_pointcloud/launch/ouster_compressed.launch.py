"""Compress /ouster/points -> /ouster/points/compressed (foxglove CompressedPointCloud).

  ros2 launch compressed_pointcloud ouster_compressed.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory("compressed_pointcloud"),
        "config",
        "ouster.yaml",
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Parameter YAML for the compress node",
            ),
            DeclareLaunchArgument(
                "input_topic",
                default_value="/ouster/points",
                description="Raw PointCloud2 topic",
            ),
            DeclareLaunchArgument(
                "output_topic",
                default_value="/ouster/points/compressed",
                description="foxglove_msgs/CompressedPointCloud topic",
            ),
            Node(
                package="compressed_pointcloud",
                executable="compress_node",
                name="compress_pointcloud",
                output="screen",
                parameters=[
                    LaunchConfiguration("config"),
                    {
                        "input_topic": LaunchConfiguration("input_topic"),
                        "output_topic": LaunchConfiguration("output_topic"),
                    },
                ],
            ),
        ]
    )
