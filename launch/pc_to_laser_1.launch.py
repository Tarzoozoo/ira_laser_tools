from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import yaml


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan',
            executable='laserscan_to_pointcloud_node',
            name='laserscan_to_pointcloud_front',
            remappings=[('scan_in', '/scan'),
                        ('cloud', '/cloud_front')],
            parameters=[{'target_frame': 'base_scan', 'transform_tolerance': 0.01}]
        ),
    ])