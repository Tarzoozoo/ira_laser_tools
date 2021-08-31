import launch
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

import os

scan_merger_node_pkg_prefix = get_package_share_directory('scan_merger')
scan_merger_node_param_file = os.path.join(scan_merger_node_pkg_prefix,
                                                  'config/params.yaml')


def generate_launch_description():
    scan_merger_node = Node(
        package='scan_merger',
        executable='scan_merger_node_exe',
        parameters=[scan_merger_node_param_file],
        remappings=[
            ("output", "/transformed_points_raw"),
            #("input1", "/lidar_front/points_raw"),
            ("input1", "/front_scan/points_raw"),
            #("input2", "/lidar_back/points_raw"),
            ("input2", "/back_scan/points_raw"),
            ("scan", "/merged_scan")
        ]
    )

    return launch.LaunchDescription([scan_merger_node])
