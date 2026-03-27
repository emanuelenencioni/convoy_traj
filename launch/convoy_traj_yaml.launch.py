#!/usr/bin/env python3
"""Launch convoy_traj_node loading parameters from YAML config files."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('convoy_traj')

    params_file = os.path.join(pkg_share, 'config', 'convoy_traj_params.yaml')
    calib_file = os.path.join(pkg_share, 'config', 'tag_calibration.yaml')

    convoy_traj_node = Node(
        package='convoy_traj',
        executable='convoy_traj_node',
        name='convoy_traj_node',
        output='screen',
        # Load base params first, then calibration so calib values take precedence.
        parameters=[params_file, calib_file],
    )

    return LaunchDescription([
        convoy_traj_node,
    ])
