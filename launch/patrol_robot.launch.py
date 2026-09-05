#!/usr/bin/env python3
"""Patrol controller only: use when a simulator (or the real robot) already publishes /scan."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('simple_navigation_project')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'config', 'patrol_params.yaml'),
            description='YAML with patrol_speed, turn_speed, safe_distance, min_lane_width'),
        Node(
            package='simple_navigation_project',
            executable='patrol_controller',
            name='patrol_controller',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('params_file'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
