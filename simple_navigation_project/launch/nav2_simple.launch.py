#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package name
    package_name = 'simple_navigation_project'
    
    # Get package directory
    pkg_dir = get_package_share_directory(package_name)
    
    # Paths
    nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Static transform publisher for map->odom (simplified Nav2 setup)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # Your obstacle avoider with Nav2 concepts
    obstacle_avoider = Node(
        package='simple_navigation_project',
        executable='obstacle_avoider',
        name='obstacle_avoider',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        # Static transform for Nav2 compatibility
        static_tf,
        
        # Launch your obstacle avoider
        obstacle_avoider
    ])