#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    pkg_dir = get_package_share_directory('simple_navigation_project')
    
    # Nav2 params file
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map', default=os.path.join(nav2_bringup_dir, 'maps', 'turtlebot3_world.yaml'))
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='Full path to map yaml file'
        ),
        
        # Launch TurtleBot3 in Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([turtlebot3_gazebo_dir, '/launch/turtlebot3_world.launch.py'])
        ),
        
        # Launch Navigation 2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir, '/launch/bringup_launch.py']),
            launch_arguments={
                'map': map_file,
                'use_sim_time': use_sim_time,
                'params_file': nav2_params
            }.items()
        ),
        
        # Launch waypoint navigator node
        Node(
            package='simple_navigation_project',
            executable='waypoint_navigator',
            name='waypoint_navigator',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
