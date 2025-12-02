#!/usr/bin/env python3
"""Production launch file for autonomous patrol robot"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with all patrol components"""
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    patrol_speed_arg = DeclareLaunchArgument(
        'patrol_speed',
        default_value='0.15',
        description='Patrol forward speed (m/s)'
    )
    
    turn_speed_arg = DeclareLaunchArgument(
        'turn_speed',
        default_value='1.0',
        description='Turn speed (rad/s)'
    )
    
    safe_distance_arg = DeclareLaunchArgument(
        'safe_distance',
        default_value='1.0',
        description='Safe distance from obstacles (m)'
    )
    
    # Main patrol controller
    patrol_controller = Node(
        package='simple_navigation_project',
        executable='patrol_controller',
        name='patrol_controller',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'patrol_speed': LaunchConfiguration('patrol_speed'),
            'turn_speed': LaunchConfiguration('turn_speed'),
            'safe_distance': LaunchConfiguration('safe_distance'),
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/scan', '/scan'),
        ]
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        patrol_speed_arg,
        turn_speed_arg,
        safe_distance_arg,
        
        # Nodes
        patrol_controller,
    ])