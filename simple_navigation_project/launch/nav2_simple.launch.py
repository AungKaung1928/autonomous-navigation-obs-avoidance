#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Parameters
    speed = LaunchConfiguration('normal_speed', default='0.2')
    turn = LaunchConfiguration('turn_speed', default='0.5')
    dist = LaunchConfiguration('min_distance', default='0.5')
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('normal_speed', default_value='0.2', description='Forward speed (m/s)'),
        DeclareLaunchArgument('turn_speed', default_value='0.5', description='Turn speed (rad/s)'),
        DeclareLaunchArgument('min_distance', default_value='0.5', description='Min obstacle distance (m)'),
        
        # Static TF for Nav2
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
        
        # Main node with parameters
        Node(
            package='simple_navigation_project',
            executable='obstacle_avoider',
            output='screen',
            parameters=[{
                'normal_speed': speed,
                'turn_speed': turn,
                'min_distance': dist
            }]
        )
    ])