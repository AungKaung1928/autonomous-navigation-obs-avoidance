#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Parameters with default values
    speed = LaunchConfiguration('normal_speed')
    turn = LaunchConfiguration('turn_speed') 
    dist = LaunchConfiguration('min_distance')
    
    return LaunchDescription([
        # Declare arguments with default values
        DeclareLaunchArgument(
            'normal_speed', 
            default_value='0.2', 
            description='Forward speed (m/s) - Range: 0.1 to 0.5'
        ),
        DeclareLaunchArgument(
            'turn_speed', 
            default_value='0.5', 
            description='Turn speed (rad/s) - Range: 0.3 to 1.5'
        ),
        DeclareLaunchArgument(
            'min_distance', 
            default_value='0.5', 
            description='Min obstacle distance (m) - Range: 0.3 to 1.0'
        ),
        
        # Main obstacle avoider node
        Node(
            package='simple_navigation_project',
            executable='obstacle_avoider',
            name='obstacle_avoider',
            output='screen',
            parameters=[{
                'normal_speed': speed,
                'turn_speed': turn,
                'min_distance': dist
            }],
            remappings=[
                ('/cmd_vel', '/cmd_vel')
            ]
        )
    ])