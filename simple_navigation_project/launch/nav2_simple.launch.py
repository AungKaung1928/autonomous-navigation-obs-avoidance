import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Nav2 bringup launch file
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
        }.items()
    )
    
    # Obstacle avoider node
    obstacle_avoider = Node(
        package='simple_navigation_project',
        executable='obstacle_avoider',
        name='obstacle_avoider',
        output='screen'
    )
    
    return LaunchDescription([
        nav2_launch,
        obstacle_avoider
    ])
