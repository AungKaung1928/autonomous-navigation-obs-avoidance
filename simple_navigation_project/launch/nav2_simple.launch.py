import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Launch TurtleBot3 in Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(turtlebot3_gazebo_dir, 'launch'),
            '/turtlebot3_world.launch.py'
        ])
    )
    
    # Launch Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_bringup_dir, 'launch'),
            '/tb3_simulation_launch.py'
        ]),
        launch_arguments={
            'headless': 'False'
        }.items()
    )
    
    # Launch our obstacle avoidance node
    obstacle_avoidance_node = Node(
        package='simple_navigation_project',
        executable='obstacle_avoider',
        name='simple_obstacle_avoider',
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_launch,
        nav2_launch,
        obstacle_avoidance_node
    ])
