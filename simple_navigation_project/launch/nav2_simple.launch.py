from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Set TurtleBot3 model
    os.environ['TURTLEBOT3_MODEL'] = 'burger'
    
    # 1. Gazebo + TurtleBot3
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch', 'empty_world.launch.py'
            ])
        ])
    )
    
    # 2. SLAM
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch', 'online_async_launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    # 3. Nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch', 'navigation_launch.py'
            ])
        ]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    # 4. Your node
    your_node = Node(
        package='simple_navigation_project',
        executable='obstacle_avoider',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([gazebo, slam, nav2, your_node])
