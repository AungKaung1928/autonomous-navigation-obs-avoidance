from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Set TurtleBot3 model (burger or waffle)
    os.environ['TURTLEBOT3_MODEL'] = 'burger'

    # 1. Start Gazebo with empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch', 'empty_world.launch.py'
            ])
        ]),
    )

    # 2. Start SLAM Toolbox for mapping
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch', 'online_async_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': PathJoinSubstitution([
                FindPackageShare('simple_navigation_project'),
                'config', 'nav2_params.yaml'
            ])
        }.items()
    )

    # 3. Start Nav2 Navigation Stack
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch', 'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': PathJoinSubstitution([
                FindPackageShare('simple_navigation_project'),
                'config', 'nav2_params.yaml'
            ])
        }.items()
    )

    # 4. RViz (Optional: uncomment if you want to visualize)
    rviz_config = PathJoinSubstitution([
        FindPackageShare('nav2_bringup'),
        'rviz', 'nav2_default_view.rviz'
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=None,  # Set to LaunchCondition if needed
        parameters=[{'use_sim_time': 'true'}]
    )

    # 5. Your obstacle avoider node
    navigator_node = Node(
        package='simple_navigation_project',
        executable='obstacle_avoider',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Optional: Spawn robot if not already in Gazebo
    spawn_robot = ExecuteProcess(
        cmd=[
            [
                FindExecutable(name='ros2'),
                ' service call /spawn_entity ',
                'gazebo_ros_interfaces/srv/SpawnEntity ',
                '"{name: \'turtlebot3\', xml: $(cat $(find turtlebot3_gazebo)/urdf/turtlebot3_burger.urdf), robot_namespace: \'\'}"'
            ]
        ],
        shell=True
    )

    # Delay Nav2 start after Gazebo
    delay_nav2 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo,
            on_exit=[nav2],
        )
    )

    # Delay SLAM after Gazebo
    delay_slam = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gazebo,
            on_exit=[slam],
        )
    )

    # Delay navigator after Nav2 starts
    delay_navigator = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=nav2,
            on_exit=[navigator_node],
        )
    )

    return LaunchDescription([
        gazebo,
        delay_slam,
        delay_nav2,
        delay_navigator,
        rviz, 
    ])
