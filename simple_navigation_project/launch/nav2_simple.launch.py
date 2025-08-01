from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('simple_navigation_project')

    # Start Nav2
    nav2_bringup_cmd = ExecuteProcess(
        cmd=['ros2', 'launch', 'nav2_bringup', 'bringup_launch.py',
             f'map:=',  # no map, use empty world
             'use_sim_time:=true',
             'params_file:=' + os.path.join(pkg_dir, 'config', 'nav2_params.yaml')],
        output='screen'
    )

    # Start your node
    avoider_node = Node(
        package='simple_navigation_project',
        executable='obstacle_avoider',
        name='obstacle_avoider',
        output='screen'
    )

    return LaunchDescription([
        nav2_bringup_cmd,
        avoider_node
    ])
