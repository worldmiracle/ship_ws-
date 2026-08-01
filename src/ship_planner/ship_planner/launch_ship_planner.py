"""
Ship path planning launch file.
Starts map server, path planner node, and RViz2.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Map file path
    map_yaml = '/home/douhouqi/ship_ws/maps/water_map.yaml'

    # RViz2 config path
    pkg_share = os.path.expanduser('~/ship_ws/src/ship_planner')
    rviz_config = os.path.join(pkg_share, 'config', 'ship_planner.rviz')

    # Launch arguments
    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=map_yaml,
        description='Map file path'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Custom map server node (replaces nav2_map_server)
    map_server_node = Node(
        package='ship_planner',
        executable='simple_map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # Path planner node
    path_planner_node = Node(
        package='ship_planner',
        executable='path_planner',
        name='path_planner_node',
        output='screen',
        parameters=[{
            'map_topic': '/map',
            'goal_topic': '/goal_pose',
            'start_topic': '/initialpose',
            'path_topic': '/planned_path',
            'csv_output_path': '/home/douhouqi/ship_ws/planned_path.csv',
            'smooth_path': True,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # RViz2 visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        map_yaml_arg,
        use_sim_time_arg,
        map_server_node,
        path_planner_node,
        rviz_node,
    ])
