"""
Ship path planning launch file.
Starts map server, path planner node, and RViz2.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Default map file path (relative to user home)
    default_map_yaml = os.path.expanduser('~/ship_ws/maps/water_map.yaml')

    # Use FindPackageShare to locate RViz2 config dynamically
    pkg_share = FindPackageShare('ship_planner')
    rviz_config = PathJoinSubstitution([pkg_share, 'config', 'ship_planner.rviz'])

    # Default CSV output path
    default_csv_path = os.path.expanduser('~/ship_ws/planned_path.csv')

    # Launch arguments
    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_yaml,
        description='Path to map YAML file'
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
            'csv_output_path': default_csv_path,
            'smooth_path': True,
            'enable_dynamic_replanning': True,
            'dynamic_obstacles_topic': '/dynamic_obstacles',
            'ship_pose_topic': '/ship_pose',
            'replan_check_interval': 2.0,
            'replan_safety_distance': 8.0,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # Dynamic obstacle detector (simulates other vessels / floating objects)
    obstacle_detector_node = Node(
        package='ship_planner',
        executable='obstacle_detector',
        name='obstacle_detector',
        output='screen',
        parameters=[{
            'spawn_interval': 8.0,
            'obstacle_radius': 12.0,
            'obstacle_lifetime': 30.0,
            'max_obstacles': 5,
            'path_topic': '/planned_path',
            'ship_pose_topic': '/ship_pose',
            'obstacles_topic': '/dynamic_obstacles',
            'enabled': True,
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

    # Ship motion simulator (visualizes movement along planned path)
    ship_simulator_node = Node(
        package='ship_planner',
        executable='ship_simulator',
        name='ship_simulator',
        output='screen',
        parameters=[{
            'speed': 5.0,
            'update_rate': 20.0,
            'path_topic': '/planned_path',
            'pose_topic': '/ship_pose',
            'loop_mode': False,
            'frame_id': 'map',
            'child_frame_id': 'ship_base',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        map_yaml_arg,
        use_sim_time_arg,
        map_server_node,
        path_planner_node,
        obstacle_detector_node,
        ship_simulator_node,
        rviz_node,
    ])
