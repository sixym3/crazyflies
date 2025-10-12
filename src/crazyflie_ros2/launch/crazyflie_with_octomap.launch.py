#!/usr/bin/env python3
"""Launch file for Crazyflie with OctoMap server and autonomous navigation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Crazyflie with OctoMap integration."""

    # Declare launch arguments
    uri_arg = DeclareLaunchArgument(
        'uri',
        default_value='radio://0/80/2M/E7E7E7E7E2',
        description='Crazyflie URI'
    )

    hover_height_arg = DeclareLaunchArgument(
        'hover_height',
        default_value='0.5',
        description='Default hover height in meters'
    )

    speed_factor_arg = DeclareLaunchArgument(
        'speed_factor',
        default_value='0.3',
        description='Speed factor for velocity commands'
    )

    resolution_arg = DeclareLaunchArgument(
        'octomap_resolution',
        default_value='0.10',
        description='OctoMap voxel resolution in meters'
    )

    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value='2.0',
        description='Maximum sensor range to trust in meters'
    )

    logging_only_arg = DeclareLaunchArgument(
        'logging_only',
        default_value='false',
        description='Enable logging-only mode (no flight commands sent to drone)'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )

    # Crazyflie node
    crazyflie_node = Node(
        package='crazyflie_ros2',
        executable='crazyflie_node',
        name='crazyflie_node',
        output='screen',
        parameters=[{
            'uri': LaunchConfiguration('uri'),
            'hover_height': LaunchConfiguration('hover_height'),
            'speed_factor': LaunchConfiguration('speed_factor'),
            'logging_only': LaunchConfiguration('logging_only'),
        }]
    )

    # OctoMap mapper node (publishes point cloud from multiranger sensors)
    octomap_mapper_node = Node(
        package='crazyflie_ros2',
        executable='octomap_mapper_node',
        name='octomap_mapper_node',
        output='screen',
        parameters=[{
            'max_range': LaunchConfiguration('max_range'),
            'min_range': 0.02,
            'robot_frame': 'crazyflie',
        }]
    )

    # OctoMap server node (converts point cloud to OctoMap and publishes projected_map)
    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[{
            'resolution': LaunchConfiguration('octomap_resolution'),
            'frame_id': 'world',
            'base_frame_id': 'crazyflie',
            'sensor_model.max_range': LaunchConfiguration('max_range'),
            'sensor_model.hit': 0.7,
            'sensor_model.miss': 0.4,
            'sensor_model.min': 0.12,
            'sensor_model.max': 0.97,
            'occupancy_min_z': 0.0,
            'occupancy_max_z': 1.5,
            'point_cloud_min_z': 0.0,
            'point_cloud_max_z': 1.5,
            'publish_2d_map': True,
            'publish_free_space': False,
            'compress_map': True,
            'incremental_2D_projection': True,
            'latch': True,
        }],
        remappings=[
            ('cloud_in', '/octomap/point_cloud'),
        ]
    )

    # Autonomous navigation node
    autonomous_nav_node = Node(
        package='crazyflie_ros2',
        executable='autonomous_navigation_node',
        name='autonomous_navigation_node',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[{
            'waypoint_tolerance': 0.2,
            'planning_frequency': 2.0,
            'scanning_yaw_rate': 0.0,
            'flight_height': LaunchConfiguration('hover_height'),
        }]
    )

    return LaunchDescription([
        uri_arg,
        hover_height_arg,
        speed_factor_arg,
        resolution_arg,
        max_range_arg,
        logging_only_arg,
        log_level_arg,
        crazyflie_node,
        octomap_mapper_node,
        octomap_server_node,
        autonomous_nav_node,
    ])
