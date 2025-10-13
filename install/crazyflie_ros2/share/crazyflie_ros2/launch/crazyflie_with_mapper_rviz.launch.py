#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    log_level_arg = DeclareLaunchArgument('log_level', default_value='info')

    def setup(context, *args, **kwargs):
        log_level = LaunchConfiguration('log_level').perform(context)

        crazyflie_node = Node(
            package='crazyflie_ros2',
            executable='crazyflie_node',
            name='crazyflie_node',
            output='screen',
            arguments=['--ros-args', '--log-level', f'crazyflie_node:={log_level}'],
            parameters=[{
                'uri': 'radio://0/80/2M/E7E7E7E7E2',
                'hover_height': 0.5,
                'speed_factor': 0.3,
                'logging_only': False,
                'map_frame': 'map',
                'world_frame': 'world',
                'odom_frame': 'crazyflie/odom',
                'base_frame': 'crazyflie',
            }]
        )

        simple_mapper_node = Node(
            package='crazyflie_ros2',
            executable='simple_mapper_node',
            name='simple_mapper_node',
            output='screen',
            arguments=['--ros-args', '--log-level', f'simple_mapper_node:={log_level}'],
            parameters=[{'robot_prefix': '/crazyflie'}]
        )

        autonomous_navigation_node = Node(
            package='crazyflie_ros2',
            executable='autonomous_navigation_node',
            name='autonomous_navigation_node',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'waypoint_tolerance': 0.2,
                'planning_frequency': 2.0,
                'scanning_yaw_rate': 0.8,
                'flight_height': 0.3,
                'active_scanning.enabled': True,
                'active_scanning.speed_scale': 0.7,
                'active_scanning.lookahead_tau': 0.3,
                'active_scanning.vz_kp': 1.0,
            }]
        )

        return [crazyflie_node, simple_mapper_node, autonomous_navigation_node]

    return LaunchDescription([log_level_arg, OpaqueFunction(function=setup)])
