#!/usr/bin/env python3
"""Launch sim + Crazyflie bringup + simple mapper stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # ----- Arguments -----
    log_level_arg = DeclareLaunchArgument(
        'log_level', default_value='info', description='Logging level'
    )
    use_real_cf_arg = DeclareLaunchArgument(
        'use_real_crazyflie', default_value='false',
        description='If true, try to connect to a real Crazyflie (not recommended for sim)'
    )
    frame_id_arg = DeclareLaunchArgument(
        'frame_id', default_value='crazyflie', description='Body frame id'
    )
    world_frame_arg = DeclareLaunchArgument(
        'world_frame', default_value='world', description='World frame id'
    )
    takeoff_height_arg = DeclareLaunchArgument(
        'takeoff_height', default_value='0.30', description='Sim takeoff height (m)'
    )
    rate_arg = DeclareLaunchArgument(
        'publish_rate_hz', default_value='10.0', description='Sim publish/update rate'
    )

    def launch_setup(context, *args, **kwargs):
        log_level = LaunchConfiguration('log_level').perform(context)
        use_real = LaunchConfiguration('use_real_crazyflie').perform(context)

        # --- Simulation node ---
        sim_node = Node(
            package='crazyflie_ros2',
            executable='crazyflie_sim_node',   # install as entry point or python script
            name='crazyflie_sim_node',
            output='screen',
            arguments=['--ros-args', '--log-level', f'crazyflie_sim_node:={log_level}'],
            parameters=[{
                'frame_id': LaunchConfiguration('frame_id'),
                'world_frame': LaunchConfiguration('world_frame'),
                'takeoff_height': LaunchConfiguration('takeoff_height'),
                'publish_rate_hz': LaunchConfiguration('publish_rate_hz'),
                # box dims default to 5x5x2; override if needed:
                'box_half_xy': 2.5,
                'box_height': 2.0,
            }]
        )

        # --- Your existing Crazyflie node (set logging_only=true so it won’t command motors) ---
        # Leaves topic interfaces intact for downstream nodes.
        crazyflie_node = Node(
            package='crazyflie_ros2',
            executable='crazyflie_node',
            name='crazyflie_node',
            output='screen',
            arguments=['--ros-args', '--log-level', f'crazyflie_node:={log_level}'],
            parameters=[{
                # even if use_real_crazyflie=false, we pass logging_only=true to be safe
                'uri': 'radio://0/80/2M/E7E7E7E7E2',
                'hover_height': 0.5,
                'speed_factor': 0.3,
                'logging_only': (use_real.lower() != 'true'),
            }]
        )

        # --- Simple mapper node (as in your bringup) ---
        simple_mapper_node = Node(
            package='crazyflie_ros2',
            executable='simple_mapper_node',
            name='simple_mapper_node',
            output='screen',
            arguments=['--ros-args', '--log-level', f'simple_mapper_node:={log_level}'],
            parameters=[{
                'robot_prefix': '/crazyflie',
            }]
        )

        # --- Autonomous navigation node (as in your bringup) ---
        autonomous_navigation_node = Node(
            package='crazyflie_ros2',
            executable='autonomous_navigation_node',
            name='autonomous_navigation_node',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'waypoint_tolerance': 0.2,
                'planning_frequency': 2.0,
                'scanning_yaw_rate': 0.5,
                'flight_height': 0.3,
            }]
        )

        # In pure-sim by default we still launch crazyflie_node in logging_only mode
        # to keep any downstream expectations about that node satisfied, but it will not command a drone.
        return [
            sim_node,
            crazyflie_node,
            simple_mapper_node,
            autonomous_navigation_node,
        ]

    return LaunchDescription([
        log_level_arg,
        use_real_cf_arg,
        frame_id_arg,
        world_frame_arg,
        takeoff_height_arg,
        rate_arg,
        OpaqueFunction(function=launch_setup),
    ])
