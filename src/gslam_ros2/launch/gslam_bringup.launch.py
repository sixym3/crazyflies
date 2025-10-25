#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    # Declare launch arguments
    height_arg = DeclareLaunchArgument(
        'height',
        default_value='0.70',
        description='Target flight height in meters'
    )

    address_arg = DeclareLaunchArgument(
        'address',
        default_value='radio://0/80/2M/E7E7E7E7E2',
        description='Crazyflie radio address'
    )

    linear_speed_factor_arg = DeclareLaunchArgument(
        'linear_speed_factor',
        default_value='0.5',
        description='Linear speed scaling factor'
    )

    angular_speed_factor_arg = DeclareLaunchArgument(
        'angular_speed_factor',
        default_value='45.0',
        description='Angular speed scaling factor (degrees/sec)'
    )

    control_rate_hz_arg = DeclareLaunchArgument(
        'control_rate_hz',
        default_value='30.0',
        description='Control loop frequency in Hz'
    )

    position_tolerance_arg = DeclareLaunchArgument(
        'position_tolerance',
        default_value='0.25',
        description='Position tolerance for switching to position hold mode (meters)'
    )

    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame name'
    )

    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base frame name'
    )

    max_waypoint_distance_arg = DeclareLaunchArgument(
        'max_waypoint_distance',
        default_value='0.4',
        description='Maximum distance to next waypoint (meters)'
    )

    nav_enabled_arg = DeclareLaunchArgument(
        'nav_enabled',
        default_value='false',
        description='Enable autonomous navigation'
    )

    enable_monitor_arg = DeclareLaunchArgument(
        'enable_monitor',
        default_value='false',
        description='Enable range monitor visualization'
    )

    # Mission Manager parameters
    search_span_m_arg = DeclareLaunchArgument(
        'search_span_m',
        default_value='0.6',
        description='Search span in meters'
    )

    search_step_m_arg = DeclareLaunchArgument(
        'search_step_m',
        default_value='0.15',
        description='Search step size in meters'
    )

    min_edge_points_arg = DeclareLaunchArgument(
        'min_edge_points',
        default_value='20',
        description='Minimum number of edge points required'
    )

    goal_tolerance_arg = DeclareLaunchArgument(
        'goal_tolerance',
        default_value='0.2',
        description='Goal position tolerance in meters'
    )

    edge_detection_delay_arg = DeclareLaunchArgument(
        'edge_detection_delay',
        default_value='15.0',
        description='Edge detection delay in seconds'
    )

    box_center_policy_arg = DeclareLaunchArgument(
        'box_center_policy',
        default_value='box_fitting',
        description='Policy for computing box center (box_fitting or centroid)'
    )

    box_size_arg = DeclareLaunchArgument(
        'box_size',
        default_value='0.04',
        description='Known box size in meters'
    )

    # Edge Detector parameters
    range_topic_arg = DeclareLaunchArgument(
        'range_topic',
        default_value='/range/down',
        description='Range sensor topic name'
    )

    box_height_arg = DeclareLaunchArgument(
        'box_height',
        default_value='0.1',
        description='Box height in meters'
    )

    modifier_arg = DeclareLaunchArgument(
        'modifier',
        default_value='0.9',
        description='Edge detection modifier'
    )

    debounce_period_arg = DeclareLaunchArgument(
        'debounce_period',
        default_value='0.3',
        description='Debounce period in seconds'
    )

    log_csv_arg = DeclareLaunchArgument(
        'log_csv',
        default_value='false',
        description='Enable CSV logging'
    )

    log_dir_arg = DeclareLaunchArgument(
        'log_dir',
        default_value='.',
        description='Log directory path'
    )

    # Range Monitor parameters
    time_window_sec_arg = DeclareLaunchArgument(
        'time_window_sec',
        default_value='5.0',
        description='Time window in seconds for range monitoring'
    )

    z_window_sec_arg = DeclareLaunchArgument(
        'z_window_sec',
        default_value='1.0',
        description='Z window in seconds'
    )

    z_threshold_arg = DeclareLaunchArgument(
        'z_threshold',
        default_value='3.0',
        description='Z threshold value'
    )

    plot_update_hz_arg = DeclareLaunchArgument(
        'plot_update_hz',
        default_value='20.0',
        description='Plot update frequency in Hz'
    )

    range_sensor_offset_arg = DeclareLaunchArgument(
        'range_sensor_offset',
        default_value='-0.2',
        description='Range sensor mounting offset in meters'
    )

    # Planner parameters
    unknown_cell_cost_arg = DeclareLaunchArgument(
        'unknown_cell_cost',
        default_value='50.0',
        description='Cost for unknown cells in path planning'
    )

    min_waypoint_distance_arg = DeclareLaunchArgument(
        'min_waypoint_distance',
        default_value='0.3',
        description='Minimum distance between waypoints in meters'
    )

    use_line_of_sight_optimization_arg = DeclareLaunchArgument(
        'use_line_of_sight_optimization',
        default_value='true',
        description='Enable line-of-sight path optimization'
    )

    # Controller node (now handles odometry and range publishing internally)
    controller_node = Node(
        package='gslam_ros2',
        executable='controller',
        name='crazyflie_controller',
        output='screen',
        parameters=[{
            'height': LaunchConfiguration('height'),
            'address': LaunchConfiguration('address'),
            'linear_speed_factor': LaunchConfiguration('linear_speed_factor'),
            'angular_speed_factor': LaunchConfiguration('angular_speed_factor'),
            'control_rate_hz': LaunchConfiguration('control_rate_hz'),
            'position_tolerance': LaunchConfiguration('position_tolerance'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'range_sensor_offset': LaunchConfiguration('range_sensor_offset'),
        }]
    )

    # 2D Mapper node
    mapper_2d_node = Node(
        package='gslam_ros2',
        executable='mapper_2d',
        name='mapper_2d',
        output='screen',
        parameters=[{
            'map_size_x': 20.0,
            'map_size_y': 20.0,
            'map_origin_x': -10.0,
            'map_origin_y': -10.0,
            'map_resolution': 0.05,
        }]
    )

    # D* Lite Planner node
    planner_node = Node(
        package='gslam_ros2',
        executable='planner_node',
        name='planner_node',
        output='screen',
        parameters=[{
            'flight_height': LaunchConfiguration('height'),
            'min_waypoint_distance': LaunchConfiguration('min_waypoint_distance'),
            'use_line_of_sight_optimization': LaunchConfiguration('use_line_of_sight_optimization'),
            'unknown_cell_cost': LaunchConfiguration('unknown_cell_cost'),
        }]
    )

    # Navigation node
    nav_node = Node(
        package='gslam_ros2',
        executable='nav_node',
        name='nav_node',
        output='screen',
        parameters=[{
            'max_waypoint_distance': LaunchConfiguration('max_waypoint_distance'),
            'enabled': LaunchConfiguration('nav_enabled'),
            'flight_height': LaunchConfiguration('height'),
        }]
    )

    # Scan node
    scan_node = Node(
        package='gslam_ros2',
        executable='scan_node',
        name='scan_node',
        output='screen',
        parameters=[{
            'angular_speed': LaunchConfiguration('angular_speed_factor'),
            'rotation_angle': 90.0,
            'publish_rate': 50.0,
        }]
    )

    # Mission Manager node
    mission_manager_node = Node(
        package='gslam_ros2',
        executable='mission_manager_node',
        name='mission_manager_node',
        output='screen',
        parameters=[{
            'flight_height': LaunchConfiguration('height'),
            'search_span_m': LaunchConfiguration('search_span_m'),
            'search_step_m': LaunchConfiguration('search_step_m'),
            'min_edge_points': LaunchConfiguration('min_edge_points'),
            'goal_tolerance': LaunchConfiguration('goal_tolerance'),
            'edge_detection_delay': LaunchConfiguration('edge_detection_delay'),
            'box_center_policy': LaunchConfiguration('box_center_policy'),
            'box_size': LaunchConfiguration('box_size'),
        }]
    )

    # Edge Detector node
    edge_detector_node = Node(
        package='gslam_ros2',
        executable='edge_detector_node',
        name='edge_detector_node',
        output='screen',
        parameters=[{
            'topic': LaunchConfiguration('range_topic'),
            'flight_height': LaunchConfiguration('height'),
            'box_height': LaunchConfiguration('box_height'),
            'modifier': LaunchConfiguration('modifier'),
            'edge_detection_delay': LaunchConfiguration('edge_detection_delay'),
            'debounce_period': LaunchConfiguration('debounce_period'),
            'log_csv': LaunchConfiguration('log_csv'),
            'log_dir': LaunchConfiguration('log_dir'),
        }]
    )

    # Range Monitor node (optional)
    range_monitor_node = Node(
        package='gslam_ros2',
        executable='range_monitor_node',
        name='range_monitor_node',
        output='screen',
        parameters=[{
            'topic': LaunchConfiguration('range_topic'),
            'time_window_sec': LaunchConfiguration('time_window_sec'),
            'z_window_sec': LaunchConfiguration('z_window_sec'),
            'z_threshold': LaunchConfiguration('z_threshold'),
            'expected_height_m': LaunchConfiguration('height'),
            'plot_update_hz': LaunchConfiguration('plot_update_hz'),
            'log_dir': LaunchConfiguration('log_dir'),
            'enable_monitor': LaunchConfiguration('enable_monitor'),
        }],
        condition=IfCondition(LaunchConfiguration('enable_monitor'))
    )

    return LaunchDescription([
        height_arg,
        address_arg,
        linear_speed_factor_arg,
        angular_speed_factor_arg,
        control_rate_hz_arg,
        position_tolerance_arg,
        odom_frame_arg,
        base_frame_arg,
        max_waypoint_distance_arg,
        nav_enabled_arg,
        enable_monitor_arg,
        search_span_m_arg,
        search_step_m_arg,
        min_edge_points_arg,
        goal_tolerance_arg,
        edge_detection_delay_arg,
        box_center_policy_arg,
        box_size_arg,
        range_topic_arg,
        box_height_arg,
        modifier_arg,
        debounce_period_arg,
        log_csv_arg,
        log_dir_arg,
        time_window_sec_arg,
        z_window_sec_arg,
        z_threshold_arg,
        plot_update_hz_arg,
        range_sensor_offset_arg,
        unknown_cell_cost_arg,
        min_waypoint_distance_arg,
        use_line_of_sight_optimization_arg,
        controller_node,
        mapper_2d_node,
        planner_node,
        nav_node,
        scan_node,
        mission_manager_node,
        edge_detector_node,
        range_monitor_node,
    ])
