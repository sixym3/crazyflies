#!/usr/bin/env python3
"""
Autonomous Navigation Node

Orchestrates scan-plan-move loop for autonomous navigation:
1. Rotate 90 degrees to scan surroundings
2. Request path planning
3. Select next waypoint from path
4. Move to waypoint
5. Repeat until goal reached

Subscribes to:
  /odom (nav_msgs/Odometry) - current position
  /position_target_reached (std_msgs/Bool) - waypoint arrival confirmation
  /planned_path (nav_msgs/Path) - path from planner
  /goal_pose (geometry_msgs/PoseStamped) - navigation goal

Publishes:
  /cmd_pos (geometry_msgs/PointStamped) - position commands

Service Clients:
  /trigger_replanning (std_srvs/Trigger) - request path planning
  /scan_surround (std_srvs/Trigger) - rotate 90 degrees
"""

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Bool
from std_srvs.srv import Trigger, SetBool
from visualization_msgs.msg import Marker

import math
import threading
import time
from typing import Optional, Tuple


class NavNode(Node):
    def __init__(self):
        super().__init__('nav_node')

        # Parameters
        self.declare_parameter('max_waypoint_distance', 1.0)
        self.declare_parameter('enabled', False)
        self.declare_parameter('flight_height', 0.5)

        self.max_waypoint_distance = self.get_parameter('max_waypoint_distance').value
        self.enabled = self.get_parameter('enabled').value
        self.flight_height = self.get_parameter('flight_height').value

        # State
        self.current_odom = None
        self.goal_pose = None
        self.current_path = None
        self.current_waypoint_index = 0
        self.position_reached = False
        self.goal_reached = False

        # Locks
        self.path_lock = threading.Lock()
        self.position_reached_lock = threading.Lock()

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.position_reached_sub = self.create_subscription(
            Bool, '/position_target_reached', self.position_reached_callback, 10)
        self.path_sub = self.create_subscription(
            Path, '/planned_path', self.path_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Publishers
        self.cmd_pos_pub = self.create_publisher(PointStamped, '/cmd_pos', 10)
        self.waypoint_marker_pub = self.create_publisher(Marker, '/nav_waypoint_marker', 10)
        self.goal_reached_pub = self.create_publisher(Bool, '/nav_node/goal_reached', 10)

        # Service clients
        self.replan_client = self.create_client(Trigger, '/trigger_replanning')
        self.scan_client = self.create_client(Trigger, '/scan_surround')

        # Service servers
        self.enable_service = self.create_service(
            SetBool, '/enable_nav', self.enable_callback)

        # Navigation thread
        self.nav_thread = None
        self.nav_running = False

        self.get_logger().info(f'Navigation node initialized')
        self.get_logger().info(f'Max waypoint distance: {self.max_waypoint_distance}m')
        self.get_logger().info(f'Enabled: {self.enabled}')

        # Start navigation loop if enabled
        if self.enabled:
            self.start_navigation()

    def odom_callback(self, msg):
        """Update current odometry."""
        self.current_odom = msg

    def position_reached_callback(self, msg):
        """Handle position target reached confirmation."""
        with self.position_reached_lock:
            if msg.data:
                self.position_reached = True
                self.get_logger().debug('Position target reached')

    def path_callback(self, msg):
        """Update current path and set waypoint index to 1 (skip first waypoint which is current position)."""
        with self.path_lock:
            self.current_path = msg
            # Skip first waypoint (index 0, typically current position) and go to second waypoint
            # Since replanning happens frequently, first waypoint is usually at/near current position
            self.current_waypoint_index = 1 if len(msg.poses) > 1 else 0
            self.get_logger().info(f'Received new path with {len(msg.poses)} waypoints, starting from index {self.current_waypoint_index}')

    def goal_callback(self, msg):
        """Handle new goal."""
        self.goal_pose = msg
        self.goal_reached = False
        self.get_logger().info(f'New goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

        # Start navigation if not already running
        if self.enabled and not self.nav_running:
            self.start_navigation()

    def enable_callback(self, request, response):
        """Handle enable/disable service requests."""
        if request.data:
            # Enable navigation
            self.enabled = True
            if self.goal_pose is not None and not self.nav_running:
                self.start_navigation()
                response.success = True
                response.message = 'Navigation enabled and started'
                self.get_logger().info('Navigation enabled via service')
            else:
                response.success = True
                response.message = 'Navigation enabled (waiting for goal or already running)'
                self.get_logger().info('Navigation enabled via service')
        else:
            # Disable navigation
            self.enabled = False
            if self.nav_running:
                self.stop_navigation()
                response.success = True
                response.message = 'Navigation disabled and stopped'
                self.get_logger().info('Navigation disabled via service')
            else:
                response.success = True
                response.message = 'Navigation disabled (was not running)'
                self.get_logger().info('Navigation disabled via service')

        return response

    def start_navigation(self):
        """Start the navigation loop in a separate thread."""
        if self.nav_running:
            self.get_logger().warning('Navigation already running')
            return

        self.nav_running = True
        self.nav_thread = threading.Thread(target=self.navigation_loop, daemon=True)
        self.nav_thread.start()
        self.get_logger().info('Navigation loop started')

    def stop_navigation(self):
        """Stop the navigation loop."""
        self.nav_running = False
        if self.nav_thread:
            self.nav_thread.join(timeout=5.0)
        self.get_logger().info('Navigation loop stopped')

    def navigation_loop(self):
        """Main navigation loop: scan -> plan -> select waypoint -> move -> repeat."""
        while rclpy.ok() and self.nav_running and not self.goal_reached:
            # Check if we have necessary data
            if self.current_odom is None:
                self.get_logger().warning('Waiting for odometry...')
                time.sleep(1.0)
                continue

            if self.goal_pose is None:
                self.get_logger().warning('Waiting for goal...')
                time.sleep(1.0)
                continue

            # 1. Scan surroundings (rotate 90 degrees)
            self.get_logger().info('Step 1: Scanning surroundings...')
            if not self.call_scan_surround():
                self.get_logger().error('Scan failed, retrying...')
                time.sleep(1.0)
                continue

            # 2. Request path planning
            self.get_logger().info('Step 2: Requesting path planning...')
            if not self.call_replan():
                self.get_logger().error('Planning failed, retrying...')
                time.sleep(1.0)
                continue

            # 3. Wait for updated path
            time.sleep(0.5)  # Give planner time to publish path

            # 4. Select waypoint from path
            waypoint = self.select_waypoint_from_path()
            if waypoint is None:
                self.get_logger().warning('No valid waypoint found, retrying...')
                time.sleep(1.0)
                continue

            self.get_logger().info(f'Step 3: Selected waypoint ({waypoint[0]:.2f}, {waypoint[1]:.2f})')

            # 5. Move to waypoint
            self.publish_waypoint(waypoint)

            # 6. Wait for arrival
            self.get_logger().info('Step 4: Moving to waypoint...')
            if not self.wait_for_position_reached(timeout=30.0):
                self.get_logger().warning('Waypoint timeout, continuing anyway...')

            # 7. Check if goal reached (waypoint is goal)
            if self.is_waypoint_goal(waypoint):
                self.goal_reached = True
                self.get_logger().info('Goal reached!')

                # Publish goal reached notification
                goal_reached_msg = Bool()
                goal_reached_msg.data = True
                self.goal_reached_pub.publish(goal_reached_msg)
                break

        self.nav_running = False
        self.get_logger().info('Navigation loop exited')

    def call_scan_surround(self) -> bool:
        """Call scan_surround service to rotate 90 degrees."""
        if not self.scan_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Scan service not available')
            return False

        request = Trigger.Request()
        future = self.scan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is not None and future.result().success:
            self.get_logger().debug('Scan service succeeded')
            return True
        else:
            self.get_logger().error('Scan service failed')
            return False

    def call_replan(self) -> bool:
        """Call trigger_replanning service."""
        if not self.replan_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Replan service not available')
            return False

        request = Trigger.Request()
        future = self.replan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is not None and future.result().success:
            self.get_logger().debug('Replan service succeeded')
            return True
        else:
            self.get_logger().error('Replan service failed')
            return False

    def select_waypoint_from_path(self) -> Optional[Tuple[float, float]]:
        """
        Select next waypoint from path sequentially.
        Starts from current_waypoint_index and returns the furthest waypoint
        within max_waypoint_distance, ensuring path is followed sequentially.
        """
        with self.path_lock:
            if self.current_path is None or len(self.current_path.poses) == 0:
                return None

            path = self.current_path

        if self.current_odom is None:
            return None

        current_x = self.current_odom.pose.pose.position.x
        current_y = self.current_odom.pose.pose.position.y

        # Start from current waypoint index, but ensure it's within bounds
        start_index = min(self.current_waypoint_index, len(path.poses) - 1)

        # Safety check: if start_index was clamped, log warning
        if start_index != self.current_waypoint_index:
            self.get_logger().warning(
                f'Waypoint index {self.current_waypoint_index} out of bounds (path length {len(path.poses)}), '
                f'clamped to {start_index}')
            self.current_waypoint_index = start_index

        # Iterate through path from current index and find furthest reachable waypoint
        selected_waypoint = None
        selected_index = -1

        for i in range(start_index, len(path.poses)):
            pose = path.poses[i]
            wx = pose.pose.position.x
            wy = pose.pose.position.y

            distance = math.hypot(wx - current_x, wy - current_y)

            # Check if within max distance
            if distance <= self.max_waypoint_distance:
                # Update to this waypoint (we want the furthest one in sequence)
                selected_waypoint = (wx, wy)
                selected_index = i
            else:
                # Once we exceed max distance, stop searching
                # (to maintain sequential path following)
                break

        # If no waypoint within max distance, use the waypoint at start_index anyway
        # (we must make progress toward the path)
        if selected_waypoint is None and start_index < len(path.poses):
            pose = path.poses[start_index]
            selected_waypoint = (pose.pose.position.x, pose.pose.position.y)
            selected_index = start_index
            distance = math.hypot(selected_waypoint[0] - current_x,
                                selected_waypoint[1] - current_y)
            self.get_logger().warning(
                f'Waypoint at index {start_index} exceeds max distance ({distance:.2f}m > {self.max_waypoint_distance}m), '
                f'using it anyway to make progress')

        if selected_waypoint:
            # Update current waypoint index to the selected one
            self.current_waypoint_index = selected_index
            self.get_logger().debug(
                f'Selected waypoint {selected_index + 1}/{len(path.poses)}: '
                f'({selected_waypoint[0]:.2f}, {selected_waypoint[1]:.2f})')

        return selected_waypoint

    def publish_waypoint(self, waypoint: Tuple[float, float]):
        """Publish waypoint as position command."""
        # Reset position reached flag
        with self.position_reached_lock:
            self.position_reached = False

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.point.x = waypoint[0]
        msg.point.y = waypoint[1]
        msg.point.z = self.flight_height

        self.cmd_pos_pub.publish(msg)
        self.get_logger().debug(f'Published waypoint: ({waypoint[0]:.2f}, {waypoint[1]:.2f})')

        # Publish marker for visualization
        self.publish_waypoint_marker(waypoint)

    def publish_waypoint_marker(self, waypoint: Tuple[float, float]):
        """Publish marker for current target waypoint."""
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'nav_waypoint'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = waypoint[0]
        marker.pose.position.y = waypoint[1]
        marker.pose.position.z = self.flight_height
        marker.pose.orientation.w = 1.0

        # Scale (size of sphere)
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        # Color (blue for navigation waypoint)
        marker.color.r = 0.0
        marker.color.g = 0.5
        marker.color.b = 1.0
        marker.color.a = 0.8

        # Lifetime (0 = forever)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        self.waypoint_marker_pub.publish(marker)

    def wait_for_position_reached(self, timeout: float = 30.0) -> bool:
        """Wait for position_target_reached signal."""
        start_time = time.time()

        while rclpy.ok():
            with self.position_reached_lock:
                if self.position_reached:
                    return True

            # Check timeout
            if time.time() - start_time > timeout:
                return False

            time.sleep(0.1)

        return False

    def is_waypoint_goal(self, waypoint: Tuple[float, float]) -> bool:
        """Check if waypoint is the goal."""
        if self.goal_pose is None:
            return False

        goal_x = self.goal_pose.pose.position.x
        goal_y = self.goal_pose.pose.position.y

        distance = math.hypot(waypoint[0] - goal_x, waypoint[1] - goal_y)

        # Consider waypoint as goal if very close (within 0.1m)
        return distance < 0.1


def main(args=None):
    rclpy.init(args=args)
    node = NavNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_navigation()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
