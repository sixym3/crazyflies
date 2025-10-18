#!/usr/bin/env python3
"""
Autonomous Navigation Node for Crazyflie (Finite State Machine with Active Scanning)

Subscribes to path waypoints from a separate path planning node and executes
finite state machine control (IDLE, TAKEOFF, FOLLOW, LAND) with active scanning.

Subscribes to:
  /planned_path (nav_msgs/Path) - waypoints from path planner
  /crazyflie/pose (geometry_msgs/PoseStamped)
  /goal_pose (geometry_msgs/PoseStamped)
  /perception/edge_event (std_msgs/Bool)

Publishes:
  /cmd_vel (geometry_msgs/Twist)
  /nav/cmd_vel_plan (geometry_msgs/Twist)
  /nav/cmd_vel_active (geometry_msgs/Twist)
  /goal_marker (visualization_msgs/Marker)
  /crazyflie/emergency_stop (std_msgs/Bool)
"""

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker

import numpy as np
import math
from std_msgs.msg import Bool


class ActiveScanningController:
    """
    Wrapper that converts world-frame planned velocity into body-frame cmd_vel
    while applying a continuous yaw spin for active sensing. Includes a small
    lookahead to compensate for the rotation that will occur before command
    execution takes effect.
    """

    def __init__(self, node: Node):
        self.node = node
        # Parameters
        node.declare_parameter('active_scanning.enabled', True)
        node.declare_parameter('active_scanning.lookahead_tau', 0.3)
        node.declare_parameter('active_scanning.vz_kp', 1.0)
        node.declare_parameter('active_scanning.speed_scale', 1.0)

        self.enabled = bool(node.get_parameter('active_scanning.enabled').value)
        self.lookahead_tau = float(node.get_parameter('active_scanning.lookahead_tau').value)
        self.vz_kp = float(node.get_parameter('active_scanning.vz_kp').value)
        self.speed_scale = float(node.get_parameter('active_scanning.speed_scale').value)

        # Publishers for introspection
        self.plan_pub = node.create_publisher(Twist, '/nav/cmd_vel_plan', 10)
        self.active_pub = node.create_publisher(Twist, '/nav/cmd_vel_active', 10)

    @staticmethod
    def _yaw_from_quat(q):
        w, x, y, z = q.w, q.x, q.y, q.z
        return math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))

    def compute(self, pose_msg: PoseStamped, v_world: np.ndarray, yaw_rate: float, flight_height: float) -> Twist:
        """
        Args:
          pose_msg: current pose (for yaw, z)
          v_world: np.array([vx_w, vy_w, vz_w]) desired in WORLD frame (m/s)
          yaw_rate: desired spin (rad/s), positive CCW
          flight_height: target altitude (m)
        Returns:
          Twist in BODY frame with angular.z set to yaw_rate
        """
        cmd_plan = Twist()
        cmd_plan.linear.x, cmd_plan.linear.y, cmd_plan.linear.z = float(v_world[0]), float(v_world[1]), float(v_world[2])
        cmd_plan.angular.z = float(yaw_rate)
        self.plan_pub.publish(cmd_plan)

        if not self.enabled or pose_msg is None:
            # Fall back to direct world->body using current yaw
            current_yaw = 0.0 if pose_msg is None else self._yaw_from_quat(pose_msg.pose.orientation)
            R = np.array([[ math.cos(-current_yaw), -math.sin(-current_yaw) ],
                          [ math.sin(-current_yaw),  math.cos(-current_yaw) ]])
            v_body_xy = R @ v_world[:2]
        else:
            # Predict yaw a little into the future (lookahead)
            current_yaw = self._yaw_from_quat(pose_msg.pose.orientation)
            yaw_pred = current_yaw + yaw_rate * self.lookahead_tau
            R = np.array([[ math.cos(-yaw_pred), -math.sin(-yaw_pred) ],
                          [ math.sin(-yaw_pred),  math.cos(-yaw_pred) ]])
            v_body_xy = R @ v_world[:2]

        # Scale speed if requested
        v_body_xy = self.speed_scale * v_body_xy

        # Simple altitude hold around flight_height
        z_err = (flight_height - pose_msg.pose.position.z) if pose_msg is not None else 0.0
        vz_cmd = self.vz_kp * z_err  # bounded naturally by CF node's handling

        cmd = Twist()
        cmd.linear.x = float(v_body_xy[0])
        cmd.linear.y = float(v_body_xy[1])
        cmd.linear.z = float(vz_cmd)
        cmd.angular.z = float(yaw_rate)

        self.active_pub.publish(cmd)
        return cmd


class AutonomousNavigationNode(Node):
    """ROS2 node for autonomous navigation FSM with active scanning wrapper."""

    def __init__(self):
        super().__init__('autonomous_navigation_node')

        # Parameters
        self.declare_parameter('waypoint_tolerance', 0.2)
        self.declare_parameter('scanning_yaw_rate', 1.0)
        self.declare_parameter('flight_height', 0.3)

        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.scanning_yaw_rate = self.get_parameter('scanning_yaw_rate').value
        self.flight_height = self.get_parameter('flight_height').value

        self.state = 'IDLE'   # IDLE, TAKEOFF, FOLLOW, LAND

        # State
        self.enabled = False
        self.current_pose = None
        self.goal_pose = None
        self.planned_path = []
        self.current_waypoint_idx = 0

        # Edge detection during FOLLOW
        self.follow_start_time = None
        self.edge_detection_delay = 2.0  # seconds to wait before activating edge detection

        # Controller
        self.controller = ActiveScanningController(self)

        # Subscribers
        self.pose_sub = self.create_subscription(PoseStamped, '/crazyflie/pose', self.pose_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.path_sub = self.create_subscription(Path, '/planned_path', self.path_callback, 10)
        self.edge_event_sub = self.create_subscription(Bool, '/perception/edge_event', self.edge_cb, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_marker_pub = self.create_publisher(Marker, '/goal_marker', 10)
        self.emergency_stop_pub = self.create_publisher(Bool, '/crazyflie/emergency_stop', 10)

        # Services
        self.enable_srv = self.create_service(SetBool, '/enable_autonomous', self.enable_callback)

        # Timers
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        self.status_timer = self.create_timer(5.0, self.status_loop)

        self.get_logger().info('Autonomous Navigation Node (FSM + Active Scanning) initialized')
        self.get_logger().info(f'Initial state: {self.state}')
        self.get_logger().info('Waiting for planned path on /planned_path...')

    # ----- Callbacks -----
    def pose_callback(self, msg):
        was_none = self.current_pose is None
        self.current_pose = msg
        if was_none:
            self.get_logger().info(
                f'Received first pose: ({msg.pose.position.x:.2f}, '
                f'{msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})')

    def goal_callback(self, msg):
        self.goal_pose = msg
        self.get_logger().info(
            f'New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        self._publish_goal_marker()

    def path_callback(self, msg: Path):
        """Receive planned path from path planning node."""
        if len(msg.poses) > 0:
            self.planned_path = msg.poses
            self.current_waypoint_idx = 0
            self.get_logger().info(f'Received path with {len(msg.poses)} waypoints')

    def enable_callback(self, request, response):
        self.enabled = request.data
        response.success = True
        response.message = f"Autonomous navigation {'enabled' if self.enabled else 'disabled'}"
        self.get_logger().info(response.message)
        if not self.enabled:
            self._publish_zero_velocity()
            self.current_waypoint_idx = 0
            self.get_logger().info("Navigation disabled - stopping drone")
        return response


    def edge_cb(self, msg: Bool):
        """Handle edge detection events - transition to LAND when edge detected during FOLLOW."""
        if not self.enabled or not msg.data:
            return

        # During FOLLOW state, check if edge detection is active (after delay)
        if self.state == 'FOLLOW':
            if self.follow_start_time is None:
                return

            current_time = self.get_clock().now().nanoseconds / 1e9
            elapsed = current_time - self.follow_start_time

            if elapsed < self.edge_detection_delay:
                return  # Not ready yet

            # Edge detected during FOLLOW - transition directly to LAND
            self.get_logger().info('EDGE detected in FOLLOW → STATE CHANGE: FOLLOW -> LAND')
            self.state = 'LAND'
            self._publish_zero_velocity()
            return

    # ----- Control Loop -----
    def control_loop(self):
        if not self.enabled:
            # Log every 100 calls (~10 seconds) to avoid spam
            if not hasattr(self, '_disabled_log_count'):
                self._disabled_log_count = 0
            self._disabled_log_count += 1
            if self._disabled_log_count >= 100:
                self.get_logger().warn(f'Control loop blocked: enabled={self.enabled}')
                self._disabled_log_count = 0
            return

        if self.current_pose is None:
            if not hasattr(self, '_no_pose_log_count'):
                self._no_pose_log_count = 0
            self._no_pose_log_count += 1
            if self._no_pose_log_count >= 100:
                self.get_logger().warn(f'Control loop blocked: no pose data')
                self._no_pose_log_count = 0
            return

        # Simple state transitions (expand as you like)
        if self.state == 'IDLE':
            if self.goal_pose is not None:
                self.get_logger().info(f'STATE CHANGE: IDLE -> TAKEOFF')
                self.state = 'TAKEOFF'
            else:
                # Don't publish zero velocity in IDLE - allow manual control
                return

        if self.state == 'TAKEOFF':
            # altitude hold is already done by controller via flight_height
            # just wait until close to target height, then follow path
            # Log altitude status periodically
            if not hasattr(self, '_takeoff_log_count'):
                self._takeoff_log_count = 0
            self._takeoff_log_count += 1
            if self._takeoff_log_count >= 50:  # Log every 5 seconds
                alt_diff = abs(self.current_pose.pose.position.z - self.flight_height)
                self.get_logger().info(f'TAKEOFF: current_alt={self.current_pose.pose.position.z:.3f}m, target={self.flight_height:.3f}m, diff={alt_diff:.3f}m')
                self._takeoff_log_count = 0

            if abs(self.current_pose.pose.position.z - self.flight_height) < 0.05:
                self.get_logger().info(f'STATE CHANGE: TAKEOFF -> FOLLOW (altitude reached: {self.current_pose.pose.position.z:.3f}m)')
                self.state = 'FOLLOW'
                self.follow_start_time = self.get_clock().now().nanoseconds / 1e9
            else:
                # Send command to climb to target altitude
                v_world = np.array([0.0, 0.0, 0.0], dtype=float)  # No horizontal movement during takeoff
                cmd = self.controller.compute(self.current_pose, v_world, self.scanning_yaw_rate, self.flight_height)
                self.cmd_vel_pub.publish(cmd)
            return



        if self.state == 'LAND':
            if not hasattr(self, '_last_land_log') or (self.get_clock().now().nanoseconds - self._last_land_log) > 5e8:
                self.get_logger().info(f'LAND: z={self.current_pose.pose.position.z:.2f} → target={self.flight_height:.2f}')
                self._last_land_log = self.get_clock().now().nanoseconds

            # Gradually lower the target altitude
            self.flight_height = max(0.12, self.flight_height - 0.01)  # Descend ~0.1 m/s at 10 Hz

            # Send command with lowered altitude setpoint through controller
            v_world = np.array([0.0, 0.0, 0.0], dtype=float)  # No horizontal movement
            cmd = self.controller.compute(self.current_pose, v_world, 0.0, self.flight_height)
            self.cmd_vel_pub.publish(cmd)

            # Check if we've reached minimum safe hover height
            if self.flight_height <= 0.12 or self.current_pose.pose.position.z < 0.15:
                self.get_logger().info("Landing complete - sending emergency stop")
                self._publish_zero_velocity()
                # Send emergency stop to kill motors
                stop_msg = Bool()
                stop_msg.data = True
                self.emergency_stop_pub.publish(stop_msg)
                self.enabled = False  # Disable autonomous control
                self.state = 'IDLE'
            return

        # Normal FOLLOW behavior - follow planned path waypoints
        if not self.planned_path:
            self._publish_zero_velocity()
            return

        # Check if we've reached the end of the path
        if self.current_waypoint_idx >= len(self.planned_path):
            self.get_logger().info('STATE CHANGE: FOLLOW -> LAND (goal reached)')
            self.state = 'LAND'
            self._publish_zero_velocity()
            return

        # Get current waypoint
        waypoint = self.planned_path[self.current_waypoint_idx]
        dx = waypoint.pose.position.x - self.current_pose.pose.position.x
        dy = waypoint.pose.position.y - self.current_pose.pose.position.y
        dist = math.hypot(dx, dy)

        # Check if we've reached the current waypoint
        if dist < self.waypoint_tolerance:
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.planned_path):
                self.get_logger().info('STATE CHANGE: FOLLOW -> LAND (goal reached)')
                self.state = 'LAND'
                self._publish_zero_velocity()
            return

        # Move towards current waypoint
        v_dir = np.array([dx, dy], dtype=float) / max(dist, 1e-6)
        v_world = np.array([v_dir[0], v_dir[1], 0.0], dtype=float)
        cmd = self.controller.compute(self.current_pose, v_world,
                                      self.scanning_yaw_rate, self.flight_height)
        self.cmd_vel_pub.publish(cmd)


    # ----- Utilities -----
    def status_loop(self):
        if not self.enabled:
            return

        parts = ["ENABLED", f"state={self.state}"]

        if self.current_pose is None:
            parts.append("⚠ NO_POSE")
        else:
            parts.append(f"pose=({self.current_pose.pose.position.x:.1f},"
                        f"{self.current_pose.pose.position.y:.1f})")

        if self.goal_pose is None:
            parts.append("⚠ NO_GOAL")
        else:
            parts.append(f"goal=({self.goal_pose.pose.position.x:.1f},"
                        f"{self.goal_pose.pose.position.y:.1f})")

        if not self.planned_path:
            parts.append("⚠ NO_PATH")
        else:
            parts.append(f"path={len(self.planned_path)} waypoints "
                        f"(at {self.current_waypoint_idx})")

        self.get_logger().info("Status: " + " | ".join(parts))

    def _publish_zero_velocity(self):
        self.cmd_vel_pub.publish(Twist())

    def _publish_goal_marker(self):
        if self.goal_pose is None: return
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.goal_pose.header.frame_id
        marker.ns = "goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = self.goal_pose.pose
        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
        marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; marker.color.a = 1.0
        self.goal_marker_pub.publish(marker)



def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()