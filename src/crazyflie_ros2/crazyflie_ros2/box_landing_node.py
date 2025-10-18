#!/usr/bin/env python3
import math, numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

class BoxLandingNode(Node):
    def __init__(self):
        super().__init__('box_landing_node')

        # Params
        self.declare_parameter('enabled', False)
        self.declare_parameter('flight_height', 0.5)
        self.declare_parameter('search_span_m', 0.6)
        self.declare_parameter('search_step_m', 0.15)
        self.declare_parameter('max_descent_rate', 0.08)
        self.declare_parameter('min_hover_height', 0.12)
        self.declare_parameter('min_edge_points', 3)
        self.declare_parameter('goal_tolerance', 0.15)

        self.enabled = bool(self.get_parameter('enabled').value)
        self.flight_height = float(self.get_parameter('flight_height').value)
        self.search_span = float(self.get_parameter('search_span_m').value)
        self.search_step = float(self.get_parameter('search_step_m').value)
        self.vz_max = float(self.get_parameter('max_descent_rate').value)
        self.min_hover = float(self.get_parameter('min_hover_height').value)
        self.min_edge_points = int(self.get_parameter('min_edge_points').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)

        # State
        self.pose = None
        self.phase = 'IDLE'      # IDLE -> SCAN -> NAVIGATE -> DESCEND -> STOP
        self.scan_targets = []
        self.edge_points = []  # List of (x, y) positions where edges detected
        self.box_centroid = None

        # IO
        self.pose_sub = self.create_subscription(PoseStamped, '/crazyflie/pose', self.pose_cb, 10)
        self.edge_event_sub = self.create_subscription(Bool, '/perception/edge_event', self.edge_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.estop_pub = self.create_publisher(Bool, '/crazyflie/emergency_stop', 10)
        self.status_pub = self.create_publisher(String, '/box_landing/status', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Service clients
        self.autonomous_nav_client = self.create_client(SetBool, '/enable_autonomous')

        # Service
        self.enable_srv = self.create_service(SetBool, '/enable_box_landing', self.enable_cb)

        # Timer
        self.timer = self.create_timer(0.1, self.loop)  # 10 Hz

        self.get_logger().info('box_landing_node ready (disabled by default)')

    def enable_cb(self, req, res):
        self.enabled = bool(req.data)
        res.success = True
        res.message = f'box_landing_node {"enabled" if self.enabled else "disabled"}'
        self.get_logger().info(res.message)
        if self.enabled:
            self.phase = 'SCAN'
            self._publish_status('searching')
            self.plan_scan_targets()
            self.edge_points.clear()
            self.box_centroid = None
            self.get_logger().info(f'Starting box landing sequence - SCAN phase initiated')
            self.get_logger().info(f'Generated {len(self.scan_targets)} scan waypoints in lawnmower pattern')
            self.get_logger().info(f'Listening for edge events - need {self.min_edge_points} minimum')
        else:
            self.phase = 'IDLE'
            self.cmd_pub.publish(Twist())
            self._call_autonomous_nav(False)  # Make sure autonomous nav is disabled
            self.get_logger().info('Box landing disabled - stopping')
        return res

    def _publish_status(self, status: str):
        """Publish status for mode manager."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def pose_cb(self, msg):
        was_none = self.pose is None
        self.pose = msg
        if was_none:
            self.get_logger().info(
                f'Received first pose: ({msg.pose.position.x:.2f}, '
                f'{msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})')

    def edge_cb(self, msg: Bool):
        """Handle edge detection events - record position when edge detected."""
        if not msg.data or not self.enabled or self.pose is None:
            return

        # Only record edges during SCAN phase
        if self.phase == 'SCAN':
            x = self.pose.pose.position.x
            y = self.pose.pose.position.y
            self.edge_points.append((x, y))
            self.get_logger().info(
                f'EDGE POINT #{len(self.edge_points)} recorded at ({x:.3f}, {y:.3f})')

            # Check if we have enough points
            if len(self.edge_points) >= self.min_edge_points:
                self.get_logger().info(
                    f'Collected {len(self.edge_points)} edge points - computing centroid')
                self._compute_and_navigate_to_centroid()

    def _call_autonomous_nav(self, enable: bool):
        """Enable/disable autonomous navigation node."""
        if not self.autonomous_nav_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Autonomous navigation service not available')
            return False

        request = SetBool.Request()
        request.data = enable
        future = self.autonomous_nav_client.call_async(request)
        # We don't wait for response, just fire and forget
        self.get_logger().info(f'Called autonomous navigation: {"ENABLE" if enable else "DISABLE"}')
        return True

    def _compute_and_navigate_to_centroid(self):
        """Compute box centroid from edge points and navigate to it."""
        if len(self.edge_points) < self.min_edge_points:
            self.get_logger().warn(
                f'Not enough edge points: {len(self.edge_points)} < {self.min_edge_points}')
            return

        # Compute centroid
        xs = [p[0] for p in self.edge_points]
        ys = [p[1] for p in self.edge_points]
        cx = sum(xs) / len(xs)
        cy = sum(ys) / len(ys)
        self.box_centroid = (cx, cy)

        self.get_logger().info(
            f'Box centroid computed: ({cx:.3f}, {cy:.3f}) from {len(self.edge_points)} edge points')

        # Create and publish goal pose
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'world'
        goal_msg.pose.position.x = cx
        goal_msg.pose.position.y = cy
        goal_msg.pose.position.z = self.flight_height
        goal_msg.pose.orientation.w = 1.0
        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f'Published goal_pose to ({cx:.3f}, {cy:.3f})')

        # Enable autonomous navigation
        self._call_autonomous_nav(True)

        # Transition to NAVIGATE phase
        self.phase = 'NAVIGATE'
        self._publish_status('navigating')
        self.get_logger().info('Transitioning to NAVIGATE phase - autonomous navigation enabled')

    # --- simple local lawnmower around current position (in WORLD XY) ---
    def plan_scan_targets(self):
        if not self.pose:
            self.scan_targets = []
            self.get_logger().warn('Cannot plan scan targets - no pose data')
            return
        x0 = self.pose.pose.position.x
        y0 = self.pose.pose.position.y
        span = self.search_span
        step = self.search_step
        targets = []
        y = -span
        toggle = False
        while y <= span + 1e-6:
            xs = np.arange(-span, span + 1e-6, step)
            if toggle: xs = xs[::-1]
            for dx in xs:
                targets.append((x0 + dx, y0 + y))
            y += step
            toggle = not toggle
        self.scan_targets = targets
        self.get_logger().info(
            f'Scan pattern: {len(targets)} waypoints in {span*2:.1f}m x {span*2:.1f}m area, '
            f'step={step:.2f}m, centered at ({x0:.2f}, {y0:.2f})')

    def loop(self):
        if not self.enabled or self.pose is None:
            return

        if self.phase == 'SCAN':
            # Execute lawnmower scan pattern
            # Edges are detected via edge_cb callback
            if not self.scan_targets:
                # Scan pattern complete but not enough edges yet
                self.get_logger().warn(
                    f'SCAN: Scan pattern complete but only {len(self.edge_points)} edges detected '
                    f'(need {self.min_edge_points}). Waiting for more edges or manual intervention.')
                # Just hover and wait for more edges
                self.cmd_pub.publish(Twist())
                return

            # Navigate to next scan waypoint
            tx, ty = self.scan_targets[0]
            cx = self.pose.pose.position.x
            cy = self.pose.pose.position.y
            dx, dy = (tx - cx), (ty - cy)
            dist = math.hypot(dx, dy)

            # Check if reached waypoint
            if dist < 0.06:
                self.scan_targets.pop(0)
                self.get_logger().info(
                    f'SCAN: Waypoint reached - {len(self.scan_targets)} remaining, '
                    f'{len(self.edge_points)} edges detected')

            # Move toward waypoint
            v = Twist()
            if dist > 1e-3:
                ux, uy = dx / max(dist, 1e-6), dy / max(dist, 1e-6)
                v.linear.x = 0.2 * ux
                v.linear.y = 0.2 * uy
            # Hold altitude
            zerr = self.flight_height - self.pose.pose.position.z
            v.linear.z = max(min(1.0 * zerr, 0.2), -0.2)
            v.angular.z = 0.0

            # Log progress periodically
            if not hasattr(self, '_scan_log_count'):
                self._scan_log_count = 0
            self._scan_log_count += 1
            if self._scan_log_count % 30 == 0:  # Every 3 seconds
                self.get_logger().info(
                    f'SCAN: Target ({tx:.2f}, {ty:.2f}), dist={dist:.3f}m, '
                    f'edges={len(self.edge_points)}, waypoints_left={len(self.scan_targets)}')

            self.cmd_pub.publish(v)
            return

        if self.phase == 'NAVIGATE':
            # Wait for autonomous navigation to reach centroid
            if self.box_centroid is None:
                self.get_logger().error('NAVIGATE: No centroid set!')
                return

            cx, cy = self.box_centroid
            px = self.pose.pose.position.x
            py = self.pose.pose.position.y
            dist = math.hypot(cx - px, cy - py)

            # Log progress periodically
            if not hasattr(self, '_nav_log_count'):
                self._nav_log_count = 0
            self._nav_log_count += 1
            if self._nav_log_count % 20 == 0:  # Every 2 seconds
                self.get_logger().info(
                    f'NAVIGATE: Current ({px:.3f}, {py:.3f}), '
                    f'Target ({cx:.3f}, {cy:.3f}), distance={dist:.3f}m')

            # Check if reached goal
            if dist < self.goal_tolerance:
                self.get_logger().info(
                    f'NAVIGATE: Reached box centroid! Distance={dist:.3f}m < {self.goal_tolerance}m')

                # Disable autonomous navigation
                self._call_autonomous_nav(False)

                # Transition to DESCEND
                self.phase = 'DESCEND'
                self._publish_status('descending')
                self.get_logger().info(f'Transitioning to DESCEND - landing from z={self.pose.pose.position.z:.3f}m')
            return

        if self.phase == 'DESCEND':
            # Direct landing
            v = Twist()
            v.linear.x = 0.0
            v.linear.y = 0.0
            v.linear.z = -self.vz_max
            v.angular.z = 0.0
            self.cmd_pub.publish(v)

            z = self.pose.pose.position.z

            # Log descent progress periodically
            if not hasattr(self, '_descend_log_count'):
                self._descend_log_count = 0
            self._descend_log_count += 1
            if self._descend_log_count % 10 == 0:  # Every 1 second
                self.get_logger().info(
                    f'DESCEND: altitude={z:.3f}m, target={self.min_hover:.3f}m, '
                    f'descent_rate={self.vz_max:.3f}m/s')

            # Check if landing complete
            if z <= (self.min_hover + 0.02):
                self.get_logger().info(
                    f'DESCEND: Landing complete at z={z:.3f}m – sending emergency stop')
                self.cmd_pub.publish(Twist())
                estop = Bool()
                estop.data = True
                self.estop_pub.publish(estop)
                self.phase = 'STOP'
                self._publish_status('completed')
                self.enabled = False
                self.get_logger().info('Box landing sequence complete!')
            return

        if self.phase == 'STOP':
            # Complete - do nothing
            return


def main():
    rclpy.init()
    node = BoxLandingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
