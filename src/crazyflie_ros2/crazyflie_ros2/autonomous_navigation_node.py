#!/usr/bin/env python3
"""
Autonomous Navigation Node for Crazyflie (with Active Scanning Controller)

- Plans paths on a 2D occupancy grid (A* via pyastar2d).
- While planning/executing, the drone continuously spins (active scanning) at a specified yaw rate.
- A wrapper controller transforms world-frame planned velocity into body-frame commands that
  account for the ongoing spin (optionally with lookahead).

New topics (for introspection):
  /nav/cmd_vel_plan     (geometry_msgs/Twist)   -- planned velocity in WORLD frame
  /nav/cmd_vel_active   (geometry_msgs/Twist)   -- body-frame command sent to /cmd_vel

Parameters (added):
  active_scanning.enabled         (bool, default: true)
  active_scanning.lookahead_tau   (double, default: 0.3)  # s, yaw lookahead horizon
  active_scanning.vz_kp           (double, default: 1.0)  # proportional gain to hold flight_height
  active_scanning.speed_scale     (double, default: 1.0)  # scale planned speed (0..1.5)

This file is a patch of your original autonomous_navigation_node.py.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist, Point
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker

import numpy as np
import math
import threading
from collections import deque

try:
    import pyastar2d
    PYASTAR_AVAILABLE = True
except ImportError:
    PYASTAR_AVAILABLE = False


class AsyncPathPlanner:
    """Background thread for A* path planning."""

    def __init__(self, logger):
        self.logger = logger
        self.current_path = deque()
        self.planning_lock = threading.Lock()
        self.should_plan = threading.Event()
        self.stop_planning = threading.Event()
        self.planning_thread = None
        self.occupancy_grid = None
        self.start_pos = None
        self.goal_pos = None

    def start(self):
        if self.planning_thread is None or not self.planning_thread.is_alive():
            self.stop_planning.clear()
            self.planning_thread = threading.Thread(target=self._planning_worker)
            self.planning_thread.daemon = True
            self.planning_thread.start()
            self.logger.info("Path planning thread started")

    def stop(self):
        self.stop_planning.set()
        if self.planning_thread and self.planning_thread.is_alive():
            self.planning_thread.join(timeout=1.0)
            self.logger.info("Path planning thread stopped")

    def request_path(self, occupancy_grid, start_pos, goal_pos):
        with self.planning_lock:
            self.occupancy_grid = occupancy_grid
            self.start_pos = start_pos
            self.goal_pos = goal_pos
            self.should_plan.set()

    def get_current_path(self):
        with self.planning_lock:
            return list(self.current_path)

    def has_path(self):
        with self.planning_lock:
            return len(self.current_path) > 0

    def _planning_worker(self):
        while not self.stop_planning.is_set():
            if self.should_plan.wait(timeout=0.1):
                self.should_plan.clear()

                if not PYASTAR_AVAILABLE:
                    self.logger.warning("pyastar2d not available, skipping planning")
                    continue

                with self.planning_lock:
                    if (self.occupancy_grid is None or
                        self.start_pos is None or
                        self.goal_pos is None):
                        continue

                    try:
                        weights = np.ones_like(self.occupancy_grid, dtype=np.float32)
                        weights[self.occupancy_grid > 50] = np.inf
                        weights[self.occupancy_grid < 0] = 2.0

                        if not self._is_valid_position(self.start_pos, weights.shape):
                            self.logger.warning(f"Start {self.start_pos} OOB for shape {weights.shape}")
                            self.current_path.clear(); continue

                        if not self._is_valid_position(self.goal_pos, weights.shape):
                            self.logger.warning(f"Goal {self.goal_pos} OOB for shape {weights.shape}")
                            self.current_path.clear(); continue

                        start_weight = weights[self.start_pos[1], self.start_pos[0]]
                        goal_weight = weights[self.goal_pos[1], self.goal_pos[0]]

                        weights[self.start_pos[1], self.start_pos[0]] = 1.0

                        if np.isinf(weights[self.goal_pos[1], self.goal_pos[0]]):
                            self.logger.info("Goal blocked, searching nearest free cell")
                            new_goal = self._find_nearest_free_cell(weights, self.goal_pos)
                            if new_goal is None:
                                self.logger.warning("No free space near goal")
                                self.current_path.clear(); continue
                            self.goal_pos = new_goal
                            self.logger.info(f"New goal @ {new_goal}")

                        start = (self.start_pos[1], self.start_pos[0])
                        goal = (self.goal_pos[1], self.goal_pos[0])

                        path = pyastar2d.astar_path(weights, start, goal, allow_diagonal=True)

                        if path is not None and len(path) > 1:
                            self.current_path.clear()
                            for point in path[1:]:
                                waypoint = (point[1], point[0])
                                self.current_path.append(waypoint)
                            self.logger.info(f"Path found with {len(path)-1} waypoints")
                        else:
                            self.logger.warning("No path found")
                            self.current_path.clear()

                    except Exception as e:
                        self.logger.error(f"Path planning error: {e}", exc_info=True)
                        self.current_path.clear()

    def _is_valid_position(self, pos, shape):
        return 0 <= pos[0] < shape[1] and 0 <= pos[1] < shape[0]

    def _find_nearest_free_cell(self, weights, goal):
        goal_x, goal_y = goal
        max_search_radius = 10
        for radius in range(1, max_search_radius + 1):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if abs(dx) == radius or abs(dy) == radius:
                        new_x = goal_x + dx
                        new_y = goal_y + dy
                        if (0 <= new_x < weights.shape[1] and
                            0 <= new_y < weights.shape[0] and
                            np.isfinite(weights[new_y, new_x]) and
                            weights[new_y, new_x] >= 1.0):
                            return (new_x, new_y)
        return None


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
    """ROS2 node for autonomous navigation using OctoMap with active scanning wrapper."""

    def __init__(self):
        super().__init__('autonomous_navigation_node')

        if not PYASTAR_AVAILABLE:
            self.get_logger().error("pyastar2d is not installed. pip install pyastar2d")

        # Parameters
        self.declare_parameter('waypoint_tolerance', 0.2)
        self.declare_parameter('planning_frequency', 2.0)
        self.declare_parameter('scanning_yaw_rate', 0.5)
        self.declare_parameter('flight_height', 0.3)

        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        self.planning_freq = self.get_parameter('planning_frequency').value
        self.scanning_yaw_rate = self.get_parameter('scanning_yaw_rate').value
        self.flight_height = self.get_parameter('flight_height').value

        # State
        self.enabled = False
        self.current_pose = None
        self.goal_pose = None
        self.occupancy_grid_msg = None
        self.current_waypoint = None

        # Planner + Controller
        self.path_planner = AsyncPathPlanner(self.get_logger())
        if PYASTAR_AVAILABLE:
            self.path_planner.start()

        self.controller = ActiveScanningController(self)

        # QoS
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.map_sub = self.create_subscription(OccupancyGrid, '/crazyflie/map', self.map_callback, map_qos)
        self.pose_sub = self.create_subscription(PoseStamped, '/crazyflie/pose', self.pose_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/autonomous_path', 10)
        self.goal_marker_pub = self.create_publisher(Marker, '/goal_marker', 10)

        # Services
        self.enable_srv = self.create_service(SetBool, '/enable_autonomous', self.enable_callback)

        # Timers
        self.control_timer = self.create_timer(0.1, self.control_loop)                  # 10 Hz
        self.planning_timer = self.create_timer(1.0 / self.planning_freq, self.planning_loop)
        self.status_timer = self.create_timer(5.0, self.status_loop)

        self.get_logger().info('Autonomous Navigation Node (active scanning) initialized')
        self.get_logger().info('Waiting for occupancy grid on /crazyflie/map...')

    # ----- Callbacks -----
    def map_callback(self, msg):
        was_none = self.occupancy_grid_msg is None
        self.occupancy_grid_msg = msg
        if was_none:
            self.get_logger().info(f'Received first occupancy grid: {msg.info.width}x{msg.info.height}, res={msg.info.resolution}m')

    def pose_callback(self, msg):
        was_none = self.current_pose is None
        self.current_pose = msg
        if was_none:
            self.get_logger().info(f'Received first pose: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})')

    def goal_callback(self, msg):
        self.goal_pose = msg
        self.current_waypoint = None
        self.get_logger().info(f'New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        self._publish_goal_marker()

    def enable_callback(self, request, response):
        self.enabled = request.data
        response.success = True
        response.message = f"Autonomous navigation {'enabled' if self.enabled else 'disabled'}"
        self.get_logger().info(response.message)
        if not self.enabled:
            self._publish_zero_velocity()
            self.current_waypoint = None
            self.get_logger().info("Navigation disabled - stopping drone")
        return response

    # ----- Loops -----
    def planning_loop(self):
        if not (self.enabled and PYASTAR_AVAILABLE and self.occupancy_grid_msg and self.current_pose and self.goal_pose):
            return

        start_world = (self.current_pose.pose.position.x, self.current_pose.pose.position.y)
        goal_world = (self.goal_pose.pose.position.x, self.goal_pose.pose.position.y)
        start_grid = self._world_to_grid(*start_world)
        goal_grid = self._world_to_grid(*goal_world)

        if start_grid is None:
            clamped = self._clamp_to_grid_bounds(*start_world)
            if clamped is None: return
            start_grid = self._world_to_grid(*clamped)
            if start_grid is None: return

        if goal_grid is None:
            clamped = self._clamp_to_grid_bounds(*goal_world)
            if clamped is None: return
            goal_grid = self._world_to_grid(*clamped)
            if goal_grid is None: return

        width = self.occupancy_grid_msg.info.width
        height = self.occupancy_grid_msg.info.height
        grid_array = np.array(self.occupancy_grid_msg.data).reshape((height, width))

        self.path_planner.request_path(grid_array, start_grid, goal_grid)
        self._publish_path()

    def control_loop(self):
        if not self.enabled or self.current_pose is None:
            return

        path = self.path_planner.get_current_path()
        if not path:
            self._publish_zero_velocity(); return

        if self.current_waypoint is None and len(path) > 0:
            self.current_waypoint = path[0]
            self.get_logger().info(f"Moving to first waypoint: {self.current_waypoint}")

        if self.current_waypoint is None:
            self._publish_zero_velocity(); return

        waypoint_world = self._grid_to_world(self.current_waypoint[0], self.current_waypoint[1])
        if waypoint_world is None:
            self._publish_zero_velocity(); return

        # Direction in WORLD frame
        dx = waypoint_world[0] - self.current_pose.pose.position.x
        dy = waypoint_world[1] - self.current_pose.pose.position.y
        dist = math.hypot(dx, dy)

        if dist < self.waypoint_tolerance:
            self.get_logger().info(f"Waypoint reached, d={dist:.2f}m")
            path = self.path_planner.get_current_path()
            if len(path) > 1:
                self.current_waypoint = path[1]
            else:
                self.get_logger().info("Goal reached!")
                self.current_waypoint = None
                self._publish_zero_velocity()
                return
            return  # wait next tick after updating waypoint

        # Plan a world-frame velocity toward the waypoint (unit vector)
        v_dir = np.array([dx, dy], dtype=float) / max(dist, 1e-6)
        v_mag = 1.0  # nominal speed command (m/s) before scaling; keep modest for CF
        v_world = np.array([v_mag * v_dir[0], v_mag * v_dir[1], 0.0], dtype=float)

        # Wrap through the ActiveScanningController
        cmd = self.controller.compute(self.current_pose, v_world, self.scanning_yaw_rate, self.flight_height)

        # Publish the command to the Crazyflie
        self.cmd_vel_pub.publish(cmd)

    # ----- Utilities -----
    def _world_to_grid(self, x, y):
        if self.occupancy_grid_msg is None: return None
        info = self.occupancy_grid_msg.info
        gx = int((x - info.origin.position.x) / info.resolution)
        gy = int((y - info.origin.position.y) / info.resolution)
        if 0 <= gx < info.width and 0 <= gy < info.height:
            return (gx, gy)
        return None

    def _grid_to_world(self, gx, gy):
        if self.occupancy_grid_msg is None: return None
        info = self.occupancy_grid_msg.info
        wx = gx * info.resolution + info.origin.position.x
        wy = gy * info.resolution + info.origin.position.y
        return (wx, wy)

    def _clamp_to_grid_bounds(self, x, y):
        if self.occupancy_grid_msg is None: return None
        info = self.occupancy_grid_msg.info
        min_x = info.origin.position.x
        max_x = info.origin.position.x + (info.width * info.resolution)
        min_y = info.origin.position.y
        max_y = info.origin.position.y + (info.height * info.resolution)
        cx = max(min_x, min(x, max_x - info.resolution))
        cy = max(min_y, min(y, max_y - info.resolution))
        return (cx, cy)

    def status_loop(self):
        if not self.enabled: return
        parts = ["ENABLED"]
        if self.current_pose is None: parts.append("⚠ NO_POSE")
        else: parts.append(f"pose=({self.current_pose.pose.position.x:.1f},{self.current_pose.pose.position.y:.1f})")
        if self.occupancy_grid_msg is None: parts.append("⚠ NO_MAP")
        else: parts.append(f"map={self.occupancy_grid_msg.info.width}x{self.occupancy_grid_msg.info.height}")
        if self.goal_pose is None: parts.append("⚠ NO_GOAL")
        else: parts.append(f"goal=({self.goal_pose.pose.position.x:.1f},{self.goal_pose.pose.position.y:.1f})")
        path = self.path_planner.get_current_path()
        parts.append(f"path={len(path)} waypoints" if path else "⚠ NO_PATH")
        self.get_logger().info("Status: " + " | ".join(parts))

    def _publish_zero_velocity(self):
        self.cmd_vel_pub.publish(Twist())

    def _publish_path(self):
        path = self.path_planner.get_current_path()
        if not path or self.occupancy_grid_msg is None:
            return
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.occupancy_grid_msg.header.frame_id
        for wp in path:
            wx, wy = self._grid_to_world(wp[0], wp[1])
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.position.z = self.flight_height
            path_msg.poses.append(ps)
        self.path_pub.publish(path_msg)

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

    def destroy_node(self):
        self.path_planner.stop()
        super().destroy_node()


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