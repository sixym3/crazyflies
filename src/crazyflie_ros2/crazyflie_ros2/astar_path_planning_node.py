#!/usr/bin/env python3
"""
A* Path Planning Node for Crazyflie

Subscribes to:
  /crazyflie/map (nav_msgs/OccupancyGrid)
  /crazyflie/pose (geometry_msgs/PoseStamped)
  /goal_pose (geometry_msgs/PoseStamped)

Publishes:
  /planned_path (nav_msgs/Path) - waypoints for autonomous navigation
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

import numpy as np
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
                        # Initialize weights array
                        weights = np.ones_like(self.occupancy_grid, dtype=np.float32)

                        # Convert occupancy grid values to path planning weights:
                        # - Values 0-50: Use as relative weights (1.0 for free, higher for near obstacles)
                        # - Values > 50: Blocked obstacle (infinite weight)
                        # - Values < 0: Unknown areas (slight penalty)

                        # Handle obstacles (values > 50)
                        weights[self.occupancy_grid > 50] = np.inf

                        # Handle unknown areas (values < 0)
                        weights[self.occupancy_grid < 0] = 2.0

                        # Handle weighted avoidance zones (values 1-50)
                        # Scale these to reasonable path costs: 1-50 → 1.0-10.0
                        avoidance_mask = (self.occupancy_grid >= 1) & (self.occupancy_grid <= 50)
                        if np.any(avoidance_mask):
                            # Linear scaling: occupancy value 1-50 maps to weight 1.0-10.0
                            weights[avoidance_mask] = 1.0 + (self.occupancy_grid[avoidance_mask] / 50.0) * 9.0

                        # Handle free space (value 0)
                        weights[self.occupancy_grid == 0] = 1.0

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


class AStarPathPlanningNode(Node):
    """ROS2 node for A* path planning on occupancy grids."""

    def __init__(self):
        super().__init__('astar_path_planning_node')

        if not PYASTAR_AVAILABLE:
            self.get_logger().error("pyastar2d is not installed. pip install pyastar2d")

        # Parameters
        self.declare_parameter('planning_frequency', 2.0)
        self.declare_parameter('flight_height', 0.3)

        self.planning_freq = self.get_parameter('planning_frequency').value
        self.flight_height = self.get_parameter('flight_height').value

        # State
        self.current_pose = None
        self.goal_pose = None
        self.occupancy_grid_msg = None

        # Planner
        self.path_planner = AsyncPathPlanner(self.get_logger())
        if PYASTAR_AVAILABLE:
            self.path_planner.start()

        # QoS
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/crazyflie/map', self.map_callback, map_qos)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/crazyflie/pose', self.pose_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)

        # Publishers
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        # Timers
        self.planning_timer = self.create_timer(
            1.0 / self.planning_freq, self.planning_loop)
        self.status_timer = self.create_timer(5.0, self.status_loop)

        self.get_logger().info('A* Path Planning Node initialized')
        self.get_logger().info('Waiting for occupancy grid on /crazyflie/map...')

    # ----- Callbacks -----
    def map_callback(self, msg):
        was_none = self.occupancy_grid_msg is None
        self.occupancy_grid_msg = msg
        if was_none:
            self.get_logger().info(
                f'Received first occupancy grid: {msg.info.width}x{msg.info.height}, '
                f'res={msg.info.resolution}m')

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

    # ----- Planning Loop -----
    def planning_loop(self):
        if not (PYASTAR_AVAILABLE and self.occupancy_grid_msg and
                self.current_pose and self.goal_pose):
            return

        start_world = (self.current_pose.pose.position.x,
                      self.current_pose.pose.position.y)
        goal_world = (self.goal_pose.pose.position.x,
                     self.goal_pose.pose.position.y)

        start_grid = self._world_to_grid(*start_world)
        goal_grid = self._world_to_grid(*goal_world)

        if start_grid is None:
            clamped = self._clamp_to_grid_bounds(*start_world)
            if clamped is None:
                return
            start_grid = self._world_to_grid(*clamped)
            if start_grid is None:
                return

        if goal_grid is None:
            clamped = self._clamp_to_grid_bounds(*goal_world)
            if clamped is None:
                return
            goal_grid = self._world_to_grid(*clamped)
            if goal_grid is None:
                return

        width = self.occupancy_grid_msg.info.width
        height = self.occupancy_grid_msg.info.height
        grid_array = np.array(self.occupancy_grid_msg.data).reshape((height, width))

        self.path_planner.request_path(grid_array, start_grid, goal_grid)
        self._publish_path()

    def status_loop(self):
        parts = []
        if self.current_pose is None:
            parts.append("⚠ NO_POSE")
        else:
            parts.append(f"pose=({self.current_pose.pose.position.x:.1f},"
                        f"{self.current_pose.pose.position.y:.1f})")

        if self.occupancy_grid_msg is None:
            parts.append("⚠ NO_MAP")
        else:
            parts.append(f"map={self.occupancy_grid_msg.info.width}x"
                        f"{self.occupancy_grid_msg.info.height}")

        if self.goal_pose is None:
            parts.append("⚠ NO_GOAL")
        else:
            parts.append(f"goal=({self.goal_pose.pose.position.x:.1f},"
                        f"{self.goal_pose.pose.position.y:.1f})")

        path = self.path_planner.get_current_path()
        parts.append(f"path={len(path)} waypoints" if path else "⚠ NO_PATH")

        self.get_logger().info("Status: " + " | ".join(parts))

    # ----- Utilities -----
    def _world_to_grid(self, x, y):
        if self.occupancy_grid_msg is None:
            return None
        info = self.occupancy_grid_msg.info
        gx = int((x - info.origin.position.x) / info.resolution)
        gy = int((y - info.origin.position.y) / info.resolution)
        if 0 <= gx < info.width and 0 <= gy < info.height:
            return (gx, gy)
        return None

    def _grid_to_world(self, gx, gy):
        if self.occupancy_grid_msg is None:
            return None
        info = self.occupancy_grid_msg.info
        wx = gx * info.resolution + info.origin.position.x
        wy = gy * info.resolution + info.origin.position.y
        return (wx, wy)

    def _clamp_to_grid_bounds(self, x, y):
        if self.occupancy_grid_msg is None:
            return None
        info = self.occupancy_grid_msg.info
        min_x = info.origin.position.x
        max_x = info.origin.position.x + (info.width * info.resolution)
        min_y = info.origin.position.y
        max_y = info.origin.position.y + (info.height * info.resolution)
        cx = max(min_x, min(x, max_x - info.resolution))
        cy = max(min_y, min(y, max_y - info.resolution))
        return (cx, cy)

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
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)

        self.path_pub.publish(path_msg)

    def destroy_node(self):
        self.path_planner.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AStarPathPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
