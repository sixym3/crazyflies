#!/usr/bin/env python3
"""
Autonomous Navigation Node for Crazyflie

Subscribes to OctoMap's projected_map and uses A* path planning
to navigate autonomously to goal positions.

TODO for next session:
- Add active scanning/rotation logic to gather more environmental sensor data
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
        """Start the planning thread."""
        if self.planning_thread is None or not self.planning_thread.is_alive():
            self.stop_planning.clear()
            self.planning_thread = threading.Thread(target=self._planning_worker)
            self.planning_thread.daemon = True
            self.planning_thread.start()
            self.logger.info("Path planning thread started")

    def stop(self):
        """Stop the planning thread."""
        self.stop_planning.set()
        if self.planning_thread and self.planning_thread.is_alive():
            self.planning_thread.join(timeout=1.0)
            self.logger.info("Path planning thread stopped")

    def request_path(self, occupancy_grid, start_pos, goal_pos):
        """Request a new path to be computed."""
        with self.planning_lock:
            self.occupancy_grid = occupancy_grid
            self.start_pos = start_pos
            self.goal_pos = goal_pos
            self.should_plan.set()

    def get_current_path(self):
        """Get the current planned path (thread-safe)."""
        with self.planning_lock:
            return list(self.current_path)

    def has_path(self):
        """Check if a path exists."""
        with self.planning_lock:
            return len(self.current_path) > 0

    def _planning_worker(self):
        """Background worker for path planning."""
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
                        # Convert occupancy grid to cost map
                        # OccupancyGrid: -1=unknown, 0=free, 100=occupied
                        # A*: 1+=traversable, inf=impassable
                        weights = np.ones_like(self.occupancy_grid, dtype=np.float32)

                        # Mark obstacles as impassable
                        weights[self.occupancy_grid > 50] = np.inf
                        # Mark unknown as higher cost
                        weights[self.occupancy_grid < 0] = 2.0

                        self.logger.debug(
                            f"_planning_worker: weights array shape={weights.shape}, "
                            f"start_pos={self.start_pos}, goal_pos={self.goal_pos}"
                        )

                        # Ensure start and goal are valid
                        if not self._is_valid_position(self.start_pos, weights.shape):
                            self.logger.warning(
                                f"_planning_worker: Start position {self.start_pos} is OUT OF BOUNDS "
                                f"for weights shape {weights.shape}"
                            )
                            self.current_path.clear()
                            continue

                        if not self._is_valid_position(self.goal_pos, weights.shape):
                            self.logger.warning(
                                f"_planning_worker: Goal position {self.goal_pos} is OUT OF BOUNDS "
                                f"for weights shape {weights.shape}"
                            )
                            self.current_path.clear()
                            continue

                        # Check start and goal values
                        start_weight = weights[self.start_pos[1], self.start_pos[0]]
                        goal_weight = weights[self.goal_pos[1], self.goal_pos[0]]

                        self.logger.debug(
                            f"_planning_worker: start weight={start_weight} "
                            f"({'FREE' if np.isfinite(start_weight) else 'BLOCKED'}), "
                            f"goal weight={goal_weight} "
                            f"({'FREE' if np.isfinite(goal_weight) else 'BLOCKED'})"
                        )

                        # Force start to be free
                        weights[self.start_pos[1], self.start_pos[0]] = 1.0

                        # If goal is blocked, find nearest free cell
                        if np.isinf(weights[self.goal_pos[1], self.goal_pos[0]]):
                            self.logger.info("Goal is blocked, finding nearest free cell")
                            new_goal = self._find_nearest_free_cell(weights, self.goal_pos)
                            if new_goal is None:
                                self.logger.warning("No free space found near goal")
                                self.current_path.clear()
                                continue
                            self.goal_pos = new_goal
                            self.logger.info(f"New goal found at {new_goal}")

                        # Run A*
                        start = (self.start_pos[1], self.start_pos[0])  # (row, col)
                        goal = (self.goal_pos[1], self.goal_pos[0])     # (row, col)

                        self.logger.debug(
                            f"_planning_worker: Running A* from {start} to {goal}"
                        )

                        path = pyastar2d.astar_path(weights, start, goal, allow_diagonal=True)

                        if path is not None and len(path) > 1:
                            # Convert path back to (x, y) and store
                            self.current_path.clear()
                            for point in path[1:]:  # Skip current position
                                waypoint = (point[1], point[0])  # Convert (row,col) to (x,y)
                                self.current_path.append(waypoint)
                            self.logger.info(f"Path found with {len(path)-1} waypoints")
                        else:
                            self.logger.warning(
                                f"No path found from {start} to {goal}. "
                                f"Start free: {np.isfinite(weights[start])}, "
                                f"Goal free: {np.isfinite(weights[goal])}"
                            )
                            self.current_path.clear()

                    except Exception as e:
                        self.logger.error(f"Path planning error: {e}", exc_info=True)
                        self.current_path.clear()

    def _is_valid_position(self, pos, shape):
        """Check if position is within grid bounds."""
        return 0 <= pos[0] < shape[1] and 0 <= pos[1] < shape[0]

    def _find_nearest_free_cell(self, weights, goal):
        """Find the nearest free cell to the goal position."""
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


class AutonomousNavigationNode(Node):
    """ROS2 node for autonomous navigation using OctoMap."""

    def __init__(self):
        super().__init__('autonomous_navigation_node')

        # Check if pyastar2d is available
        if not PYASTAR_AVAILABLE:
            self.get_logger().error(
                "pyastar2d is not installed. Please install with: pip install pyastar2d"
            )

        # Declare parameters
        self.declare_parameter('waypoint_tolerance', 0.2)
        self.declare_parameter('planning_frequency', 2.0)
        self.declare_parameter('scanning_yaw_rate', 0.5)
        self.declare_parameter('flight_height', 0.3)

        # Get parameters
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

        # Path planner
        self.path_planner = AsyncPathPlanner(self.get_logger())
        if PYASTAR_AVAILABLE:
            self.path_planner.start()

        # QoS profiles
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/projected_map',
            self.map_callback,
            map_qos
        )

        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/crazyflie/pose',
            self.pose_callback,
            10
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            '/autonomous_path',
            10
        )

        self.goal_marker_pub = self.create_publisher(
            Marker,
            '/goal_marker',
            10
        )

        # Services
        self.enable_srv = self.create_service(
            SetBool,
            '/enable_autonomous',
            self.enable_callback
        )

        # Timer for navigation control loop
        self.control_timer = self.create_timer(
            0.1,  # 10 Hz control loop
            self.control_loop
        )

        # Timer for path planning
        self.planning_timer = self.create_timer(
            1.0 / self.planning_freq,
            self.planning_loop
        )

        # Timer for status reporting (every 5 seconds)
        self.status_timer = self.create_timer(5.0, self.status_loop)

        self.get_logger().info('Autonomous Navigation Node initialized')
        self.get_logger().info(f'Waiting for occupancy grid on /projected_map...')

    def map_callback(self, msg):
        """Handle occupancy grid updates."""
        was_none = self.occupancy_grid_msg is None
        self.occupancy_grid_msg = msg
        if was_none:
            self.get_logger().info(
                f'Received first occupancy grid: {msg.info.width}x{msg.info.height}, '
                f'resolution: {msg.info.resolution}m'
            )
        elif self.enabled:
            self.get_logger().debug(
                f'Occupancy grid update: {msg.info.width}x{msg.info.height}, '
                f'resolution: {msg.info.resolution}m'
            )

    def pose_callback(self, msg):
        """Handle pose updates."""
        was_none = self.current_pose is None
        self.current_pose = msg
        if was_none:
            self.get_logger().info(
                f'Received first pose: ({msg.pose.position.x:.2f}, '
                f'{msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})'
            )

    def goal_callback(self, msg):
        """Handle goal pose updates."""
        self.goal_pose = msg
        self.current_waypoint = None  # Reset current waypoint
        self.get_logger().info(
            f'New goal received: ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f})'
        )
        self._publish_goal_marker()

    def enable_callback(self, request, response):
        """Handle enable/disable service calls."""
        self.enabled = request.data
        response.success = True
        response.message = f"Autonomous navigation {'enabled' if self.enabled else 'disabled'}"
        self.get_logger().info(response.message)

        # Debug logging for state
        self.get_logger().debug(f"Enable service called: enabled={self.enabled}")
        self.get_logger().debug(f"  Has pose: {self.current_pose is not None}")
        self.get_logger().debug(f"  Has goal: {self.goal_pose is not None}")
        self.get_logger().debug(f"  Has map: {self.occupancy_grid_msg is not None}")

        if not self.enabled:
            # Stop the drone
            self._publish_zero_velocity()
            self.current_waypoint = None
            self.get_logger().info("Navigation disabled - stopping drone")

        return response

    def planning_loop(self):
        """Periodic path planning."""
        if not self.enabled:
            self.get_logger().debug("Planning loop: Not enabled")
            return

        if not PYASTAR_AVAILABLE:
            self.get_logger().debug("Planning loop: pyastar2d not available")
            return

        if self.occupancy_grid_msg is None:
            self.get_logger().debug("Planning loop: No occupancy grid available")
            return

        if self.current_pose is None:
            self.get_logger().debug("Planning loop: No current pose available")
            return

        if self.goal_pose is None:
            self.get_logger().debug("Planning loop: No goal pose set")
            return

        # Log world coordinates
        start_world = (self.current_pose.pose.position.x, self.current_pose.pose.position.y)
        goal_world = (self.goal_pose.pose.position.x, self.goal_pose.pose.position.y)

        self.get_logger().debug(
            f"Planning loop: start_world=({start_world[0]:.2f}, {start_world[1]:.2f}), "
            f"goal_world=({goal_world[0]:.2f}, {goal_world[1]:.2f})"
        )

        # Convert poses to grid coordinates
        start_grid = self._world_to_grid(start_world[0], start_world[1])
        goal_grid = self._world_to_grid(goal_world[0], goal_world[1])

        # Handle out-of-bounds start position
        if start_grid is None:
            self.get_logger().warning(
                f"Planning loop: Start position ({start_world[0]:.2f}, {start_world[1]:.2f}) "
                f"is OUT OF GRID BOUNDS - clamping to nearest in-bounds position"
            )
            clamped_start = self._clamp_to_grid_bounds(start_world[0], start_world[1])
            if clamped_start is None:
                self.get_logger().error("Failed to clamp start position")
                return
            start_grid = self._world_to_grid(clamped_start[0], clamped_start[1])
            if start_grid is None:
                self.get_logger().error("Clamped start position still out of bounds")
                return

        # Handle out-of-bounds goal position
        if goal_grid is None:
            self.get_logger().warning(
                f"Planning loop: Goal position ({goal_world[0]:.2f}, {goal_world[1]:.2f}) "
                f"is OUT OF GRID BOUNDS - clamping to nearest in-bounds position"
            )
            clamped_goal = self._clamp_to_grid_bounds(goal_world[0], goal_world[1])
            if clamped_goal is None:
                self.get_logger().error("Failed to clamp goal position")
                return
            goal_grid = self._world_to_grid(clamped_goal[0], clamped_goal[1])
            if goal_grid is None:
                self.get_logger().error("Clamped goal position still out of bounds")
                return

        self.get_logger().debug(
            f"Planning loop: Requesting path from grid {start_grid} to {goal_grid}"
        )

        # Convert occupancy grid data to numpy array
        width = self.occupancy_grid_msg.info.width
        height = self.occupancy_grid_msg.info.height
        grid_array = np.array(self.occupancy_grid_msg.data).reshape((height, width))

        # Request path planning
        self.path_planner.request_path(grid_array, start_grid, goal_grid)

        # Publish path visualization
        self._publish_path()

    def control_loop(self):
        """Periodic control loop for navigation."""
        if not self.enabled:
            self.get_logger().debug("Control loop: Not enabled")
            return

        if self.current_pose is None:
            self.get_logger().debug("Control loop: No current pose available")
            return

        # Get current path
        path = self.path_planner.get_current_path()

        if not path:
            self.get_logger().debug("Control loop: No path available")
            self._publish_zero_velocity()
            return

        # Get or update current waypoint
        if self.current_waypoint is None and len(path) > 0:
            self.current_waypoint = path[0]
            self.get_logger().debug(f"Control loop: New waypoint set to {self.current_waypoint}")

        if self.current_waypoint is None:
            self.get_logger().debug("Control loop: No waypoint available")
            self._publish_zero_velocity()
            return

        # Convert waypoint to world coordinates
        waypoint_world = self._grid_to_world(
            self.current_waypoint[0],
            self.current_waypoint[1]
        )

        if waypoint_world is None:
            self._publish_zero_velocity()
            return

        # Calculate movement command
        dx = waypoint_world[0] - self.current_pose.pose.position.x
        dy = waypoint_world[1] - self.current_pose.pose.position.y
        distance = math.sqrt(dx * dx + dy * dy)

        # Check if waypoint reached
        if distance < self.waypoint_tolerance:
            self.get_logger().info(f"Waypoint reached, distance: {distance:.2f}m")
            # Move to next waypoint
            path = self.path_planner.get_current_path()
            if len(path) > 1:
                self.current_waypoint = path[1]
            else:
                self.get_logger().info("Goal reached!")
                self.current_waypoint = None
                self._publish_zero_velocity()
                return

        # Generate velocity command
        if distance > 0:
            # Normalize direction (speed will be controlled by crazyflie_node speed_factor)
            cmd = Twist()
            cmd.linear.x = dx / distance
            cmd.linear.y = dy / distance
            # Add continuous yaw rotation for active environment scanning
            cmd.angular.z = self.scanning_yaw_rate

            self.get_logger().debug(
                f"Publishing cmd_vel: vx={cmd.linear.x:.3f}, vy={cmd.linear.y:.3f}, "
                f"yaw_rate={cmd.angular.z:.3f}, distance to waypoint={distance:.3f}m"
            )
            self.cmd_vel_pub.publish(cmd)

    def _world_to_grid(self, x, y):
        """Convert world coordinates to grid coordinates."""
        if self.occupancy_grid_msg is None:
            self.get_logger().debug("_world_to_grid: No occupancy grid available")
            return None

        info = self.occupancy_grid_msg.info
        grid_x = int((x - info.origin.position.x) / info.resolution)
        grid_y = int((y - info.origin.position.y) / info.resolution)

        self.get_logger().debug(
            f"_world_to_grid: world=({x:.2f}, {y:.2f}) -> grid=({grid_x}, {grid_y}) | "
            f"map_origin=({info.origin.position.x:.2f}, {info.origin.position.y:.2f}), "
            f"resolution={info.resolution:.3f}, map_size=({info.width}x{info.height})"
        )

        # Check bounds
        if (0 <= grid_x < info.width and 0 <= grid_y < info.height):
            self.get_logger().debug(f"_world_to_grid: Position IN BOUNDS")
            return (grid_x, grid_y)

        self.get_logger().debug(
            f"_world_to_grid: Position OUT OF BOUNDS - grid=({grid_x}, {grid_y}) "
            f"not in [0, {info.width}) x [0, {info.height})"
        )
        return None

    def _grid_to_world(self, grid_x, grid_y):
        """Convert grid coordinates to world coordinates."""
        if self.occupancy_grid_msg is None:
            return None

        info = self.occupancy_grid_msg.info
        world_x = grid_x * info.resolution + info.origin.position.x
        world_y = grid_y * info.resolution + info.origin.position.y

        return (world_x, world_y)

    def _clamp_to_grid_bounds(self, x, y):
        """Clamp world coordinates to be within the occupancy grid bounds."""
        if self.occupancy_grid_msg is None:
            return None

        info = self.occupancy_grid_msg.info

        # Calculate grid boundaries in world coordinates
        min_x = info.origin.position.x
        max_x = info.origin.position.x + (info.width * info.resolution)
        min_y = info.origin.position.y
        max_y = info.origin.position.y + (info.height * info.resolution)

        # Clamp to boundaries
        clamped_x = max(min_x, min(x, max_x - info.resolution))
        clamped_y = max(min_y, min(y, max_y - info.resolution))

        if clamped_x != x or clamped_y != y:
            self.get_logger().info(
                f"_clamp_to_grid_bounds: Clamped ({x:.2f}, {y:.2f}) -> ({clamped_x:.2f}, {clamped_y:.2f}) | "
                f"grid bounds: x=[{min_x:.2f}, {max_x:.2f}], y=[{min_y:.2f}, {max_y:.2f}]"
            )

        return (clamped_x, clamped_y)

    def status_loop(self):
        """Periodic status reporting."""
        if not self.enabled:
            return

        # Build status message
        status_parts = []
        status_parts.append("ENABLED")

        if self.current_pose is None:
            status_parts.append("⚠ NO_POSE")
        else:
            status_parts.append(f"pose=({self.current_pose.pose.position.x:.1f},{self.current_pose.pose.position.y:.1f})")

        if self.occupancy_grid_msg is None:
            status_parts.append("⚠ NO_MAP")
        else:
            status_parts.append(f"map={self.occupancy_grid_msg.info.width}x{self.occupancy_grid_msg.info.height}")

        if self.goal_pose is None:
            status_parts.append("⚠ NO_GOAL")
        else:
            status_parts.append(f"goal=({self.goal_pose.pose.position.x:.1f},{self.goal_pose.pose.position.y:.1f})")

        path = self.path_planner.get_current_path()
        if path:
            status_parts.append(f"path={len(path)} waypoints")
        else:
            status_parts.append("⚠ NO_PATH")

        self.get_logger().info(f"Status: {' | '.join(status_parts)}")

    def _publish_zero_velocity(self):
        """Publish zero velocity command."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def _publish_path(self):
        """Publish planned path for visualization."""
        path = self.path_planner.get_current_path()
        if not path or self.occupancy_grid_msg is None:
            return

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.occupancy_grid_msg.header.frame_id

        for waypoint in path:
            world_pos = self._grid_to_world(waypoint[0], waypoint[1])
            if world_pos is not None:
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = world_pos[0]
                pose.pose.position.y = world_pos[1]
                pose.pose.position.z = self.flight_height
                path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def _publish_goal_marker(self):
        """Publish goal marker for visualization."""
        if self.goal_pose is None:
            return

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.goal_pose.header.frame_id
        marker.ns = "goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = self.goal_pose.pose
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.goal_marker_pub.publish(marker)

    def destroy_node(self):
        """Cleanup when node is destroyed."""
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
