#!/usr/bin/env python3
"""
Mission Manager Node for Crazyflie Mission Control

Manages the finite state machine for the entire mission, integrating
box landing logic directly without separate node coordination.

States:
  - IDLE: Startup state
  - TAKEOFF: Using controller takeoff service
  - READY: Hovering, waiting for goal
  - NAVIGATION: Path following to goal
  - BOX_LANDING: Precision landing on detected box
    - GRID_SEARCH: Sequential waypoint grid search
    - NAVIGATE_TO_CENTROID: Move to computed box center
    - WAIT_COMPLETE: Brief hover before completion
  - LAND: Using controller land service
  - WAITING: Post-landing, mission complete

Subscribes to:
  /odom (nav_msgs/Odometry)
  /goal_pose (geometry_msgs/PoseStamped)
  /perception/edge_event (std_msgs/Bool)
  /position_target_reached (std_msgs/Bool)
  /crazyflie/state (std_msgs/Int8)

Service Clients:
  /takeoff (std_srvs/Trigger) - controller takeoff
  /land (std_srvs/Trigger) - controller landing

Publishes:
  /cmd_pos (geometry_msgs/PointStamped)
  /mode_manager/state (std_msgs/String)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Int8
from std_srvs.srv import Trigger, SetBool
from visualization_msgs.msg import Marker, MarkerArray
import math


class MissionManagerNode(Node):
    """Central FSM for mission control with integrated box landing."""

    def __init__(self):
        super().__init__('mission_manager_node')

        # Parameters
        self.declare_parameter('flight_height', 0.5)
        self.declare_parameter('search_span_m', 0.6)
        self.declare_parameter('search_step_m', 0.15)
        self.declare_parameter('min_edge_points', 20)
        self.declare_parameter('goal_tolerance', 0.15)
        self.declare_parameter('edge_detection_delay', 5.0)
        self.declare_parameter('box_center_policy', 'box_fitting')
        self.declare_parameter('box_size', 0.3)

        self.flight_height = self.get_parameter('flight_height').value
        self.search_span = self.get_parameter('search_span_m').value
        self.search_step = self.get_parameter('search_step_m').value
        self.min_edge_points = self.get_parameter('min_edge_points').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.edge_delay = self.get_parameter('edge_detection_delay').value
        self.box_center_policy = self.get_parameter('box_center_policy').value
        self.box_size = self.get_parameter('box_size').value

        # State
        self.state = 'IDLE'
        self.box_landing_phase = None  # GRID_SEARCH, NAVIGATE_TO_CENTROID, WAIT_COMPLETE
        self.current_pose = None
        self.goal_pose = None
        self.navigation_start_time = None
        self.home_positioning_start = None  # Timer for precise home positioning
        self.return_home_after_takeoff = False  # Flag for /refly service

        # Crazyflie controller state (0=FLYING, 1=LANDING, 2=LANDED, 3=TAKING_OFF)
        self.crazyflie_state = None

        # Box landing state
        self.edge_points = []
        self.box_centroid = None
        self.grid_waypoints = []
        self.current_grid_idx = 0
        self.waiting_for_position = False
        self.waypoint_reached_time = None  # Timer for stabilization between waypoints
        self.wait_complete_start = None

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.edge_event_sub = self.create_subscription(
            Bool, '/perception/edge_event', self.edge_callback, 10)
        self.position_reached_sub = self.create_subscription(
            Bool, '/position_target_reached', self.position_reached_callback, 10)
        self.crazyflie_state_sub = self.create_subscription(
            Int8, '/crazyflie/state', self.crazyflie_state_callback, 10)
        self.nav_goal_reached_sub = self.create_subscription(
            Bool, '/nav_node/goal_reached', self.nav_goal_reached_callback, 10)

        # Publishers
        self.state_pub = self.create_publisher(String, '/mode_manager/state', 10)
        self.cmd_pos_pub = self.create_publisher(PointStamped, '/cmd_pos', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.edge_markers_pub = self.create_publisher(MarkerArray, '/perception/edge_points', 10)

        # Service clients
        self.takeoff_client = self.create_client(Trigger, '/takeoff')
        self.land_client = self.create_client(Trigger, '/land')
        self.enable_nav_client = self.create_client(SetBool, '/enable_nav')
        self.clear_map_client = self.create_client(Trigger, '/clear_map')

        # Service servers
        self.refly_srv = self.create_service(Trigger, '/refly', self.refly_srv_handler)

        # Timers
        self.control_timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        self.status_timer = self.create_timer(5.0, self.status_loop)

        # Clear any stale markers from previous runs
        self._clear_edge_markers()

        self.get_logger().info('Mission Manager Node initialized')
        self.get_logger().info(f'Initial state: {self.state}')

    # ----- Callbacks -----
    def crazyflie_state_callback(self, msg: Int8):
        """Handle crazyflie controller state updates."""
        was_none = self.crazyflie_state is None
        self.crazyflie_state = msg.data

        state_names = {-1: 'UNINITIALIZED', 0: 'FLYING', 1: 'LANDING', 2: 'LANDED', 3: 'TAKING_OFF'}
        state_name = state_names.get(msg.data, f'UNKNOWN({msg.data})')

        if was_none:
            self.get_logger().info(f'Received first crazyflie state: {state_name}')

    def odom_callback(self, msg: Odometry):
        """Handle odometry updates from controller."""
        was_none = self.current_pose is None
        # Store as PoseStamped for compatibility with rest of code
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        pose_msg.pose = msg.pose.pose
        self.current_pose = pose_msg
        if was_none:
            self.get_logger().info(
                f'Received first pose: ({msg.pose.pose.position.x:.2f}, '
                f'{msg.pose.pose.position.y:.2f}, {msg.pose.pose.position.z:.2f})')

    def goal_callback(self, msg: PoseStamped):
        self.goal_pose = msg
        self.get_logger().info(
            f'New goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

        # Transition to NAVIGATION if in READY state
        if self.state == 'READY':
            self.get_logger().info('Goal received → STATE CHANGE: READY -> NAVIGATION')
            self._change_state('NAVIGATION')

    def edge_callback(self, msg: Bool):
        """Handle edge detection events."""
        if not msg.data:
            return

        # Only respond to edge events in NAVIGATION state or BOX_LANDING GRID_SEARCH
        if self.state == 'NAVIGATION':
            if self.navigation_start_time is None:
                return

            current_time = self.get_clock().now().nanoseconds / 1e9
            elapsed = current_time - self.navigation_start_time

            if elapsed < self.edge_delay:
                self.get_logger().info(
                    f'Edge detected but delay not met ({elapsed:.1f}s < {self.edge_delay}s)')
                return

            # Edge detected - transition to box landing
            self.get_logger().info('EDGE detected → STATE CHANGE: NAVIGATION -> BOX_LANDING')
            self._change_state('BOX_LANDING')

        elif self.state == 'BOX_LANDING' and self.box_landing_phase == 'GRID_SEARCH':
            # Record edge point during grid search
            if self.current_pose is not None:
                x = self.current_pose.pose.position.x
                y = self.current_pose.pose.position.y
                self.edge_points.append((x, y))
                self.get_logger().info(
                    f'EDGE POINT #{len(self.edge_points)} recorded at ({x:.3f}, {y:.3f})')

                # Publish visualization markers
                self._publish_edge_markers()

                # Check if we have enough points
                if len(self.edge_points) >= self.min_edge_points:
                    self.get_logger().info(
                        f'Collected {len(self.edge_points)} edge points (>= {self.min_edge_points})')
                    self._compute_and_navigate_to_centroid()

    def position_reached_callback(self, msg: Bool):
        """Handle position target reached notifications from controller."""
        if msg.data and self.waiting_for_position:
            self.waiting_for_position = False
            # Record time for waypoint stabilization delay
            if self.box_landing_phase == 'GRID_SEARCH':
                self.waypoint_reached_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().debug('Position target reached')

    def nav_goal_reached_callback(self, msg: Bool):
        """Handle nav_node goal reached notification."""
        if msg.data and self.state == 'NAVIGATION':
            if self.goal_pose is None:
                return

            # Check if current goal is at (0, 0)
            goal_x = self.goal_pose.pose.position.x
            goal_y = self.goal_pose.pose.position.y
            tolerance = 0.1  # 10cm tolerance for considering goal at origin

            if abs(goal_x) < tolerance and abs(goal_y) < tolerance:
                # Reached origin - use precise positioning before landing
                self.get_logger().info('Nav_node reached origin (0,0) → STATE CHANGE: NAVIGATION -> HOME_POSITIONING')
                self._call_enable_nav(False)  # Disable nav_node
                self._change_state('HOME_POSITIONING')
            else:
                # Reached goal - stay in navigation mode and wait
                self.get_logger().info(f'Nav_node reached goal ({goal_x:.2f}, {goal_y:.2f}) - waiting in nav_mode for next command or edge detection')

    # ----- State Management -----
    def _change_state(self, new_state: str):
        """Centralized state change handler."""
        old_state = self.state
        self.state = new_state

        # Publish state change
        state_msg = String()
        state_msg.data = new_state
        self.state_pub.publish(state_msg)

        # Handle state transitions
        if new_state == 'TAKEOFF':
            # Call takeoff service
            self._call_takeoff_service()

        elif new_state == 'NAVIGATION':
            self.navigation_start_time = self.get_clock().now().nanoseconds / 1e9
            # Enable nav_node for autonomous navigation
            self._call_enable_nav(True)

        elif new_state == 'BOX_LANDING':
            # Disable nav_node - we're switching to direct position control for box landing
            self._call_enable_nav(False)

            # Clear markers from previous mission
            self._clear_edge_markers()

            # Initialize box landing
            self.box_landing_phase = 'GRID_SEARCH'
            self.edge_points.clear()
            self.box_centroid = None

            # Generate grid waypoints centered at current position
            if self.current_pose is not None:
                cx = self.current_pose.pose.position.x
                cy = self.current_pose.pose.position.y
                self.edge_points.append((cx, cy))
                self.get_logger().info(
                    f'Box landing started - using current position as EDGE POINT #1: ({cx:.3f}, {cy:.3f})')

                self.grid_waypoints = self._generate_grid_waypoints(cx, cy)
                self.current_grid_idx = 0
                self.waypoint_reached_time = None  # Reset stabilization timer
                self.get_logger().info(
                    f'Generated {len(self.grid_waypoints)} grid waypoints for search')

        elif new_state == 'HOME_POSITIONING':
            # Send precise position command to (0, 0) and start timer
            self._send_position_target(0.0, 0.0, self.flight_height)
            self.home_positioning_start = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info('Starting precise home positioning at (0, 0)')

        elif new_state == 'LAND':
            # Call land service
            self._call_land_service()

        elif new_state == 'IDLE':
            # Reset mission state
            self.goal_pose = None
            self.navigation_start_time = None
            self.home_positioning_start = None

        self.get_logger().info(f'STATE CHANGE: {old_state} -> {new_state}')

    def _call_takeoff_service(self):
        """Call controller takeoff service."""
        if not self.takeoff_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Takeoff service not available')
            return

        request = Trigger.Request()
        future = self.takeoff_client.call_async(request)
        future.add_done_callback(self._takeoff_response_callback)
        self.get_logger().info('Calling takeoff service...')

    def _takeoff_response_callback(self, future):
        """Handle takeoff service response."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Takeoff successful: {response.message}')
                self._change_state('READY')

                # Check if we should auto-publish home goal (from /refly)
                if self.return_home_after_takeoff:
                    self.return_home_after_takeoff = False
                    self.get_logger().info('Publishing return-to-home goal at (0, 0) after refly')
                    home_goal = PoseStamped()
                    home_goal.header.stamp = self.get_clock().now().to_msg()
                    home_goal.header.frame_id = 'world'
                    home_goal.pose.position.x = 0.0
                    home_goal.pose.position.y = 0.0
                    home_goal.pose.position.z = self.flight_height
                    home_goal.pose.orientation.w = 1.0
                    self.goal_pub.publish(home_goal)
            else:
                self.get_logger().error(f'Takeoff failed: {response.message}')
                self._change_state('IDLE')
        except Exception as e:
            self.get_logger().error(f'Takeoff service call failed: {e}')
            self._change_state('IDLE')

    def _call_land_service(self):
        """Call controller land service."""
        if not self.land_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Land service not available')
            return

        request = Trigger.Request()
        future = self.land_client.call_async(request)
        future.add_done_callback(self._land_response_callback)
        self.get_logger().info('Calling land service...')

    def _land_response_callback(self, future):
        """Handle land service response."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Landing successful: {response.message}')
                self._change_state('WAITING')
            else:
                self.get_logger().error(f'Landing failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Land service call failed: {e}')

    def refly_srv_handler(self, request, response):
        """
        Handle /refly service - takeoff and return to (0,0).
        Should be called from WAITING state after landing.
        """
        if self.state != 'WAITING':
            response.success = False
            response.message = f'Cannot refly: not in WAITING state (current state: {self.state})'
            self.get_logger().warn(response.message)
            return response

        self.get_logger().info('Refly requested - clearing map and initiating takeoff')

        # Clear the map before reflying
        self._call_clear_map()

        # Transition to TAKEOFF state
        self._change_state('TAKEOFF')

        # After takeoff completes, the _takeoff_response_callback will transition to READY
        # We need to modify that callback to check if we should auto-publish (0,0) goal
        # Store a flag to indicate we should return home after takeoff
        self.return_home_after_takeoff = True

        response.success = True
        response.message = 'Refly initiated - map cleared, will takeoff and return to (0,0)'
        return response

    def _call_enable_nav(self, enable: bool):
        """Enable or disable nav_node navigation."""
        if not self.enable_nav_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Enable nav service not available')
            return

        request = SetBool.Request()
        request.data = enable
        future = self.enable_nav_client.call_async(request)
        future.add_done_callback(
            lambda f: self._enable_nav_response_callback(f, enable))
        action = "Enabling" if enable else "Disabling"
        self.get_logger().info(f'{action} nav_node...')

    def _enable_nav_response_callback(self, future, enable: bool):
        """Handle enable nav service response."""
        try:
            response = future.result()
            action = "enabled" if enable else "disabled"
            if response.success:
                self.get_logger().info(f'Nav_node {action}: {response.message}')
            else:
                self.get_logger().error(f'Nav_node {action} failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Enable nav service call failed: {e}')

    def _call_clear_map(self):
        """Clear the occupancy grid map."""
        if not self.clear_map_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Clear map service not available')
            return

        request = Trigger.Request()
        future = self.clear_map_client.call_async(request)
        future.add_done_callback(self._clear_map_response_callback)
        self.get_logger().info('Calling clear map service...')

    def _clear_map_response_callback(self, future):
        """Handle clear map service response."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Map cleared: {response.message}')
            else:
                self.get_logger().error(f'Clear map failed: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Clear map service call failed: {e}')

    # ----- Grid Search -----
    def _generate_grid_waypoints(self, center_x: float, center_y: float):
        """
        Generate waypoint list for two perpendicular raster/lawnmower grid patterns.
        First pattern: horizontal sweeps (left-right), moving down between rows.
        Second pattern: vertical sweeps (up-down), moving right between columns.
        Returns list of (x, y) positions.
        """
        half_span = self.search_span / 2.0
        step = self.search_step

        num_cols = int(self.search_span / step) + 1
        num_rows = int(self.search_span / step) + 1

        waypoints = []

        # Start at top-left corner
        start_x = center_x - half_span
        start_y = center_y + half_span
        waypoints.append((start_x, start_y))

        # HORIZONTAL RASTER PATTERN
        for row in range(num_rows):
            y = start_y - row * step
            if row % 2 == 0:
                # Even row: sweep right
                for col in range(1, num_cols + 1):
                    x = start_x + col * step
                    waypoints.append((x, y))
            else:
                # Odd row: sweep left
                for col in range(num_cols - 1, -1, -1):
                    x = start_x + col * step
                    waypoints.append((x, y))

        # Position after horizontal pattern
        last_x = waypoints[-1][0]
        last_y = waypoints[-1][1]

        # Move to bottom-left if needed
        if last_x != start_x:
            waypoints.append((start_x, last_y))

        # VERTICAL RASTER PATTERN
        for col in range(num_cols):
            x = start_x + col * step
            if col % 2 == 0:
                # Even column: sweep up
                for row in range(num_rows - 1, -1, -1):
                    y = start_y - row * step
                    waypoints.append((x, y))
            else:
                # Odd column: sweep down
                for row in range(num_rows):
                    y = start_y - row * step
                    waypoints.append((x, y))

        return waypoints

    def _compute_centroid(self, edge_points):
        """Compute simple centroid (average) from edge points."""
        xs = [p[0] for p in edge_points]
        ys = [p[1] for p in edge_points]
        cx = sum(xs) / len(xs)
        cy = sum(ys) / len(ys)
        return (cx, cy)

    def _fit_box_to_edges(self, edge_points):
        """
        Fit a square box of known size to edge points.

        Uses least-squares optimization to find box center that best fits
        the detected edge points, assuming box dimensions are known (0.3m x 0.3m).

        Args:
            edge_points: List of (x, y) tuples representing detected edges

        Returns:
            (cx, cy): Fitted box center coordinates
        """
        half_size = self.box_size / 2.0  # 0.15m

        # Objective: minimize sum of squared distances from points to nearest box edge
        def distance_to_box(center, point):
            cx, cy = center
            px, py = point
            # Distance to nearest edge of box centered at (cx, cy)
            dx = max(0, abs(px - cx) - half_size)
            dy = max(0, abs(py - cy) - half_size)
            return math.sqrt(dx**2 + dy**2)

        # Start with centroid as initial guess
        x0, y0 = self._compute_centroid(edge_points)

        # Simple grid search for optimization (avoids scipy dependency)
        best_center = (x0, y0)
        best_error = sum(distance_to_box(best_center, p)**2 for p in edge_points)

        # Grid search around initial guess
        search_radius = 0.1  # Search within 10cm
        search_step = 0.01   # 1cm resolution

        for dx in range(-int(search_radius/search_step), int(search_radius/search_step)+1):
            for dy in range(-int(search_radius/search_step), int(search_radius/search_step)+1):
                cx = x0 + dx * search_step
                cy = y0 + dy * search_step
                error = sum(distance_to_box((cx, cy), p)**2 for p in edge_points)
                if error < best_error:
                    best_error = error
                    best_center = (cx, cy)

        return best_center

    def _compute_box_center(self, edge_points):
        """
        Compute box center using selected policy.

        Returns:
            (cx, cy): Box center coordinates
        """
        if self.box_center_policy == 'box_fitting':
            return self._fit_box_to_edges(edge_points)
        elif self.box_center_policy == 'centroid':
            return self._compute_centroid(edge_points)
        else:
            self.get_logger().warn(
                f'Unknown policy: {self.box_center_policy}, defaulting to centroid')
            return self._compute_centroid(edge_points)

    def _compute_and_navigate_to_centroid(self):
        """Compute box center from edge points and navigate to it."""
        if len(self.edge_points) < 3:
            self.get_logger().warn(
                f'Not enough edge points: {len(self.edge_points)} < 3')
            return

        # Use selected policy to compute box center
        cx, cy = self._compute_box_center(self.edge_points)
        self.box_centroid = (cx, cy)

        # Publish updated markers with center
        self._publish_edge_markers()

        self.get_logger().info(
            f'Box center ({self.box_center_policy}): ({cx:.3f}, {cy:.3f}) '
            f'from {len(self.edge_points)} edge points')

        # Transition to navigate to centroid phase
        self.box_landing_phase = 'NAVIGATE_TO_CENTROID'
        self._send_position_target(cx, cy, self.flight_height)
        self.waiting_for_position = True

    def _clear_edge_markers(self):
        """Clear all edge point and box center markers from RViz."""
        marker_array = MarkerArray()

        # Delete all markers in edge_points namespace
        edge_delete = Marker()
        edge_delete.header.frame_id = 'world'
        edge_delete.header.stamp = self.get_clock().now().to_msg()
        edge_delete.ns = 'edge_points'
        edge_delete.id = 0
        edge_delete.action = Marker.DELETEALL
        marker_array.markers.append(edge_delete)

        # Delete all markers in box_center namespace
        center_delete = Marker()
        center_delete.header.frame_id = 'world'
        center_delete.header.stamp = self.get_clock().now().to_msg()
        center_delete.ns = 'box_center'
        center_delete.id = 0
        center_delete.action = Marker.DELETEALL
        marker_array.markers.append(center_delete)

        self.edge_markers_pub.publish(marker_array)
        self.get_logger().debug('Cleared all edge point markers')

    def _publish_edge_markers(self):
        """Publish visualization markers for all detected edge points."""
        marker_array = MarkerArray()

        for i, (x, y) in enumerate(self.edge_points):
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'edge_points'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            # Position
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = self.flight_height
            marker.pose.orientation.w = 1.0

            # Scale (small spheres)
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05

            # Color (red for edge points)
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            # Lifetime (persistent)
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0

            marker_array.markers.append(marker)

        # Add marker for computed box center (if available)
        if self.box_centroid is not None:
            cx, cy = self.box_centroid
            center_marker = Marker()
            center_marker.header.frame_id = 'world'
            center_marker.header.stamp = self.get_clock().now().to_msg()
            center_marker.ns = 'box_center'
            center_marker.id = 1000  # Unique ID
            center_marker.type = Marker.CYLINDER
            center_marker.action = Marker.ADD

            center_marker.pose.position.x = cx
            center_marker.pose.position.y = cy
            center_marker.pose.position.z = self.flight_height
            center_marker.pose.orientation.w = 1.0

            # Scale (larger cylinder to show center)
            center_marker.scale.x = 0.1
            center_marker.scale.y = 0.1
            center_marker.scale.z = 0.02

            # Color (green for center)
            center_marker.color.r = 0.0
            center_marker.color.g = 1.0
            center_marker.color.b = 0.0
            center_marker.color.a = 0.8

            center_marker.lifetime.sec = 0
            center_marker.lifetime.nanosec = 0

            marker_array.markers.append(center_marker)

        self.edge_markers_pub.publish(marker_array)

    def _send_position_target(self, x: float, y: float, z: float):
        """Send position target to controller via cmd_pos."""
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.point.x = x
        msg.point.y = y
        msg.point.z = z
        self.cmd_pos_pub.publish(msg)
        self.get_logger().debug(f'Sent position target: ({x:.3f}, {y:.3f}, {z:.3f})')

    # ----- Control Loop -----
    def control_loop(self):
        """Main control loop - handles state transitions and position commands."""
        # State machine logic
        if self.state == 'IDLE':
            # Auto-takeoff when crazyflie is in LANDED state (2)
            if self.crazyflie_state == 2:  # STATE_LANDED
                self.get_logger().info('Crazyflie is LANDED and ready - initiating takeoff')
                self._change_state('TAKEOFF')
            return

        elif self.state == 'TAKEOFF':
            # Waiting for takeoff service to complete
            return

        elif self.state == 'READY':
            # Hovering, waiting for goal_pose
            # Transition happens in goal_callback when goal is received
            # Need pose data for this state
            if self.current_pose is None:
                return
            return

        elif self.state == 'NAVIGATION':
            # Nav_node handles autonomous navigation - we just monitor for edge detection
            # Edge detection will trigger transition to BOX_LANDING
            # Goal reached will be handled by nav_goal_reached_callback
            return

        elif self.state == 'BOX_LANDING':
            # Need pose data for this state
            if self.current_pose is None:
                return
            if self.box_landing_phase == 'GRID_SEARCH':
                # Send next grid waypoint
                if not self.waiting_for_position and self.current_grid_idx < len(self.grid_waypoints):
                    # Check if we need to wait for stabilization after reaching previous waypoint
                    if self.waypoint_reached_time is not None:
                        elapsed = self.get_clock().now().nanoseconds / 1e9 - self.waypoint_reached_time
                        if elapsed < 0.5:
                            return  # Still waiting for stabilization

                    # Ready to send next waypoint
                    wx, wy = self.grid_waypoints[self.current_grid_idx]
                    self._send_position_target(wx, wy, self.flight_height)
                    self.waiting_for_position = True
                    self.waypoint_reached_time = None  # Reset timer
                    self.current_grid_idx += 1

                elif self.current_grid_idx >= len(self.grid_waypoints):
                    # Grid search complete
                    self.get_logger().info(
                        f'Grid search complete - {len(self.edge_points)} edges detected')

                    if len(self.edge_points) >= 3:
                        self.get_logger().warn(
                            f'{len(self.edge_points)} edges collected (target: {self.min_edge_points}), '
                            f'attempting navigation with available points')
                        self._compute_and_navigate_to_centroid()
                    else:
                        self.get_logger().warn(
                            f'Insufficient edges for navigation: {len(self.edge_points)} < 3 (minimum required)')
                        self._change_state('LAND')

            elif self.box_landing_phase == 'NAVIGATE_TO_CENTROID':
                # Check if reached centroid (trust controller's position_target_reached signal)
                if not self.waiting_for_position:
                    self.get_logger().info('Reached box centroid (controller confirmed arrival)')
                    self.box_landing_phase = 'WAIT_COMPLETE'
                    self.wait_complete_start = self.get_clock().now().nanoseconds / 1e9

            elif self.box_landing_phase == 'WAIT_COMPLETE':
                # Wait 2 seconds to stabilize before transitioning to land
                if self.wait_complete_start is not None:
                    elapsed = self.get_clock().now().nanoseconds / 1e9 - self.wait_complete_start
                    if elapsed >= 2.0:
                        self.get_logger().info('Box landing complete (stabilized for 2s) - initiating landing')
                        self._change_state('LAND')
            return

        elif self.state == 'HOME_POSITIONING':
            # Precise positioning at (0, 0) before landing
            if self.home_positioning_start is not None:
                elapsed = self.get_clock().now().nanoseconds / 1e9 - self.home_positioning_start
                if elapsed >= 0.2:  # Wait 0.2 seconds
                    self.get_logger().info('Home positioning complete (0.2s elapsed) - initiating landing')
                    self._change_state('LAND')
            return

        elif self.state == 'LAND':
            # Waiting for land service to complete
            return

        elif self.state == 'WAITING':
            # Mission complete
            return

    def status_loop(self):
        """Periodic status logging."""
        parts = [f"STATE={self.state}"]

        if self.state == 'BOX_LANDING' and self.box_landing_phase:
            parts.append(f"PHASE={self.box_landing_phase}")

        if self.current_pose is None:
            parts.append("NO_POSE")
        else:
            parts.append(f"pose=({self.current_pose.pose.position.x:.1f},"
                        f"{self.current_pose.pose.position.y:.1f},"
                        f"{self.current_pose.pose.position.z:.1f})")

        if self.goal_pose is None:
            parts.append("NO_GOAL")
        else:
            parts.append(f"goal=({self.goal_pose.pose.position.x:.1f},"
                        f"{self.goal_pose.pose.position.y:.1f})")

        if self.state == 'BOX_LANDING':
            parts.append(f"edges={len(self.edge_points)}")

        self.get_logger().info("Status: " + " | ".join(parts))


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
