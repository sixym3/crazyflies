#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int8
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range, LaserScan
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

import time
import threading
import math
import numpy as np
from abc import ABC, abstractmethod

# State machine constants (same as controller.py)
STATE_UNINITIALIZED = -1
STATE_FLYING = 0
STATE_LANDING = 1
STATE_LANDED = 2
STATE_TAKING_OFF = 3

# Control mode constants
CONTROL_MODE_VELOCITY = 0
CONTROL_MODE_POSITION = 1


# ============================================================================
# Obstacle System - Procedural Environment Generation
# ============================================================================

class Obstacle(ABC):
    """Abstract base class for obstacles in the simulated environment"""

    @abstractmethod
    def ray_intersect(self, origin, direction):
        """
        Calculate intersection distance from origin along direction ray.

        Args:
            origin: np.array([x, y, z]) - ray origin point
            direction: np.array([dx, dy, dz]) - normalized ray direction

        Returns:
            float: distance to intersection, or float('inf') if no intersection
        """
        pass

    @abstractmethod
    def check_collision(self, position, radius=0.0):
        """
        Check if a sphere at position with given radius collides with obstacle.

        Args:
            position: np.array([x, y, z]) - center of sphere
            radius: float - radius of sphere (0 for point collision)

        Returns:
            bool: True if collision detected
        """
        pass


class WallObstacle(Obstacle):
    """Infinite vertical wall defined by a point and normal vector"""

    def __init__(self, point, normal, name="Wall"):
        """
        Create a vertical wall obstacle.

        Args:
            point: np.array([x, y]) - a point on the wall (2D)
            normal: np.array([nx, ny]) - normal vector (2D, will be normalized)
            name: str - name for debugging
        """
        self.point = np.array(point, dtype=float)
        self.normal = np.array(normal, dtype=float)
        self.normal = self.normal / np.linalg.norm(self.normal)  # Normalize
        self.name = name

    def ray_intersect(self, origin, direction):
        """Ray-plane intersection for vertical wall (ignoring z)"""
        # Work in 2D (x, y plane)
        origin_2d = origin[:2]
        direction_2d = direction[:2]

        # Check if ray is parallel to wall
        denom = np.dot(direction_2d, self.normal)
        if abs(denom) < 1e-6:
            return float('inf')

        # Calculate intersection distance
        t = np.dot(self.normal, self.point - origin_2d) / denom

        # Only return positive distances (forward along ray)
        if t < 0:
            return float('inf')

        return t

    def check_collision(self, position, radius=0.0):
        """Check if position is within radius of wall"""
        pos_2d = position[:2]
        distance = abs(np.dot(self.normal, pos_2d - self.point))
        return distance < radius

    def __repr__(self):
        return f"{self.name}(point={self.point}, normal={self.normal})"


class CircleObstacle(Obstacle):
    """Circular obstacle (infinite cylinder in z direction)"""

    def __init__(self, center, radius, name="Circle"):
        """
        Create a circular obstacle.

        Args:
            center: np.array([x, y]) - center of circle (2D)
            radius: float - radius of circle
            name: str - name for debugging
        """
        self.center = np.array(center, dtype=float)
        self.radius = float(radius)
        self.name = name

    def ray_intersect(self, origin, direction):
        """Ray-circle intersection (in 2D, treating as cylinder)"""
        # Work in 2D (x, y plane)
        origin_2d = origin[:2]
        direction_2d = direction[:2]

        # Normalize direction in 2D
        dir_len = np.linalg.norm(direction_2d)
        if dir_len < 1e-6:
            return float('inf')
        direction_2d = direction_2d / dir_len

        # Vector from origin to circle center
        to_center = self.center - origin_2d

        # Project to_center onto ray direction
        t_closest = np.dot(to_center, direction_2d)

        # Find closest point on ray to circle center
        closest_point = origin_2d + t_closest * direction_2d

        # Distance from circle center to closest point
        distance_to_ray = np.linalg.norm(closest_point - self.center)

        # Check if ray intersects circle
        if distance_to_ray > self.radius:
            return float('inf')

        # Calculate intersection distance using Pythagorean theorem
        # Distance along ray from closest point to intersection
        offset = math.sqrt(self.radius**2 - distance_to_ray**2)

        # First intersection point
        t_intersect = t_closest - offset

        # Only return positive distances
        if t_intersect < 0:
            # Check if we're inside the circle
            if np.linalg.norm(origin_2d - self.center) < self.radius:
                # Return exit intersection
                t_intersect = t_closest + offset
                if t_intersect < 0:
                    return float('inf')
            else:
                return float('inf')

        return t_intersect

    def check_collision(self, position, radius=0.0):
        """Check if position is within combined radius of circle"""
        pos_2d = position[:2]
        distance = np.linalg.norm(pos_2d - self.center)
        return distance < (self.radius + radius)

    def __repr__(self):
        return f"{self.name}(center={self.center}, radius={self.radius})"


class FloorCeilingObstacle(Obstacle):
    """Horizontal plane (floor or ceiling)"""

    def __init__(self, z_height, is_ceiling=False, name="Floor"):
        """
        Create a horizontal plane obstacle.

        Args:
            z_height: float - height of the plane
            is_ceiling: bool - True if ceiling (blocks upward rays), False if floor
            name: str - name for debugging
        """
        self.z = float(z_height)
        self.is_ceiling = is_ceiling
        self.name = name

    def ray_intersect(self, origin, direction):
        """Ray-plane intersection for horizontal plane"""
        # Check if ray is parallel to plane
        if abs(direction[2]) < 1e-6:
            return float('inf')

        # Calculate intersection distance
        t = (self.z - origin[2]) / direction[2]

        # Only return positive distances in correct direction
        if t < 0:
            return float('inf')

        # Floor only blocks downward rays, ceiling only blocks upward rays
        if self.is_ceiling and direction[2] < 0:
            return float('inf')
        if not self.is_ceiling and direction[2] > 0:
            return float('inf')

        return t

    def check_collision(self, position, radius=0.0):
        """Check if position is within radius of plane"""
        if self.is_ceiling:
            return (position[2] + radius) > self.z
        else:
            return (position[2] - radius) < self.z

    def __repr__(self):
        return f"{self.name}(z={self.z})"


class SimCrazyflieCommunicator(Node):
    """Simulated Crazyflie controller - drop-in replacement for controller.py"""

    def __init__(self):
        super().__init__('sim_crazyflie_communicator')

        # Declare and get parameters (same as controller.py)
        self.declare_parameter('height', 0.5)
        self.declare_parameter('address', 'radio://0/80/2M/E7E7E7E7E2')  # Not used in sim
        self.declare_parameter('linear_speed_factor', 0.5)
        self.declare_parameter('angular_speed_factor', 90.0)
        self.declare_parameter('control_rate_hz', 30.0)
        self.declare_parameter('position_tolerance', 0.05)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('min_range', 0.02)
        self.declare_parameter('max_range', 3.5)
        self.declare_parameter('field_of_view', 0.436)
        self.declare_parameter('range_sensor_offset', 0.15)  # meters

        # Additional simulation parameters
        self.declare_parameter('ground_level', 0.0)  # Floor height
        self.declare_parameter('ceiling_height', 3.0)  # Ceiling height
        self.declare_parameter('max_acceleration', 2.0)  # m/s^2
        self.declare_parameter('max_yaw_acceleration', 3.0)  # rad/s^2
        self.declare_parameter('position_noise_std', 0.01)  # meters
        self.declare_parameter('orientation_noise_std', 0.02)  # radians
        self.declare_parameter('range_noise_std', 0.02)  # meters
        self.declare_parameter('enable_noise', True)

        # Get parameters
        self.height = self.get_parameter('height').value
        self.linear_speed_factor = self.get_parameter('linear_speed_factor').value
        self.angular_speed_factor = self.get_parameter('angular_speed_factor').value
        self.control_rate_hz = self.get_parameter('control_rate_hz').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.fov = float(self.get_parameter('field_of_view').value)
        self.range_offset = float(self.get_parameter('range_sensor_offset').value)

        # Simulation parameters
        self.ground_level = self.get_parameter('ground_level').value
        self.ceiling_height = self.get_parameter('ceiling_height').value
        self.max_accel = self.get_parameter('max_acceleration').value
        self.max_yaw_accel = self.get_parameter('max_yaw_acceleration').value
        self.pos_noise_std = self.get_parameter('position_noise_std').value
        self.orient_noise_std = self.get_parameter('orientation_noise_std').value
        self.range_noise_std = self.get_parameter('range_noise_std').value
        self.enable_noise = self.get_parameter('enable_noise').value

        # Initialize obstacle list - procedurally generated environment
        self.obstacles = self._create_test_environment()

        # Physics state
        self.sim_lock = threading.Lock()
        self.sim_pos = np.array([0.0, 0.0, 0.0])  # x, y, z in meters
        self.sim_vel = np.array([0.0, 0.0, 0.0])  # vx, vy, vz in m/s
        self.sim_yaw = 0.0  # radians
        self.sim_yaw_rate = 0.0  # rad/s
        self.sim_roll = 0.0  # radians
        self.sim_pitch = 0.0  # radians

        # Landing pose offset (for Kalman filter reset simulation)
        self.landing_pose_offset = np.array([0.0, 0.0, 0.0])  # x, y, z offset

        # Detection box for edge event simulation (0.3m x 0.3m at (0, 3))
        self.detection_box_center = np.array([0.0, 3.0])  # x, y in meters
        self.detection_box_half_size = 0.15  # Half of 0.3m = 0.15m
        self.last_detection_pos = None  # Track previous position for edge crossing

        # Command buffer for one-shot command execution
        self.cmd_buffer = Twist()
        self.cmd_lock = threading.Lock()

        # Position control state
        self.control_mode = CONTROL_MODE_VELOCITY
        self.position_target = None
        self.position_lock = threading.Lock()
        self.position_reached = False

        # Current odometry (for position feedback)
        self.current_odom = None
        self.odom_lock = threading.Lock()

        # State machine
        self.state = STATE_LANDED  # Start in landed state (sim is always "connected")
        self.state_lock = threading.Lock()

        # Create publishers
        self.setpoint_pub = self.create_publisher(Twist, 'hover_setpoint', 1)
        self.position_reached_pub = self.create_publisher(Bool, 'position_target_reached', 1)
        self.state_pub = self.create_publisher(Int8, '/crazyflie/state', 1)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.range_pubs = {
            'front': self.create_publisher(Range, 'range/front', 10),
            'back':  self.create_publisher(Range, 'range/back', 10),
            'left':  self.create_publisher(Range, 'range/left', 10),
            'right': self.create_publisher(Range, 'range/right', 10),
            'up':    self.create_publisher(Range, 'range/up', 10),
            'down':  self.create_publisher(Range, 'range/down', 10),
        }
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publisher for simulated edge detection
        self.edge_event_pub = self.create_publisher(Bool, '/perception/edge_event', 10)

        # Publisher for landing box visualization
        self.landing_box_marker_pub = self.create_publisher(Marker, '/sim/landing_box', 10)

        # Subscribers
        self.command_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 1)
        self.position_sub = self.create_subscription(PointStamped, "cmd_pos", self.cmd_pos_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, "odom", self.odom_callback, 1)

        # Create timer for continuous control loop
        timer_period = 1.0 / self.control_rate_hz
        self.control_timer = self.create_timer(timer_period, self.control_timer_callback)

        # Create timer for periodic state broadcasting (10Hz)
        state_timer_period = 0.1
        self.state_timer = self.create_timer(state_timer_period, self.state_timer_callback)

        # Create timer for sensor publishing (10Hz)
        sensor_timer_period = 0.1
        self.sensor_timer = self.create_timer(sensor_timer_period, self.sensor_timer_callback)

        # Create timer for landing box marker visualization (1Hz)
        marker_timer_period = 1.0
        self.marker_timer = self.create_timer(marker_timer_period, self._publish_landing_box_marker)

        # Create landing and takeoff services
        self.landing_srv = self.create_service(Trigger, 'land', self.land_srv_handler)
        self.takeoff_srv = self.create_service(Trigger, 'takeoff', self.takeoff_srv_handler)
        self.reset_kalman_srv = self.create_service(Trigger, 'reset_kalman', self.reset_kalman_srv_handler)

        # Publish initial state
        self.publish_state()
        self.get_logger().info('Simulation controller initialized - state: LANDED')
        self.get_logger().info(f'Procedural environment: {len(self.obstacles)} obstacles')
        for obs in self.obstacles:
            self.get_logger().info(f'  - {obs}')
        self.get_logger().info(f'Noise enabled: {self.enable_noise}')

    def _create_test_environment(self):
        """
        Create default test environment with 3 walls and 1 circular obstacle.

        Returns:
            list[Obstacle]: List of obstacles in the environment
        """
        obstacles = []

        # Floor and ceiling
        obstacles.append(FloorCeilingObstacle(
            z_height=self.ground_level,
            is_ceiling=False,
            name="Floor"
        ))
        obstacles.append(FloorCeilingObstacle(
            z_height=self.ceiling_height,
            is_ceiling=True,
            name="Ceiling"
        ))

        # Three walls forming an open area (missing one wall for entry/exit)
        # North wall (blocks movement in +Y direction)
        obstacles.append(WallObstacle(
            point=[0.0, 5.0],
            normal=[0.0, -1.0],  # Normal points inward (toward -Y)
            name="North Wall"
        ))

        # East wall (blocks movement in +X direction)
        obstacles.append(WallObstacle(
            point=[5.0, 0.0],
            normal=[-1.0, 0.0],  # Normal points inward (toward -X)
            name="East Wall"
        ))

        # South wall (blocks movement in -Y direction)
        obstacles.append(WallObstacle(
            point=[0.0, -5.0],
            normal=[0.0, 1.0],  # Normal points inward (toward +Y)
            name="South Wall"
        ))

        # West side is OPEN (no wall) - allows entry/exit

        # Circular obstacle in the middle of the room
        obstacles.append(CircleObstacle(
            center=[2.0, 2.0],
            radius=0.5,
            name="Central Pillar"
        ))

        return obstacles

    def _check_box_edge_crossing(self, current_pos):
        """
        Check if drone crossed any edge of the detection box and publish edge event.

        Detection box edges:
        - North edge: y = 3.15 (box_center_y + half_size)
        - South edge: y = 2.85 (box_center_y - half_size)
        - East edge:  x = 0.15 (box_center_x + half_size)
        - West edge:  x = -0.15 (box_center_x - half_size)

        Args:
            current_pos: np.array([x, y, z]) - current drone position
        """
        if self.last_detection_pos is None:
            self.last_detection_pos = current_pos[:2].copy()
            return

        # Get previous and current positions (2D)
        prev_pos = self.last_detection_pos
        curr_pos = current_pos[:2]

        # Box boundaries
        box_x_min = self.detection_box_center[0] - self.detection_box_half_size
        box_x_max = self.detection_box_center[0] + self.detection_box_half_size
        box_y_min = self.detection_box_center[1] - self.detection_box_half_size
        box_y_max = self.detection_box_center[1] + self.detection_box_half_size

        edge_crossed = False

        # Check if line segment (prev_pos -> curr_pos) crosses any edge
        # North edge (horizontal line at y_max)
        if (prev_pos[1] < box_y_max <= curr_pos[1]) or (curr_pos[1] < box_y_max <= prev_pos[1]):
            # Check if x is within box x range
            if box_x_min <= curr_pos[0] <= box_x_max or box_x_min <= prev_pos[0] <= box_x_max:
                edge_crossed = True
                self.get_logger().info(f'Edge event: Crossed NORTH edge at y={box_y_max:.2f}')

        # South edge (horizontal line at y_min)
        if (prev_pos[1] < box_y_min <= curr_pos[1]) or (curr_pos[1] < box_y_min <= prev_pos[1]):
            if box_x_min <= curr_pos[0] <= box_x_max or box_x_min <= prev_pos[0] <= box_x_max:
                edge_crossed = True
                self.get_logger().info(f'Edge event: Crossed SOUTH edge at y={box_y_min:.2f}')

        # East edge (vertical line at x_max)
        if (prev_pos[0] < box_x_max <= curr_pos[0]) or (curr_pos[0] < box_x_max <= prev_pos[0]):
            if box_y_min <= curr_pos[1] <= box_y_max or box_y_min <= prev_pos[1] <= box_y_max:
                edge_crossed = True
                self.get_logger().info(f'Edge event: Crossed EAST edge at x={box_x_max:.2f}')

        # West edge (vertical line at x_min)
        if (prev_pos[0] < box_x_min <= curr_pos[0]) or (curr_pos[0] < box_x_min <= prev_pos[0]):
            if box_y_min <= curr_pos[1] <= box_y_max or box_y_min <= prev_pos[1] <= box_y_max:
                edge_crossed = True
                self.get_logger().info(f'Edge event: Crossed WEST edge at x={box_x_min:.2f}')

        # Publish edge event if any edge was crossed
        if edge_crossed:
            msg = Bool()
            msg.data = True
            self.edge_event_pub.publish(msg)

        # Update previous position
        self.last_detection_pos = curr_pos.copy()

    def _publish_landing_box_marker(self):
        """
        Publish visualization marker for the landing box.

        Box dimensions: 0.3m x 0.3m x 0.1m (width x depth x height)
        Box position: (0.0, 3.0) centered at ground level
        """
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'landing_box'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        # Position (center of box, height/2 above ground)
        marker.pose.position.x = float(self.detection_box_center[0])
        marker.pose.position.y = float(self.detection_box_center[1])
        marker.pose.position.z = 0.05  # 0.1m height / 2 = 0.05m center
        marker.pose.orientation.w = 1.0

        # Scale (box dimensions)
        marker.scale.x = 0.3  # width
        marker.scale.y = 0.3  # depth
        marker.scale.z = 0.1  # height

        # Color (semi-transparent cyan/blue)
        marker.color.r = 0.0
        marker.color.g = 0.7
        marker.color.b = 1.0
        marker.color.a = 0.6  # semi-transparent

        # Lifetime (persistent)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        self.landing_box_marker_pub.publish(marker)

    def cmd_vel_callback(self, data):
        """Store incoming cmd_vel for one-shot execution"""
        with self.cmd_lock:
            self.cmd_buffer = data
        with self.position_lock:
            self.control_mode = CONTROL_MODE_VELOCITY
            self.position_target = None
            self.position_reached = False

    def cmd_pos_callback(self, data):
        """Store incoming position target"""
        with self.position_lock:
            self.position_target = data
            self.control_mode = CONTROL_MODE_POSITION
            self.position_reached = False

    def odom_callback(self, data):
        """Store current odometry for position feedback"""
        with self.odom_lock:
            self.current_odom = data

    def publish_state(self):
        """Publish current state"""
        with self.state_lock:
            current_state = self.state

        state_msg = Int8()
        state_msg.data = current_state
        self.state_pub.publish(state_msg)

    def land_srv_handler(self, request, response):
        """Landing service: gradual descent simulation"""
        with self.state_lock:
            self.state = STATE_LANDING

        self.publish_state()
        self.get_logger().info("Landing initiated...")

        # Record current position before landing (for Kalman reset offset)
        with self.sim_lock:
            self.landing_pose_offset = self.sim_pos.copy()
            self.landing_pose_offset[2] = 0.0  # Don't offset Z (height is controlled)
            self.get_logger().info(f"Recorded landing pose offset: ({self.landing_pose_offset[0]:.3f}, {self.landing_pose_offset[1]:.3f})")

        # Simulate gradual descent
        with self.sim_lock:
            self.sim_vel = np.array([0.0, 0.0, -0.2])  # Descend at 0.2 m/s

        # Wait for descent (simulated)
        time.sleep(1.0)

        # Touch ground
        with self.sim_lock:
            self.sim_pos[2] = 0.0
            self.sim_vel = np.array([0.0, 0.0, 0.0])

        with self.state_lock:
            self.state = STATE_LANDED

        self.publish_state()
        self.get_logger().info("Landing complete")
        response.success = True
        response.message = "Landed successfully"
        return response

    def takeoff_srv_handler(self, request, response):
        """Takeoff service: gradual ascent simulation"""
        with self.state_lock:
            if self.state != STATE_LANDED:
                response.success = False
                response.message = f"Cannot takeoff: not in LANDED state (current state: {self.state})"
                return response
            self.state = STATE_TAKING_OFF

        self.publish_state()
        self.get_logger().info("Takeoff initiated...")

        # Simulate gradual ascent
        with self.sim_lock:
            self.sim_vel = np.array([0.0, 0.0, 0.5])  # Ascend at 0.5 m/s

        # Wait for ascent to target height
        time.sleep(self.height / 0.5)

        with self.sim_lock:
            # Reset position to origin (simulates Kalman filter reset on takeoff)
            self.sim_pos = np.array([0.0, 0.0, self.height])
            self.sim_vel = np.array([0.0, 0.0, 0.0])
            self.get_logger().info(f"Kalman filter reset - internal pos: (0, 0, {self.height:.2f}), offset: ({self.landing_pose_offset[0]:.3f}, {self.landing_pose_offset[1]:.3f})")

        with self.state_lock:
            self.state = STATE_FLYING

        self.publish_state()
        self.get_logger().info("Takeoff complete")
        response.success = True
        response.message = "Takeoff successful"
        return response

    def reset_kalman_srv_handler(self, request, response):
        """Reset Kalman filter (in sim, just reset position to origin)"""
        self.get_logger().info("Resetting simulated Kalman filter...")

        with self.sim_lock:
            self.sim_pos = np.array([0.0, 0.0, self.sim_pos[2]])  # Keep Z, reset X,Y
            self.sim_vel = np.array([0.0, 0.0, 0.0])
            self.sim_yaw = 0.0
            self.sim_yaw_rate = 0.0

        self.get_logger().info("Simulated Kalman filter reset complete")
        response.success = True
        response.message = "Kalman filter reset successfully (simulated)"
        return response

    def process_and_send_velocity(self, vx, vy, angular_z):
        """Process velocity commands and update simulation state"""
        # Scale linear velocity only if magnitude exceeds speed_factor
        linear_magnitude = math.sqrt(vx**2 + vy**2)
        if linear_magnitude > self.linear_speed_factor:
            vx_scaled = (vx / linear_magnitude) * self.linear_speed_factor
            vy_scaled = (vy / linear_magnitude) * self.linear_speed_factor
        else:
            vx_scaled = vx
            vy_scaled = vy

        # Convert angular velocity to radians and cap
        angular_rad = angular_z  # Assume input is already in rad/s
        if abs(angular_rad) > math.radians(self.angular_speed_factor):
            angular_rad = math.copysign(math.radians(self.angular_speed_factor), angular_rad)

        # Publish setpoint (for visualization/debugging)
        setpoint_msg = Twist()
        setpoint_msg.linear.x = vx_scaled
        setpoint_msg.linear.y = vy_scaled
        setpoint_msg.linear.z = self.height
        setpoint_msg.angular.z = math.degrees(angular_rad)
        self.setpoint_pub.publish(setpoint_msg)

        # Update simulation physics
        self.update_physics(vx_scaled, vy_scaled, angular_rad)

    def update_physics(self, vx_body, vy_body, yaw_rate_cmd):
        """Update simulated physics with smoothed acceleration"""
        dt = 1.0 / self.control_rate_hz

        with self.sim_lock:
            # Transform body frame velocity commands to world frame
            cos_yaw = math.cos(self.sim_yaw)
            sin_yaw = math.sin(self.sim_yaw)
            vx_world_target = cos_yaw * vx_body - sin_yaw * vy_body
            vy_world_target = sin_yaw * vx_body + cos_yaw * vy_body

            # Apply acceleration limits (smoothed response)
            target_vel = np.array([vx_world_target, vy_world_target, 0.0])
            vel_error = target_vel - self.sim_vel
            max_vel_change = self.max_accel * dt

            if np.linalg.norm(vel_error[:2]) > max_vel_change:
                vel_error[:2] = vel_error[:2] / np.linalg.norm(vel_error[:2]) * max_vel_change

            self.sim_vel += vel_error

            # Apply yaw acceleration limits
            yaw_rate_error = yaw_rate_cmd - self.sim_yaw_rate
            max_yaw_rate_change = self.max_yaw_accel * dt
            if abs(yaw_rate_error) > max_yaw_rate_change:
                yaw_rate_error = math.copysign(max_yaw_rate_change, yaw_rate_error)

            self.sim_yaw_rate += yaw_rate_error

            # Integrate velocity to position
            self.sim_pos += self.sim_vel * dt

            # Integrate yaw rate to yaw
            self.sim_yaw += self.sim_yaw_rate * dt

            # Normalize yaw to [-pi, pi]
            self.sim_yaw = math.atan2(math.sin(self.sim_yaw), math.cos(self.sim_yaw))

            # Simulate slight roll/pitch from velocity (for realism)
            self.sim_pitch = -vx_body * 0.1  # Small pitch angle from forward velocity
            self.sim_roll = vy_body * 0.1   # Small roll angle from lateral velocity

            # Check for collisions with obstacles (drone safety radius ~0.1m)
            drone_radius = 0.1
            for obstacle in self.obstacles:
                if obstacle.check_collision(self.sim_pos, drone_radius):
                    # Collision detected - stop motion and back away slightly
                    self.sim_vel *= -0.5  # Reverse and slow down
                    self.sim_pos += self.sim_vel * dt * 2  # Back away
                    break

    def control_timer_callback(self):
        """Continuous control loop"""
        with self.state_lock:
            current_state = self.state

        if current_state == STATE_UNINITIALIZED:
            return
        elif current_state == STATE_LANDED:
            return
        elif current_state in [STATE_LANDING, STATE_TAKING_OFF]:
            return
        elif current_state == STATE_FLYING:
            with self.position_lock:
                mode = self.control_mode
                target_pos = self.position_target

            if mode == CONTROL_MODE_VELOCITY:
                with self.cmd_lock:
                    cmd = self.cmd_buffer
                    self.cmd_buffer = Twist()

                vx = cmd.linear.x
                vy = cmd.linear.y
                angular = cmd.angular.z

                self.process_and_send_velocity(vx, vy, angular)

            elif mode == CONTROL_MODE_POSITION:
                if target_pos is None:
                    self.process_and_send_velocity(0.0, 0.0, 0.0)
                    return

                with self.odom_lock:
                    current_odom = self.current_odom

                if current_odom is None:
                    self.process_and_send_velocity(0.0, 0.0, 0.0)
                    return

                # Calculate position error
                current_x = current_odom.pose.pose.position.x
                current_y = current_odom.pose.pose.position.y
                target_x = target_pos.point.x
                target_y = target_pos.point.y

                error_x = target_x - current_x
                error_y = target_y - current_y
                distance = math.sqrt(error_x**2 + error_y**2)

                # Extract current yaw
                q = current_odom.pose.pose.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y**2 + q.z**2))

                if distance > self.position_tolerance:
                    # Far from target: generate velocity commands at max speed
                    direction_x = error_x / distance
                    direction_y = error_y / distance

                    vel_world_x = direction_x * self.linear_speed_factor
                    vel_world_y = direction_y * self.linear_speed_factor

                    # Transform to body frame
                    cos_yaw = math.cos(yaw)
                    sin_yaw = math.sin(yaw)
                    vel_body_x = cos_yaw * vel_world_x + sin_yaw * vel_world_y
                    vel_body_y = -sin_yaw * vel_world_x + cos_yaw * vel_world_y

                    self.process_and_send_velocity(vel_body_x, vel_body_y, 0.0)

                    if self.position_reached:
                        msg = Bool()
                        msg.data = False
                        self.position_reached_pub.publish(msg)
                        self.position_reached = False
                else:
                    # Close to target: use proportional controller for precise position hold
                    # This mimics the behavior of send_position_setpoint in real controller
                    # P-gain of 2.0 means full speed at position_tolerance distance
                    p_gain = 2.0
                    vel_world_x = error_x * p_gain
                    vel_world_y = error_y * p_gain

                    # Clamp to max speed
                    speed = math.sqrt(vel_world_x**2 + vel_world_y**2)
                    if speed > self.linear_speed_factor:
                        scale = self.linear_speed_factor / speed
                        vel_world_x *= scale
                        vel_world_y *= scale

                    # Transform to body frame
                    cos_yaw = math.cos(yaw)
                    sin_yaw = math.sin(yaw)
                    vel_body_x = cos_yaw * vel_world_x + sin_yaw * vel_world_y
                    vel_body_y = -sin_yaw * vel_world_x + cos_yaw * vel_world_y

                    self.process_and_send_velocity(vel_body_x, vel_body_y, 0.0)

                    if not self.position_reached:
                        msg = Bool()
                        msg.data = True
                        self.position_reached_pub.publish(msg)
                        self.position_reached = True
                        self.get_logger().info(f"Position target reached: ({target_x:.2f}, {target_y:.2f})")

    def state_timer_callback(self):
        """Periodic state broadcast"""
        self.publish_state()

    def sensor_timer_callback(self):
        """Publish simulated sensor data (odometry, range, TF)"""
        stamp = self.get_clock().now().to_msg()

        # Get current simulated state with noise
        with self.sim_lock:
            pos = self.sim_pos.copy()
            yaw = self.sim_yaw
            roll = self.sim_roll
            pitch = self.sim_pitch
            pose_offset = self.landing_pose_offset.copy()

        # Apply landing pose offset (simulates Kalman filter reset while at landing position)
        pos += pose_offset

        # Add noise if enabled
        if self.enable_noise:
            pos += np.random.normal(0, self.pos_noise_std, 3)
            yaw += np.random.normal(0, self.orient_noise_std)
            roll += np.random.normal(0, self.orient_noise_std * 0.5)
            pitch += np.random.normal(0, self.orient_noise_std * 0.5)

        # Check for box edge crossings (triggers edge events)
        self._check_box_edge_crossing(pos)

        # Convert to quaternion
        qx, qy, qz, qw = self._euler_to_quaternion(roll, pitch, yaw)

        # Publish odometry
        odom = Odometry()
        odom.header = Header(stamp=stamp, frame_id=self.odom_frame)
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = pos[0]
        odom.pose.pose.position.y = pos[1]
        odom.pose.pose.position.z = pos[2]
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.odom_pub.publish(odom)

        # Publish TF
        t = TransformStamped()
        t.header = Header(stamp=stamp, frame_id=self.odom_frame)
        t.child_frame_id = self.base_frame
        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[1]
        t.transform.translation.z = pos[2]
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

        # Publish range sensors (simulated)
        self.publish_range_sensors(stamp, pos, yaw)

    def publish_range_sensors(self, stamp, pos, yaw):
        """Simulate range sensors using ray-casting through obstacle list"""
        # Sensor directions in body frame (assuming standard multiranger orientation)
        # Front: +X body frame, Back: -X, Left: +Y, Right: -Y, Up: +Z, Down: -Z
        sensor_dirs_body = {
            'front': np.array([1.0, 0.0, 0.0]),
            'back':  np.array([-1.0, 0.0, 0.0]),
            'left':  np.array([0.0, 1.0, 0.0]),
            'right': np.array([0.0, -1.0, 0.0]),
            'up':    np.array([0.0, 0.0, 1.0]),
            'down':  np.array([0.0, 0.0, -1.0]),
        }

        ranges = {}
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        for direction, dir_body in sensor_dirs_body.items():
            # Transform direction to world frame
            if direction in ['up', 'down']:
                # Vertical sensors are not affected by yaw
                dir_world = dir_body
            else:
                # Rotate horizontal directions by yaw
                dir_world = np.array([
                    cos_yaw * dir_body[0] - sin_yaw * dir_body[1],
                    sin_yaw * dir_body[0] + cos_yaw * dir_body[1],
                    0.0
                ])

            # Ray-cast through all obstacles to find closest intersection
            min_dist = self.max_range
            for obstacle in self.obstacles:
                dist = obstacle.ray_intersect(pos, dir_world)
                if dist < min_dist:
                    min_dist = dist

            # Apply sensor offset to horizontal sensors (matches crazyflie_node.py)
            # Accounts for physical sensor mounting position (~0.15m from drone center)
            if direction in ['front', 'back', 'left', 'right']:
                min_dist += self.range_offset

            # Add noise if enabled
            if self.enable_noise and min_dist < self.max_range:
                min_dist += np.random.normal(0, self.range_noise_std)

            # Clamp to sensor range
            dist = max(self.min_range, min(self.max_range, min_dist))
            ranges[direction] = dist

        # Publish individual Range messages
        for direction, dist in ranges.items():
            if direction not in self.range_pubs:
                continue
            r = Range()
            r.header = Header(stamp=stamp, frame_id=f'{self.base_frame}/{direction}')
            r.radiation_type = Range.INFRARED
            r.field_of_view = self.fov
            r.min_range = self.min_range
            r.max_range = self.max_range
            r.range = dist
            self.range_pubs[direction].publish(r)

        # Publish LaserScan (4 horizontal beams: back, right, front, left)
        scan = LaserScan()
        scan.header = Header(stamp=stamp, frame_id=self.base_frame)
        scan.angle_min = 0.0
        scan.angle_increment = math.pi / 2.0
        scan.angle_max = scan.angle_min + 3.0 * scan.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = self.min_range
        scan.range_max = self.max_range
        scan.ranges = [
            ranges['back'],
            ranges['right'],
            ranges['front'],
            ranges['left'],
        ]
        self.scan_pub.publish(scan)

    def _euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw

    def dispose(self):
        """Cleanup (nothing to do for simulation)"""
        self.get_logger().info("Shutting down simulation controller")


def main(args=None):
    rclpy.init(args=args)
    time.sleep(0.5)

    handle = SimCrazyflieCommunicator()

    try:
        rclpy.spin(handle)
    except KeyboardInterrupt:
        pass
    finally:
        handle.dispose()
        handle.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
