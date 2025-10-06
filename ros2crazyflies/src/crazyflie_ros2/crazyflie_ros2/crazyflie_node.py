#!/usr/bin/env python3
"""
ROS2 Crazyflie Bringup Node

Handles communication with Crazyflie drone using cflib.
Publishes sensor data and position estimates.
Subscribes to cmd_vel for velocity control.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from sensor_msgs.msg import Range
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

import math
import threading


class CrazyflieNode(Node):
    """ROS2 node for Crazyflie control and telemetry."""

    def __init__(self):
        super().__init__('crazyflie_node')

        # Declare parameters
        self.declare_parameter('uri', 'radio://0/80/2M/E7E7E7E7E2')
        self.declare_parameter('hover_height', 0.6)
        self.declare_parameter('speed_factor', 0.6)
        self.declare_parameter('frame_id', 'crazyflie')
        self.declare_parameter('logging_only', False)

        # Get parameters
        self.uri = self.get_parameter('uri').get_parameter_value().string_value
        self.hover_height = self.get_parameter('hover_height').get_parameter_value().double_value
        self.speed_factor = self.get_parameter('speed_factor').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.logging_only = self.get_parameter('logging_only').get_parameter_value().bool_value

        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers - Pose
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/crazyflie/pose',
            10
        )

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Publishers - Range sensors
        self.range_pubs = {
            'front': self.create_publisher(Range, '/crazyflie/range/front', sensor_qos),
            'back': self.create_publisher(Range, '/crazyflie/range/back', sensor_qos),
            'left': self.create_publisher(Range, '/crazyflie/range/left', sensor_qos),
            'right': self.create_publisher(Range, '/crazyflie/range/right', sensor_qos),
            'up': self.create_publisher(Range, '/crazyflie/range/up', sensor_qos),
            'down': self.create_publisher(Range, '/crazyflie/range/down', sensor_qos),
        }

        # Subscriber - cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Control state
        self.cmd_vel = Twist()
        self.cmd_lock = threading.Lock()

        # Initialize Crazyflie
        cflib.crtp.init_drivers()
        self.cf = Crazyflie(ro_cache=None, rw_cache='cache')

        # Setup callbacks
        self.cf.connected.add_callback(self._connected)
        self.cf.disconnected.add_callback(self._disconnected)
        self.cf.connection_failed.add_callback(self._connection_failed)

        # Control timer (10Hz = 100ms)
        self.control_timer = self.create_timer(0.1, self.send_control_command)

        # Connect to Crazyflie
        mode = "LOGGING ONLY (no flight commands)" if self.logging_only else "FULL CONTROL"
        self.get_logger().info(f'Mode: {mode}')
        self.get_logger().info(f'Connecting to Crazyflie at {self.uri}...')
        self.cf.open_link(self.uri)

    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands."""
        with self.cmd_lock:
            self.cmd_vel = msg

    def send_control_command(self):
        """Send velocity commands to Crazyflie at 10Hz."""
        if self.logging_only:
            # In logging-only mode, don't send any flight commands
            return

        with self.cmd_lock:
            # Map Twist message to Crazyflie hover command
            # Twist.linear.x -> forward/backward (vx)
            # Twist.linear.y -> left/right (vy)
            # Twist.linear.z -> up/down (vz, affects hover_height)
            # Twist.angular.z -> yaw rate

            vx = self.cmd_vel.linear.x * self.speed_factor
            vy = self.cmd_vel.linear.y * self.speed_factor
            yaw_rate = math.degrees(self.cmd_vel.angular.z) * self.speed_factor

            # Handle height adjustment
            if self.cmd_vel.linear.z != 0:
                self.hover_height += self.cmd_vel.linear.z * 0.01  # Small increments
                self.hover_height = max(0.1, min(1.0, self.hover_height))  # Clamp to safe range

            self.cf.commander.send_hover_setpoint(vx, vy, yaw_rate, self.hover_height)

    def _connected(self, uri):
        """Callback when Crazyflie connects."""
        self.get_logger().info(f'Connected to {uri}')

        # Setup position logging
        self._setup_position_logging()

        # Setup sensor logging
        self._setup_sensor_logging()

    def _disconnected(self, uri):
        """Callback when Crazyflie disconnects."""
        self.get_logger().info(f'Disconnected from {uri}')

    def _connection_failed(self, uri, msg):
        """Callback when connection fails."""
        self.get_logger().error(f'Connection to {uri} failed: {msg}')

    def _setup_position_logging(self):
        """Setup position and orientation logging."""
        log_conf = LogConfig(name='Position', period_in_ms=100)
        log_conf.add_variable('stateEstimate.x', 'float')
        log_conf.add_variable('stateEstimate.y', 'float')
        log_conf.add_variable('stateEstimate.z', 'float')
        log_conf.add_variable('stabilizer.roll', 'float')
        log_conf.add_variable('stabilizer.pitch', 'float')
        log_conf.add_variable('stabilizer.yaw', 'float')

        try:
            self.cf.log.add_config(log_conf)
            log_conf.data_received_cb.add_callback(self._position_callback)
            log_conf.start()
            self.get_logger().info('Position logging started')
        except KeyError as e:
            self.get_logger().error(f'Could not start position logging: {e}')
        except AttributeError:
            self.get_logger().error('Could not add position log config')

    def _setup_sensor_logging(self):
        """Setup range sensor logging."""
        log_conf = LogConfig(name='Range', period_in_ms=100)
        log_conf.add_variable('range.front', 'uint16_t')
        log_conf.add_variable('range.back', 'uint16_t')
        log_conf.add_variable('range.left', 'uint16_t')
        log_conf.add_variable('range.right', 'uint16_t')
        log_conf.add_variable('range.up', 'uint16_t')
        log_conf.add_variable('range.zrange', 'uint16_t')

        try:
            self.cf.log.add_config(log_conf)
            log_conf.data_received_cb.add_callback(self._sensor_callback)
            log_conf.start()
            self.get_logger().info('Range sensor logging started')
        except KeyError as e:
            self.get_logger().error(f'Could not start sensor logging: {e}')
        except AttributeError:
            self.get_logger().error('Could not add sensor log config')

    def _position_callback(self, timestamp, data, logconf):
        """Publish position and orientation data."""
        # Create timestamp
        stamp = self.get_clock().now().to_msg()

        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = 'world'

        # Position
        pose_msg.pose.position.x = data['stateEstimate.x']
        pose_msg.pose.position.y = data['stateEstimate.y']
        pose_msg.pose.position.z = data['stateEstimate.z']

        # Orientation (convert roll, pitch, yaw to quaternion)
        roll = math.radians(data['stabilizer.roll'])
        pitch = math.radians(data['stabilizer.pitch'])
        yaw = math.radians(data['stabilizer.yaw'])

        # Convert Euler angles to quaternion
        qx, qy, qz, qw = self._euler_to_quaternion(roll, pitch, yaw)
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.pose_pub.publish(pose_msg)

        # Broadcast TF transform
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'world'
        t.child_frame_id = self.frame_id

        t.transform.translation.x = pose_msg.pose.position.x
        t.transform.translation.y = pose_msg.pose.position.y
        t.transform.translation.z = pose_msg.pose.position.z

        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

    def _sensor_callback(self, timestamp, data, logconf):
        """Publish range sensor data."""
        stamp = self.get_clock().now().to_msg()

        # Publish each range sensor
        sensors = {
            'front': data['range.front'],
            'back': data['range.back'],
            'left': data['range.left'],
            'right': data['range.right'],
            'up': data['range.up'],
            'down': data['range.zrange']
        }

        for sensor_name, distance_mm in sensors.items():
            range_msg = Range()
            range_msg.header = Header()
            range_msg.header.stamp = stamp
            range_msg.header.frame_id = f'{self.frame_id}/{sensor_name}'

            range_msg.radiation_type = Range.INFRARED
            range_msg.field_of_view = 0.436  # ~25 degrees in radians
            range_msg.min_range = 0.01  # 1cm
            range_msg.max_range = 4.0   # 4m
            range_msg.range = distance_mm / 1000.0  # Convert mm to meters

            self.range_pubs[sensor_name].publish(range_msg)

    def _euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
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

    def destroy_node(self):
        """Cleanup when node is destroyed."""
        self.get_logger().info('Shutting down Crazyflie node...')
        if self.cf is not None:
            self.cf.close_link()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = CrazyflieNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
