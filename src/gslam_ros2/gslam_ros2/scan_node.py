#!/usr/bin/env python3
"""
Scan Node for Autonomous Navigation

Provides a service to rotate the drone for environmental scanning.
Continuously publishes velocity commands to /cmd_vel during rotation.

Services:
  /scan_surround (std_srvs/Trigger) - Rotate specified angle at specified speed

Publishes:
  /cmd_vel (geometry_msgs/Twist) - Velocity commands during rotation
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
import threading
import time
import math


class ScanNode(Node):
    def __init__(self):
        super().__init__('scan_node')

        # Parameters
        self.declare_parameter('angular_speed', 90.0)  # degrees/sec
        self.declare_parameter('rotation_angle', 90.0)  # degrees
        self.declare_parameter('publish_rate', 50.0)  # Hz

        self.angular_speed = self.get_parameter('angular_speed').value
        self.rotation_angle = self.get_parameter('rotation_angle').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Service
        self.scan_service = self.create_service(
            Trigger, '/scan_surround', self.scan_surround_callback)

        # State
        self.is_scanning = False
        self.scan_lock = threading.Lock()

        self.get_logger().info(f'Scan Node initialized')
        self.get_logger().info(f'Angular speed: {self.angular_speed} deg/s')
        self.get_logger().info(f'Rotation angle: {self.rotation_angle} deg')
        self.get_logger().info(f'Publish rate: {self.publish_rate} Hz')

    def scan_surround_callback(self, request, response):
        """Handle scan surround service request."""
        with self.scan_lock:
            if self.is_scanning:
                response.success = False
                response.message = "Scan already in progress"
                return response

            self.is_scanning = True

        self.get_logger().info(f'Starting scan: rotating {self.rotation_angle} degrees...')

        # Calculate rotation duration
        rotation_time = self.rotation_angle / self.angular_speed

        # Start publishing thread
        scan_thread = threading.Thread(
            target=self._publish_rotation_commands,
            args=(rotation_time,),
            daemon=True
        )
        scan_thread.start()
        scan_thread.join()  # Wait for completion

        with self.scan_lock:
            self.is_scanning = False

        self.get_logger().info('Scan complete')
        response.success = True
        response.message = f'{self.rotation_angle} degree rotation complete'
        return response

    def _publish_rotation_commands(self, duration: float):
        """Publish rotation commands for specified duration."""
        sleep_time = 1.0 / self.publish_rate
        angular_velocity_rad = math.radians(self.angular_speed)

        # Calculate number of iterations
        num_iterations = int(duration * self.publish_rate)

        # Create rotation command
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_velocity_rad

        # Publish rotation commands
        for i in range(num_iterations):
            self.cmd_vel_pub.publish(twist)
            time.sleep(sleep_time)

        # Send stop commands (publish multiple times to ensure it takes effect)
        twist.angular.z = 0.0
        for _ in range(5):
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)


def main(args=None):
    rclpy.init(args=args)
    node = ScanNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
