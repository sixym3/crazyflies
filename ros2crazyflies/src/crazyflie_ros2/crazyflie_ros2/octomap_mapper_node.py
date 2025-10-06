#!/usr/bin/env python3
"""
OctoMap Mapper Node for Crazyflie

Subscribes to Crazyflie range sensors and pose, builds a 3D OctoMap
for obstacle mapping and path planning.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import Range, PointCloud2, PointField
from octomap_msgs.msg import Octomap
from std_msgs.msg import Header

import numpy as np
import struct
import math
from collections import deque


class OctomapMapperNode(Node):
    """ROS2 node for building OctoMap from Crazyflie sensors."""

    def __init__(self):
        super().__init__('octomap_mapper_node')

        # Declare parameters
        self.declare_parameter('resolution', 0.05)  # 5cm voxel resolution
        self.declare_parameter('max_range', 3.0)    # Max sensor range to trust
        self.declare_parameter('min_range', 0.02)   # Min sensor range to trust
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('robot_frame', 'crazyflie')
        self.declare_parameter('publish_point_cloud', True)
        self.declare_parameter('point_cloud_history', 100)  # Keep last N measurements

        # Get parameters
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.min_range = self.get_parameter('min_range').get_parameter_value().double_value
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.publish_pc = self.get_parameter('publish_point_cloud').get_parameter_value().bool_value
        self.pc_history = self.get_parameter('point_cloud_history').get_parameter_value().integer_value

        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Current pose
        self.current_pose = None
        self.pose_received = False

        # Point cloud history for obstacle tracking
        self.obstacle_points = deque(maxlen=self.pc_history)

        # Sensor directions in robot frame (unit vectors)
        self.sensor_directions = {
            'front': np.array([1.0, 0.0, 0.0]),
            'back': np.array([-1.0, 0.0, 0.0]),
            'left': np.array([0.0, 1.0, 0.0]),
            'right': np.array([0.0, -1.0, 0.0]),
            'up': np.array([0.0, 0.0, 1.0]),
            'down': np.array([0.0, 0.0, -1.0]),
        }

        # Only use horizontal sensors for obstacle mapping (not up/down)
        self.horizontal_sensors = {'front', 'back', 'left', 'right'}

        # Subscribe to pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/crazyflie/pose',
            self.pose_callback,
            10
        )

        # Subscribe to range sensors
        self.range_subs = {}
        for sensor_name in self.sensor_directions.keys():
            self.range_subs[sensor_name] = self.create_subscription(
                Range,
                f'/crazyflie/range/{sensor_name}',
                lambda msg, name=sensor_name: self.range_callback(msg, name),
                sensor_qos
            )

        # Publishers
        if self.publish_pc:
            self.pointcloud_pub = self.create_publisher(
                PointCloud2,
                '/octomap/point_cloud',
                10
            )
            # Timer for publishing aggregated point cloud
            self.pc_timer = self.create_timer(0.5, self.publish_point_cloud)

        self.get_logger().info('OctoMap Mapper Node initialized')
        self.get_logger().info(f'Resolution: {self.resolution}m, Max range: {self.max_range}m')

    def pose_callback(self, msg):
        """Store current pose for transforming sensor data."""
        self.current_pose = msg
        if not self.pose_received:
            self.pose_received = True
            self.get_logger().info('Pose received, ready to map obstacles')

    def range_callback(self, msg, sensor_name):
        """Process range sensor data and add obstacles to map."""
        # Wait until we have pose data
        if not self.pose_received or self.current_pose is None:
            return

        # Skip up/down sensors for obstacle mapping (avoid adding obstacles at drone position)
        if sensor_name not in self.horizontal_sensors:
            return

        # Check if range is valid
        if msg.range < self.min_range or msg.range > self.max_range:
            return

        # Get sensor direction in robot frame
        sensor_dir = self.sensor_directions[sensor_name]

        # Calculate obstacle position in robot frame (not world frame)
        # octomap_server will use TF to transform and determine sensor origin
        obstacle_point_robot = sensor_dir * msg.range

        # Add to point cloud history (in robot frame)
        self.obstacle_points.append(obstacle_point_robot)

    def transform_to_world(self, sensor_dir, distance):
        """Transform sensor reading to world coordinates."""
        if self.current_pose is None:
            return None

        # Get position
        pos = self.current_pose.pose.position
        robot_pos = np.array([pos.x, pos.y, pos.z])

        # Get orientation as quaternion
        quat = self.current_pose.pose.orientation
        q = np.array([quat.w, quat.x, quat.y, quat.z])

        # Convert quaternion to rotation matrix
        R = self.quaternion_to_rotation_matrix(q)

        # Transform sensor direction to world frame
        sensor_dir_world = R @ sensor_dir

        # Calculate obstacle position
        obstacle_pos = robot_pos + sensor_dir_world * distance

        return obstacle_pos

    def quaternion_to_rotation_matrix(self, q):
        """Convert quaternion [w, x, y, z] to 3x3 rotation matrix."""
        w, x, y, z = q

        R = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
        ])

        return R

    def publish_point_cloud(self):
        """Publish aggregated point cloud of obstacles."""
        if len(self.obstacle_points) == 0:
            return

        # Create PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.robot_frame  # Publish in robot frame, not world frame

        # Define point cloud fields (x, y, z)
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # Pack points into binary data
        points = []
        for point in self.obstacle_points:
            points.append(struct.pack('fff', point[0], point[1], point[2]))

        # Create PointCloud2 message
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12  # 3 floats * 4 bytes
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = True
        cloud_msg.data = b''.join(points)

        self.pointcloud_pub.publish(cloud_msg)

    def destroy_node(self):
        """Cleanup when node is destroyed."""
        self.get_logger().info('Shutting down OctoMap Mapper node...')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = OctomapMapperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
