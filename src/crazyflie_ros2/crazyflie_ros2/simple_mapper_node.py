#!/usr/bin/env python3

""" This simple mapper is loosely based on both the bitcraze cflib point cloud example
 https://github.com/bitcraze/crazyflie-lib-python/blob/master/examples/multiranger/multiranger_pointcloud.py
 and the webots epuck simple mapper example:
 https://github.com/cyberbotics/webots_ros2

 Originally from https://github.com/knmcguire/crazyflie_ros2_experimental/
 """

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

import math
import numpy as np
from bresenham import bresenham

GLOBAL_SIZE_X = 20.0
GLOBAL_SIZE_Y = 20.0
MAP_RES = 0.1


class SimpleMapperMultiranger(Node):
    def __init__(self):
        super().__init__('simple_mapper_multiranger')
        self.declare_parameter('robot_prefix', '/crazyflie')
        robot_prefix = self.get_parameter('robot_prefix').value

        self.odom_subscriber = self.create_subscription(
            Odometry, robot_prefix + '/odom', self.odom_subscribe_callback, 10)
        self.ranges_subscriber = self.create_subscription(
            LaserScan, robot_prefix + '/scan', self.scan_subscribe_callback, 10)
        self.position = [0.0, 0.0, 0.0]
        self.angles = [0.0, 0.0, 0.0]
        self.ranges = [0.0, 0.0, 0.0, 0.0]
        self.range_max = 3.5

        self.tfbr = StaticTransformBroadcaster(self)
        t_map = TransformStamped()
        t_map.header.stamp = self.get_clock().now().to_msg()
        t_map.header.frame_id = 'map'
        t_map.child_frame_id = robot_prefix + '/odom'
        t_map.transform.translation.x = 0.0
        t_map.transform.translation.y = 0.0
        t_map.transform.translation.z = 0.0
        self.tfbr.sendTransform(t_map)

        self.position_update = False

        self.map = [-1] * int(GLOBAL_SIZE_X / MAP_RES) * \
            int(GLOBAL_SIZE_Y / MAP_RES)
        self.map_publisher = self.create_publisher(OccupancyGrid, robot_prefix + '/map',
                                                   qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST,))

        self.get_logger().info(f"Simple mapper set for crazyflie " + robot_prefix +
                               f" using the odom and scan topic")
        self.publish_initial_map()

    def publish_initial_map(self):
        """Publish an initial map for testing."""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = MAP_RES
        msg.info.width = int(GLOBAL_SIZE_X / MAP_RES)
        msg.info.height = int(GLOBAL_SIZE_Y / MAP_RES)
        msg.info.origin.position.x = -GLOBAL_SIZE_X / 2.0
        msg.info.origin.position.y = -GLOBAL_SIZE_Y / 2.0
        msg.data = self.map
        self.map_publisher.publish(msg)
        self.get_logger().info("Published initial empty map")

    def euler_from_quaternion(self, x, y, z, w):
        """Convert quaternion to euler angles (roll, pitch, yaw)."""
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def odom_subscribe_callback(self, msg):
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.position[2] = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        roll, pitch, yaw = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.angles[0] = roll
        self.angles[1] = pitch
        self.angles[2] = yaw
        self.position_update = True

    def scan_subscribe_callback(self, msg):
        self.ranges = msg.ranges
        self.range_max = msg.range_max
        
        self.get_logger().debug(f"Received scan with {len(self.ranges)} ranges: {self.ranges}")
        
        if self.position_update is False:
            self.get_logger().warning("No position update yet, skipping scan")
            return
        
        self.get_logger().debug(f"Robot position: ({self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f})")
        
        data = self.rotate_and_create_points()
        self.get_logger().debug(f"Created {len(data)} obstacle points")
        
        points_x = []
        points_y = []
        
        # Calculate map dimensions
        map_width = int(GLOBAL_SIZE_X / MAP_RES)
        map_height = int(GLOBAL_SIZE_Y / MAP_RES)
        
        # Calculate current position in map coordinates
        position_x_map = int((self.position[0] - (-GLOBAL_SIZE_X / 2.0)) / MAP_RES)
        position_y_map = int((self.position[1] - (-GLOBAL_SIZE_Y / 2.0)) / MAP_RES)
        
        self.get_logger().debug(f"Robot in map coords: ({position_x_map}, {position_y_map}), map size: {map_width}x{map_height}")
        
        # Check if robot position is within map bounds
        if not (0 <= position_x_map < map_width and 0 <= position_y_map < map_height):
            self.get_logger().error(f"Robot outside map bounds! Skipping scan.")
            return
        
        obstacle_count = 0
        for i in range(len(data)):
            point_x = int((data[i][0] - (-GLOBAL_SIZE_X / 2.0)) / MAP_RES)
            point_y = int((data[i][1] - (-GLOBAL_SIZE_Y / 2.0)) / MAP_RES)
            
            # Check if point is within map bounds
            if not (0 <= point_x < map_width and 0 <= point_y < map_height):
                self.get_logger().debug(f"Point ({point_x}, {point_y}) outside bounds, skipping")
                continue
            
            points_x.append(point_x)
            points_y.append(point_y)
            obstacle_count += 1
            
            # Draw line from robot position to detected point
            for line_x, line_y in bresenham(position_x_map, position_y_map, point_x, point_y):
                if 0 <= line_x < map_width and 0 <= line_y < map_height:
                    self.map[line_y * map_width + line_x] = 0
            
            # Mark detected obstacle point
            self.map[point_y * map_width + point_x] = 100
        
        self.get_logger().debug(f"Processed {obstacle_count} obstacles, updating map")
        
        # Create and publish map message
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = MAP_RES
        msg.info.width = map_width
        msg.info.height = map_height
        msg.info.origin.position.x = -GLOBAL_SIZE_X / 2.0
        msg.info.origin.position.y = -GLOBAL_SIZE_Y / 2.0
        msg.data = self.map
        
        self.get_logger().info(f"Publishing map: {map_width}x{map_height}, {len(self.map)} cells")
        self.map_publisher.publish(msg)

    def rotate_and_create_points(self):
        data = []
        o = self.position
        roll = self.angles[0]
        pitch = self.angles[1]
        yaw = self.angles[2]
        r_back = self.ranges[0]
        r_right = self.ranges[1]
        r_front = self.ranges[2]
        r_left = self.ranges[3]
        
        self.get_logger().debug(f"Ranges - back: {r_back}, right: {r_right}, front: {r_front}, left: {r_left}, max: {self.range_max}")


        if (r_left < self.range_max and r_left != 0.0 and math.isinf(r_left) == False):
            left = [o[0], o[1] + r_left, o[2]]
            data.append(self.rot(roll, pitch, yaw, o, left))

        if (r_right < self.range_max and r_right != 0.0 and math.isinf(r_right) == False):
            right = [o[0], o[1] - r_right, o[2]]
            data.append(self.rot(roll, pitch, yaw, o, right))

        if (r_front < self.range_max and r_front != 0.0 and math.isinf(r_front) == False):
            front = [o[0] + r_front, o[1], o[2]]
            data.append(self.rot(roll, pitch, yaw, o, front))

        if (r_back < self.range_max and r_back != 0.0 and math.isinf(r_back) == False):
            back = [o[0] - r_back, o[1], o[2]]
            data.append(self.rot(roll, pitch, yaw, o, back))

        return data

    def rot(self, roll, pitch, yaw, origin, point):
        cosr = math.cos((roll))
        cosp = math.cos((pitch))
        cosy = math.cos((yaw))

        sinr = math.sin((roll))
        sinp = math.sin((pitch))
        siny = math.sin((yaw))

        roty = np.array([[cosy, -siny, 0],
                        [siny, cosy, 0],
                        [0, 0,    1]])

        rotp = np.array([[cosp, 0, sinp],
                        [0, 1, 0],
                        [-sinp, 0, cosp]])

        rotr = np.array([[1, 0,   0],
                        [0, cosr, -sinr],
                        [0, sinr,  cosr]])

        rotFirst = np.dot(rotr, rotp)

        rot = np.array(np.dot(rotFirst, roty))

        tmp = np.subtract(point, origin)
        tmp2 = np.dot(rot, tmp)
        return np.add(tmp2, origin)


def main(args=None):

    rclpy.init(args=args)
    simple_mapper_multiranger = SimpleMapperMultiranger()
    rclpy.spin(simple_mapper_multiranger)
    rclpy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
