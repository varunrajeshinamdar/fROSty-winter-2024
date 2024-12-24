#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_exploration')

        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for LiDAR scan data
        self.scan_sub = self.create_subscription(LaserScan, '/lidar', self.lidar_callback, 10)

        # Initialize control variables
        self.safe_distance = 0.5  # Minimum safe distance from obstacles
        self.twist = Twist()  # Velocity command

        self.get_logger().info("Obstacle Avoidance Node Initialized")

    def lidar_callback(self, scan_data):
        # Get the minimum distance readings from LiDAR data
        left_side = min(scan_data.ranges[0:45])  # Left side (90° to 135°)
        right_side = min(scan_data.ranges[315:360])  # Right side (225° to 270°)
        front = min(scan_data.ranges[90:270])  # Front (90° to 270°)

        if front < self.safe_distance and left_side < self.safe_distance and right_side < self.safe_distance:
            # If all directions are blocked, move backward slightly
            self.get_logger().info("All directions blocked. Moving backward!")
            self.twist.linear.x = ____1____  # Fill blank 1: Linear velocity for backward motion
            self.twist.angular.z = 0.0
        elif front < self.safe_distance:
            # If front is blocked, check the left and right
            if left_side > right_side:
                self.get_logger().info("Obstacle in front. Turning left.")
                self.twist.angular.z = ____2____  # Fill blank 2: Angular velocity for left turn
            else:
                self.get_logger().info("Obstacle in front. Turning right.")
                self.twist.angular.z = ____3____  # Fill blank 3: Angular velocity for right turn
            self.twist.linear.x = 0.0  # Stop moving forward
        else:
            # Path is clear, move forward
            self.twist.linear.x = ____4____  # Fill blank 4: Linear velocity for forward motion
            self.twist.angular.z = 0.0

        # Publish the velocity command
        self.cmd_vel_pub.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    node = ____5____  # Fill blank 5: Instance of the ObstacleAvoidanceNode class
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
