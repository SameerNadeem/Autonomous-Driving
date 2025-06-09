#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
from rclpy.qos import qos_profile_sensor_data  # Quality of Service settings for real-time data
import time
from ament_index_python.packages import get_package_prefix

from ros2_numpy import from_ackermann, np_to_pose


class OdometryNode(Node):
    def __init__(self, pkg_dir, every_nth = 20):
        super().__init__('odometry')
        self.get_logger().info("Initializing Ackermann odometry node")

        self.output_dir = pkg_dir + '/results'

        # Create a service called 'reset_odometry' using the Empty service type.
        self.srv = self.create_service(Empty, 'reset_odometry', self.reset_odometry)

        # Only publish every nth waypoint
        self.every_nth = every_nth
    
        # Define Quality of Service (QoS) for communication
        qos_profile = qos_profile_sensor_data  # Suitable for sensor data
        qos_profile.depth = 1  # Keep only the latest message

        # Create a subscriber for the calibrated IMU data
        self.subscription = self.create_subscription(AckermannDriveStamped, '/rc/ackermann_cmd', self.ack_callback, qos_profile)
        
        # Create a publisher for the resulting odometry path
        self.publisher = self.create_publisher(PoseStamped, '/position', qos_profile)

        self.wheelbase = 0.36 # meters

        self.last_time = time.time()

        self.init_position()

    def init_position(self):
        self.speed = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def reset_odometry(self, request, response):
        self.init_position()
        self.get_logger().info("Vehicle odometry reset to default")
        return response

    def ack_callback(self, msg: AckermannDriveStamped):
        """
        Callback that processes Ackermann data to update odometry.
        """
        # Convert Ackermann message to numpy format
        # timestamp_unix is the image timestamp in seconds (Unix time)
        speed, steering_angle, timestamp_unix = from_ackermann(msg)

        timestamp = msg.header.stamp

        dt = timestamp_unix - self.last_time
        dx = dt * speed
        self.x += dx
        self.get_logger().info(f"{self.x:.2f}")
        
        # Publish waypoint as Pose message
        pose_msg = np_to_pose(np.array([self.x, .0]), 0.0, timestamp=timestamp)
        self.publisher.publish(pose_msg)

        self.last_time = timestamp_unix

def main(args=None):
    pkg_dir = get_package_prefix('odometry').replace('install', 'src') # /mxck2_ws/install/odometry → /mxck2_ws/src/odometry

    rclpy.init(args=args)
    node = OdometryNode(pkg_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Odometry node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
