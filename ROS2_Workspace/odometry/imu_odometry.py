#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path
from std_srvs.srv import Empty
from rclpy.qos import qos_profile_sensor_data  # Quality of Service settings for real-time data
import time
import matplotlib.pyplot as plt
from ament_index_python.packages import get_package_prefix

from ros2_numpy import imu_to_np


class OdometryNode(Node):
    def __init__(self, pkg_dir, every_nth = 20):
        super().__init__('odometry')
        self.get_logger().info("Initializing IMU odometry node")

        self.output_dir = pkg_dir + '/results'

        # Create a service called 'reset_odometry' using the Empty service type.
        self.srv = self.create_service(Empty, 'reset_odometry', self.reset_odometry)

        # Only publish every nth waypoint
        self.every_nth = every_nth
    
        # Define Quality of Service (QoS) for communication
        qos_profile = qos_profile_sensor_data  # Suitable for sensor data
        qos_profile.depth = 1  # Keep only the latest message

        # Create a subscriber for the calibrated IMU data
        # Use /camera/imu for RealSense IMU
        self.subscription = self.create_subscription(Imu, '/imu', self.imu_callback, qos_profile)
        
        # Create a publisher for the resulting odometry path
        self.imu_pub = self.create_publisher(Path, 'path', 10)

        self.last_time = time.time()

        # This correction applies only to the microcontroller IMU.
        # We selected a gyro range of ±500°/s but mistakenly used the sensitivity for ±250°/s (131 instead of 65.5).
        # This underestimates the angular velocity by a factor of 2, so we multiply the result by 2 to correct it.
        # For the RealSense IMU, no correction is needed — use yaw_gain = 1.0.
        self.yaw_gain = 2.0

        self.init_position()

    def init_position(self):
        self.path = []
        self.speed = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

    def reset_odometry(self, request, response):
        self.init_position()
        self.get_logger().info("Vehicle odometry reset to default")
        return response

    def imu_callback(self, msg: Imu):
        """
        Callback that processes IMU data to update odometry.
        """
        # Convert IMU message to numpy format
        # timestamp_unix is the image timestamp in seconds (Unix time)
        data, timestamp_unix = imu_to_np(msg) # [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z]

        # Coordinate transformation for microcontroller IMU (accel_y → accel_x, gyro_z → yaw_rate)
        # For RealSense IMU (accel_z → accel_x, gyro_y → yaw_rate)
        accel_x = data[1]
        yaw_rate = data[-1] * self.yaw_gain

        self.get_logger().info(f"Got {accel_x=} and {yaw_rate=}")

        # Filter data if needed (e.g., clipping, bias correction, or noise reduction)
        # accel_x = ...

        # Calculate time difference since last callback
        # dt = 

        # Update speed and orientation
        # self.speed += 
        # self.theta +=

        # Update position
        self.x += 1.0
        self.y += 1.0

        self.path.append([self.x, self.y])

        self.last_time = timestamp_unix


    def plot_path(self):
        # Convert path to numpy array for easier slicing
        path_array = np.array(self.path)  # shape (N, 2)

        # Plot
        plt.figure()
        plt.plot(path_array[:, 0], path_array[:, 1], marker='o', linewidth=2)
        plt.xlabel("X position")
        plt.ylabel("Y position")
        plt.title("IMU Odomentry Vehicle Path")
        plt.grid(True)
        plt.axis("equal")  # Keep aspect ratio

        # Save to file
        plt.savefig(self.output_dir + "/odometry_path.png")
        plt.close()  # Close the figure to free memory

    def cleanup(self):

        # Plot result
        self.plot_path()

        self.get_logger().info("Shutting down odometry node, cleaning up resources.")
        # Add any additional cleanup logic here, like saving logs or closing files


def main(args=None):
    pkg_dir = get_package_prefix('odometry').replace('install', 'src') # /mxck2_ws/install/odometry → /mxck2_ws/src/odometry

    rclpy.init(args=args)
    node = OdometryNode(pkg_dir)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Odometry node interrupted by user.")
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
