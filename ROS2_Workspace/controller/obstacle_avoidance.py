
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import math
from ros2_numpy import scan_to_np, to_ackermann

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        
        # QoS profile for real-time data
        qos_profile = qos_profile_sensor_data
        qos_profile.depth = 1
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile)
        self.cmd_sub = self.create_subscription(
            AckermannDriveStamped, '/autonomous/ackermann_cmd', self.cmd_callback, qos_profile)
        
        # Publisher for modified commands
        self.cmd_pub = self.create_publisher(
            AckermannDriveStamped, '/safe/ackermann_cmd', qos_profile)
        
        # Publisher for obstacle warnings
        self.obstacle_pub = self.create_publisher(
            PoseStamped, '/obstacle_warning', qos_profile)
        
        # Parameters for obstacle avoidance
        self.safe_distance = 1.5  # meters
        self.critical_distance = 0.8  # meters  
        self.avoidance_gain = 1.2  # steering correction strength
        self.speed_reduction = 0.7  # speed factor when avoiding obstacles
        
        # Vehicle parameters
        self.vehicle_width = 0.3  # meters
        
        # Current command storage
        self.current_speed = 0.0
        self.current_steering = 0.0
        
        # LiDAR data storage
        self.obstacles = []
        
        self.get_logger().info("🚧 Obstacle Avoidance Node Started! 🚧")

    def scan_callback(self, msg):
        """Process LiDAR data to detect obstacles"""
        try:
            # Convert scan to numpy array
            xyi, timestamp = scan_to_np(msg)
            if xyi.size == 0:
                return
                
            x, y, intensity = xyi[:, 0], xyi[:, 1], xyi[:, 2]
            
            # Filter for obstacles in front of vehicle (forward facing arc)
            forward_mask = (x > 0) & (x < 4.0)  # 4 meters ahead
            side_mask = (np.abs(y) < 2.0)  # 2 meters on each side
            relevant_mask = forward_mask & side_mask
            
            if not np.any(relevant_mask):
                self.obstacles = []
                return
                
            # Get relevant obstacles
            obs_x = x[relevant_mask]
            obs_y = y[relevant_mask]
            distances = np.sqrt(obs_x**2 + obs_y**2)
            
            # Group nearby points into obstacles
            self.obstacles = self.cluster_obstacles(obs_x, obs_y, distances)
            
            # Publish obstacle warnings for visualization
            self.publish_obstacle_warnings()
            
        except Exception as e:
            self.get_logger().error(f"Error in scan_callback: {e}")

    def cluster_obstacles(self, x, y, distances, cluster_radius=0.3):
        """Group nearby LiDAR points into distinct obstacles"""
        obstacles = []
        points = list(zip(x, y, distances))
        
        while points:
            # Start new cluster with first point
            seed = points.pop(0)
            cluster = [seed]
            
            # Find all points within cluster_radius
            i = 0
            while i < len(points):
                point = points[i]
                # Check distance to any point in current cluster
                if any(np.sqrt((point[0] - cp[0])**2 + (point[1] - cp[1])**2) < cluster_radius 
                       for cp in cluster):
                    cluster.append(points.pop(i))
                else:
                    i += 1
            
            # Calculate obstacle center and size
            cluster_x = [p[0] for p in cluster]
            cluster_y = [p[1] for p in cluster]
            cluster_dist = [p[2] for p in cluster]
            
            obs_center_x = np.mean(cluster_x)
            obs_center_y = np.mean(cluster_y)
            obs_distance = np.min(cluster_dist)
            obs_width = max(cluster_y) - min(cluster_y) if len(cluster) > 1 else 0.1
            
            obstacles.append({
                'x': obs_center_x,
                'y': obs_center_y,
                'distance': obs_distance,
                'width': obs_width,
                'threat_level': self.calculate_threat_level(obs_center_x, obs_center_y, obs_distance)
            })
        
        return obstacles

    def calculate_threat_level(self, x, y, distance):
        """Calculate how threatening an obstacle is (0-1, 1 = very dangerous)"""
        # Distance factor (closer = more dangerous)
        dist_factor = max(0, 1 - (distance / self.safe_distance))
        
        # Path factor (directly ahead = more dangerous)
        path_factor = max(0, 1 - (abs(y) / 1.0))  # 1 meter path width
        
        # Forward bias (obstacles ahead more dangerous than behind)
        forward_factor = max(0, x / 2.0) if x > 0 else 0
        
        return min(1.0, dist_factor * 0.5 + path_factor * 0.3 + forward_factor * 0.2)

    def cmd_callback(self, msg):
        """Receive original command and apply obstacle avoidance"""
        self.current_speed = msg.drive.speed
        self.current_steering = msg.drive.steering_angle
        
        # Apply obstacle avoidance
        safe_speed, safe_steering = self.calculate_safe_command()
        
        # Create and publish safe command
        safe_msg = to_ackermann(safe_speed, safe_steering, msg.header.stamp)
        safe_msg.header = msg.header  # Preserve original header
        self.cmd_pub.publish(safe_msg)
        
        # Log if we're making corrections
        if abs(safe_steering - self.current_steering) > 0.1 or abs(safe_speed - self.current_speed) > 0.1:
            self.get_logger().info(f"🚨 Obstacle avoidance active! "
                                 f"Speed: {self.current_speed:.2f} → {safe_speed:.2f}, "
                                 f"Steering: {self.current_steering:.2f} → {safe_steering:.2f}")

    def calculate_safe_command(self):
        """Calculate safe speed and steering based on obstacles"""
        if not self.obstacles:
            return self.current_speed, self.current_steering
        
        # Find most threatening obstacle
        max_threat = max(obs['threat_level'] for obs in self.obstacles)
        most_threatening = max(self.obstacles, key=lambda x: x['threat_level'])
        
        safe_speed = self.current_speed
        safe_steering = self.current_steering
        
        # Emergency braking for critical threats
        if most_threatening['distance'] < self.critical_distance:
            safe_speed = 0.0
            self.get_logger().warn(f"🛑 EMERGENCY BRAKE! Obstacle at {most_threatening['distance']:.2f}m")
            return safe_speed, safe_steering
        
        # Speed reduction based on threat level
        if max_threat > 0.3:
            speed_factor = 1.0 - (max_threat * 0.6)  # Reduce up to 60% of speed
            safe_speed = self.current_speed * max(0.2, speed_factor)
        
        # Steering avoidance
        steering_correction = 0.0
        
        for obstacle in self.obstacles:
            if obstacle['threat_level'] > 0.2:  # Only avoid significant threats
                # Calculate avoidance direction
                obs_y = obstacle['y']
                obs_distance = obstacle['distance']
                
                # Steer away from obstacle (positive y = steer right, negative y = steer left)
                avoidance_direction = -np.sign(obs_y)  # Steer opposite to obstacle
                
                # Avoidance strength based on threat and distance
                avoidance_strength = (obstacle['threat_level'] * self.avoidance_gain) / max(0.5, obs_distance)
                
                correction = avoidance_direction * avoidance_strength
                steering_correction += correction
        
        # Apply steering correction with limits
        safe_steering = self.current_steering + steering_correction
        safe_steering = np.clip(safe_steering, -0.5, 0.5)  # Limit steering angle
        
        return safe_speed, safe_steering

    def publish_obstacle_warnings(self):
        """Publish obstacle positions for visualization"""
        if not self.obstacles:
            return
            
        # Publish most threatening obstacle
        most_threatening = max(self.obstacles, key=lambda x: x['threat_level'])
        
        warning_msg = PoseStamped()
        warning_msg.header.stamp = self.get_clock().now().to_msg()
        warning_msg.header.frame_id = 'base_link'
        
        warning_msg.pose.position.x = float(most_threatening['x'])
        warning_msg.pose.position.y = float(most_threatening['y'])
        warning_msg.pose.position.z = float(most_threatening['threat_level'])  # Encode threat in z
        
        self.obstacle_pub.publish(warning_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Obstacle avoidance node shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()