import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import time

class DummyPublisher(Node):
    def __init__(self):
        super().__init__('dummy_pose_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/objects', 10)

        # Define id-to-target mapping
        self.id2target = {
            0: ('car', 0),
            1: ('stop', 1),
            2: ('speed_2mph', 2),
            3: ('speed_3mph', 3),
            4: ('green', 4),
            5: ('red', 5),
        }

        self.timer = self.create_timer(0.3, self.publish_pose)
        self.current_id = 0
        self.current_x = 3.0

    def publish_pose(self):
        if self.current_id >= len(self.id2target):
            self.get_logger().info('Finished publishing all targets.')
            rclpy.shutdown()
            return

        target_name, target_id = self.id2target[self.current_id]

        pose_msg = PoseStamped()
        pose_msg.header = Header()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'

        pose_msg.pose.position.x = self.current_x
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = float(target_id)  # Encode ID in z

        pose_msg.pose.orientation.w = 1.0  # Identity quaternion

        self.publisher.publish(pose_msg)
        self.get_logger().info(f'Published: {target_name} at x={self.current_x:.2f}, z={target_id}')

        self.current_x -= 0.1
        if self.current_x < 0.0:
            self.current_id += 1
            self.current_x = 3.0

def main(args=None):
    rclpy.init(args=args)
    node = DummyPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
