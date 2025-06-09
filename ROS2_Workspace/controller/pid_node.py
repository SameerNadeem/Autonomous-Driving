import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.qos import qos_profile_sensor_data  # Quality of Service settings for real-time data
import time
from ros2_numpy import pose_to_np
from collections import deque
import numpy as np
from controller.targets import Vehicle, StopSign, SpeedSign, GreenLight, YellowLight, RedLight
from vision_msgs.msg import LabelInfo, Detection3DArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from line_follower.conversions import from_label_info, from_detection3d_array, to_ackermann

class PIDcontroller(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # Define Quality of Service (QoS) for communication
        qos_profile = qos_profile_sensor_data  # Suitable for sensor data
        qos_profile.depth = 1  # Keep only the latest message

        # Create a publisher for sending AckermannDriveStamped messages to the '/autonomous/ackermann_cmd' topic
        self.publisher = self.create_publisher(AckermannDriveStamped, 'autonomous/ackermann_cmd', qos_profile)

        # Create a subscription to listen for Detection3DArray messages from the '/objects_3d' topic
        # When a message is received, the 'self.detection_callback' function is called
        self.detection_sub = self.create_subscription(Detection3DArray,'/objects_3d', self.detection_callback, qos_profile)

        self.error_sub = self.create_subscription(PoseStamped,'/errors', self.error_callback, qos_profile)

        # QoS for transient local data (e.g., static label map)
        qos_transient = QoSProfile(depth=1)
        qos_transient.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # Subscription
        self.label_sub = self.create_subscription(
            LabelInfo,
            '/label_mapping',
            self.label_mapping_callback,
            qos_transient
        )

        # Load parameters
        self.params_set = False
        self.declare_params()
        self.load_params()

        # Create a timer that calls self.load_params every 10 seconds (10.0 seconds)
        self.timer = self.create_timer(10.0, self.load_params)

        self.last_heading_err = 0.0
        self.last_waypoint_err = 0.0
        self.last_time = time.time()
        self.last_steering_angle = 0.0

        self.kp1 = 0.1
        self.kp2 = 0.1
        self.kd1 = 0.0
        self.kd2 = 0.0

        # Initialize deque with a fixed length of self.max_out
        # This could be useful to allow the vehicle to temporarily lose the track for up to max_out frames before deciding to stop. (Currently not used yet.)
        self.max_out = 9
        self.success = deque([True] * self.max_out, maxlen=self.max_out)

  
    def label_mapping_callback(self, msg: LabelInfo):
        self.id2label = from_label_info(msg)
        self.label2id = {lbl: id for id, lbl in self.id2label.items()}
        self.get_logger().info(f"Label mapping received: {self.id2label}")
        self.setup_object_callbacks()
    
    def setup_object_callbacks(self):
        self.lbl2target = {}

        if 'car' in self.label2id:
            self.lbl2target['car'] = Vehicle(self.label2id['car'], 'car')
        if 'stop' in self.label2id:
            self.lbl2target['stop'] = StopSign(self.label2id['stop'], 'stop')
        if 'speed_2mph' in self.label2id:
            self.lbl2target['speed_2mph'] = SpeedSign(self.label2id['speed_2mph'], 'speed_2mph', speed=0.6)
        if 'speed_3mph' in self.label2id:
            self.lbl2target['speed_3mph'] = SpeedSign(self.label2id['speed_3mph'], 'speed_3mph', speed=0.8)
        if 'red' in self.label2id:
            self.lbl2target['red'] = RedLight(self.label2id['red'], 'red')
        if 'yellow' in self.label2id:
            self.lbl2target['yellow'] = YellowLight(self.label2id['yellow'], 'yellow')
        if 'green' in self.label2id:
            self.lbl2target['green'] = GreenLight(self.label2id['green'], 'green')

        self.id2target = {
            self.label2id[lbl]: fn for lbl, fn in self.lbl2target.items()
        }

    def detection_callback(self, msg: Detection3DArray):
        detections, timestamp_unix = from_detection3d_array(msg)

        for d in detections:
            lbl, score, x, y, z = d

            if lbl in self.lbl2target.keys():
                self.lbl2target[lbl].update(score, (x, y, z), self)
            elif lbl == 'center':
                self.waypoint_callback(score, y, timestamp_unix)

    def error_callback(self, msg: PoseStamped):
        # Convert incoming pose message to position, heading, and timestamp
        point, heading, timestamp_unix = pose_to_np(msg)

        if not isinstance(point, np.ndarray):
            point = np.asarray(point)

        # If the detected point contains NaN (tracking lost) stop the vehicle
        if np.isnan(point).any():
            self.success.append(False)
            if any(self.success):
                # Keep driving with the last known steering angle unless the line is lost for self.max_out consecutive frames
                ackermann_msg = to_ackermann(self.speed, self.last_steering_angle, timestamp_unix)
            else:
                ackermann_msg = to_ackermann(0.0, self.last_steering_angle, timestamp_unix)
                self.publisher.publish(ackermann_msg) # BRAKE
            return
        else:
            self.success.append(True)

        # Calculate time difference since last callback
        dt = timestamp_unix - self.last_time

        # Update the last time to the current timestamp
        self.last_time = timestamp_unix

        # Get x and y coordinates (ignore z), and compute the error in y
        heading_err, waypoint_err, _ = point

        # Calculate the derivative of the error (change in error over time)
        d_heading_err = (heading_err - self.last_heading_err) / dt
        d_waypoint_err = (waypoint_err - self.last_waypoint_err) / dt

        # Compute the steering angle using a PD controller
        steering_angle = (self.kp1 * heading_err) + (self.kd1 * d_heading_err) + (self.kp2 * waypoint_err) + (self.kd2 * d_waypoint_err)

        base_speed = 1.5
        min_speed = 0.3
        abs_steering = abs(steering_angle)

        if abs_steering < 0.05:      # < 3 degrees
            self.speed = base_speed
        elif abs_steering < 0.2:     # 3-11 degree
            self.speed = base_speed * 0.8
        elif abs_steering < 0.4:     # 11-23 degrees
            self.speed = base_speed * 0.6
        else:                        # > 23 degrees
            self.speed =  max(min_speed, base_speed * 0.4)
        # Get the timestamp from the message header
        timestamp = msg.header.stamp

        # Create an Ackermann drive message with speed and steering angle
        ackermann_msg = to_ackermann(self.speed, steering_angle, timestamp)

        # Publish the message to the vehicle
        self.publisher.publish(ackermann_msg)

        # Save the current error for use in the next iteration
        self.last_heading_err = heading_err
        self.last_waypoint_err = waypoint_err

        # If tracking is lost, continue driving with the last known steering angle
        # to follow the previously estimated path (e.g., maintain the current curve)
        self.last_steering_angle = steering_angle

    def waypoint_callback(self, score, y, timestamp_unix):
        # Process 3D detection-based waypoint (instead of pose data).
        # If score is 0.0 (i.e., no valid detection), consider tracking lost and stop the vehicle.
        pass
        '''
        self.get_logger().info(f"{type(self.speed)=}, {self.speed=}")
        if score == 0.0:
            self.success.append(False)
            if any(self.success):
                # Keep driving with the last known steering angle unless the line is lost for self.max_out consecutive frames
                ackermann_msg = to_ackermann(self.speed, self.last_steering_angle, timestamp_unix)
            else:
                ackermann_msg = to_ackermann(0.0, self.last_steering_angle, timestamp_unix)
                self.publisher.publish(ackermann_msg)  # BRAKE
            return
        else:
            self.success.append(True)

        # Calculate time difference since last callback
        dt = timestamp_unix - self.last_time
        self.last_time = timestamp_unix  # Update timestamp

        error = 0.0 - y  # Lateral deviation from center (y=0 is the desired centerline)
        d_error = (error - self.last_error) / dt  # Derivative of error

        # PD control for steering
        steering_angle = (self.kp * error) + (self.kd * d_error)

        # Send speed and steering command
        ackermann_msg = to_ackermann(self.speed, steering_angle, timestamp_unix)
        self.publisher.publish(ackermann_msg)

        self.last_error = error
        self.last_steering_angle = steering_angle  # Store for fallback during lost tracking
        '''

    def declare_params(self):

        # Declare parameters with default values
        self.declare_parameters(
            namespace='',
            parameters=[
                ('kp', 0.9),
                ('kd', 0.0),
                ('max_speed', 0.8),
                ('min_speed', 0.4),
            ]
        )

    def load_params(self):
        try:
            self.kp = self.get_parameter('kp').get_parameter_value().double_value
            self.kd = self.get_parameter('kd').get_parameter_value().double_value
            self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
            self.min_speed = self.get_parameter('min_speed').get_parameter_value().double_value
            self.speed = self.max_speed
            self.mid_speed = (self.max_speed + self.min_speed) / 2

            if not self.params_set:
                self.get_logger().info("Parameters loaded successfully")
                self.params_set = True

        except Exception as e:
            self.get_logger().error(f"Failed to load parameters: {e}")

def main(args=None):
    rclpy.init(args=args)

    node = PIDcontroller()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
