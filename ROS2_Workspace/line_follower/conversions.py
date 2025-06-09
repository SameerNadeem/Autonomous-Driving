from rclpy.clock import Clock
from builtin_interfaces.msg import Time as TimeMsg
import numpy as np
import cv2
import struct
from sensor_msgs.msg import Image, CompressedImage, Imu, PointCloud2, MagneticField, PointField
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
from ackermann_msgs.msg import AckermannDriveStamped
import math
import warnings
import time 
from builtin_interfaces.msg import Time as TimeMsg
import rclpy

from vision_msgs.msg import Detection3D, ObjectHypothesisWithPose, BoundingBox3D, Detection3DArray, LabelInfo, VisionClass
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header


def get_ros_timestamp(timestamp = None) -> TimeMsg:
    """
    Ensures a ROS timestamp message (builtin_interfaces.msg.Time) is returned.

    This helper function handles different possible inputs for a timestamp and
    returns the appropriate message object ready to be assigned to a header stamp.

    Args:
        timestamp_input: An optional timestamp. Can be:
                         - None: The current ROS time (Clock().now()) will be used.
                         - rclpy.time.Time: The object will be converted using .to_msg().
                         - builtin_interfaces.msg.Time: The object will be returned directly.
                         - Any other type: Defaults to using the current ROS time.

    Returns:
        A builtin_interfaces.msg.Time message object suitable for msg.header.stamp.
    """
    if isinstance(timestamp, rclpy.time.Time):
        # Input is an rclpy Time object, convert it to a message
        return timestamp.to_msg()
    elif isinstance(timestamp, TimeMsg):
        # Input is already a Time message object, return it directly
        return timestamp
    elif isinstance(timestamp, int):
        # Convert Unix timestamp (in seconds) to ROS TimeMsg
        sec = int(timestamp)
        nanosec = int((timestamp - sec) * 1e9)
        return TimeMsg(sec=sec, nanosec=nanosec)
    else:
        # Input is None or an unexpected type, default to current time
        return Clock().now().to_msg()

def get_timestamp_unix(msg):
    """
    Extracts the Unix timestamp (seconds since epoch) from a message header's stamp.
    If the message does not have the expected header and stamp attributes,
    it logs a warning and returns the current ROS time as a Unix timestamp.

    Args:
        msg (object): A message object with a 'header' attribute containing a 'stamp'
                      attribute.

    Returns:
        float: The Unix timestamp in seconds.  Returns the current ROS time
               as a Unix timestamp if the input message is invalid.
    """
    try:
        timestamp_ros = msg.header.stamp
        timestamp_unix = timestamp_ros.to_sec()
        return timestamp_unix
    except AttributeError:
        warnings.warn("Message does not have a valid header.stamp. Returning current system time.")
        timestamp_unix = time.time()
        return timestamp_unix
    
# --- Orientation ---
def quaternion_to_yaw(quaternion):
    """
    Converts a ROS Quaternion message to a yaw angle (rotation around Z-axis).

    Parameters
    ----------
    quaternion : geometry_msgs.msg.Quaternion
        The quaternion to convert.

    Returns
    -------
    float:
        The yaw angle in radians.
    """
    t3 = +2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    t4 = +1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    yaw = math.atan2(t3, t4)
    return yaw

def yaw_to_quaternion(yaw_angle):
    """
    Converts a yaw angle (rotation around the Z-axis) to a ROS Quaternion message.

    In the context of driving, this yaw angle represents the vehicle's heading direction.
    A yaw angle of 0 radians corresponds to the vehicle pointing in the +X direction.

    Parameters
    ----------
    yaw_angle : float
        The yaw angle in radians. This angle represents the vehicle's heading
        direction, measured counterclockwise from the +X axis.

    Returns
    -------
    geometry_msgs.msg.Quaternion
        A ROS Quaternion message representing the rotation around the Z-axis
        that corresponds to the given yaw angle. The x and y components of the
        quaternion will be 0.0, as this rotation is solely around the z-axis.
    """
    quaternion = Quaternion()
    quaternion.x = 0.0  # Rotation around Z-axis, so no rotation around X
    quaternion.y = 0.0  # Rotation around Z-axis, so no rotation around Y
    quaternion.z = np.sin(yaw_angle / 2.0)
    quaternion.w = np.cos(yaw_angle / 2.0)
    return quaternion

# --- Label Info ---
def from_label_info(label_info_msg: LabelInfo) -> dict:
    """
    Converts a vision_msgs/LabelInfo message to a Python dict.

    Parameters
    ----------
    label_info_msg : vision_msgs.msg.LabelInfo
        A ROS2 LabelInfo message containing class_map (list of VisionClass)

    Returns
    -------
    dict
        Mapping from class_id (int) to class_name (str)
    """
    return {vc.class_id: vc.class_name for vc in label_info_msg.class_map}

def to_label_info(id_to_label: dict, timestamp=None, frame_id='base_link'):
    """
    Converts a sequential ID-to-label dict to a LabelInfo message.

    Parameters
    ----------
    id_to_label : dict
        Mapping from integer class IDs (e.g., 0, 1, 2) to label names.
    timestamp : float, rclpy.time.Time, or None
        Optional timestamp. If None, current time is used.
    frame_id : str
        Frame ID for the header.

    Returns
    -------
    LabelInfo
        A vision_msgs LabelInfo message.
    """
    msg = LabelInfo()

    # Header using existing helper
    msg.header = Header()
    msg.header.stamp = get_ros_timestamp(timestamp)
    msg.header.frame_id = frame_id

    # Build class_map using VisionClass entries
    msg.class_map = []
    for id, label in id_to_label.items():
        vc = VisionClass()
        vc.class_id = id
        vc.class_name = label
        msg.class_map.append(vc)

    return msg


# --- Detection 3D ---
def to_detection3d(label, score, x, y, z, timestamp=None, frame_id='base_link'):
    """
    Creates a Detection3D message from label, score, and 3D position.

    Parameters
    ----------
    label : str
        Class label (e.g., "car", "person").
    score : float
        Confidence score between 0 and 1.
    x, y, z : float
        3D position of the detected object.
    timestamp : rclpy.time.Time, optional
        Timestamp for the message. Uses current time if None.
    frame_id : str
        Frame ID for the detection (default: 'base_link').

    Returns
    -------
    vision_msgs.msg.Detection3D
        A Detection3D message ready to publish.
    """
    detection = Detection3D()

    # Header
    detection.header = Header()
    detection.header.stamp = get_ros_timestamp(timestamp)
    detection.header.frame_id = frame_id

    # Hypothesis
    hypothesis = ObjectHypothesisWithPose()
    hypothesis.hypothesis.class_id = label
    hypothesis.hypothesis.score = score

    # Pose
    pose_stamped = np_to_pose(np.array([x, y, z]), yaw_angle=0.0)
    hypothesis.pose.pose = pose_stamped.pose

    # Default bounding box
    bbox = BoundingBox3D()
    bbox.center = pose_stamped.pose
    bbox.size.x = 1.0
    bbox.size.y = 1.0
    bbox.size.z = 1.0

    # Fill detection message
    detection.results.append(hypothesis)
    detection.bbox = bbox

    return detection

def to_detection3d_array(detections: dict, timestamp=None, frame_id='base_link'):
    """
    Converts a dictionary of detections into a Detection3DArray using to_detection3d().

    Parameters
    ----------
    detections : dict
        Mapping from class ID to a list in the format [label:str, score:float, x:float, y:float, z:float].
    timestamp : rclpy.time.Time or float, optional
        Timestamp for the header. Uses current time if None.
    frame_id : str
        Frame ID for all detections.

    Returns
    -------
    Detection3DArray
        A ROS2 Detection3DArray message with populated detections.
    """
    array_msg = Detection3DArray()
    array_msg.header = Header()
    array_msg.header.stamp = get_ros_timestamp(timestamp)
    array_msg.header.frame_id = frame_id

    for _, (label, score, x, y, z) in detections.items():
        detection = to_detection3d(
            label=label,
            score=score,
            x=x,
            y=y,
            z=z,
            timestamp=timestamp,
            frame_id=frame_id
        )
        array_msg.detections.append(detection)

    return array_msg


def from_detection3d(detection):
    """
    Extracts label, score, and 3D position (x, y, z) from a Detection3D message.

    Parameters
    ----------
    detection : vision_msgs.msg.Detection3D
        A Detection3D message.

    Returns
    -------
    list
        A list in the format [label, score, x, y, z].
    """
    if not detection.results:
        return [None, 0.0, 0.0, 0.0, 0.0]  # fallback if no results

    result = detection.results[0]
    label = result.hypothesis.class_id
    score = result.hypothesis.score
    pose = result.pose.pose.position

    return [label, score, pose.x, pose.y, pose.z]

def from_detection3d_array(msg):
    """
    Converts a Detection3DArray message into a list of [label, score, x, y, z] entries,
    using the from_detection3d() helper for each Detection3D.

    Parameters
    ----------
    msg : vision_msgs.msg.Detection3DArray
        A Detection3DArray message containing multiple Detection3D objects.

    Returns
    -------
    list of lists
        Each inner list has the format [label, score, x, y, z].
    """
    return [from_detection3d(d) for d in msg.detections], get_timestamp_unix(msg)

# --- Image ---
def image_to_np(msg):
    image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    return image, get_timestamp_unix(msg)

def np_to_image(image: np.ndarray, timestamp=None) -> Image:
    """
    Convert a numpy image (BGR or Grayscale) to a ROS Image message.

    Args:
        image: The numpy array representing the image.
               Expected shapes:
               - H x W    (grayscale, mono8)
               - H x W x 1 (grayscale, mono8)
               - H x W x 3 (color, bgr8 assumed)
        timestamp: Optional timestamp for the image header. If None,
                   tries to get current time (best effort outside a node).

    Returns:
        A sensor_msgs.msg.Image message.

    Raises:
        ValueError: If the image shape is not supported.
    """
    # Create an Image message
    ros_image = Image()

    # Determine encoding and channel count based on image shape
    if image.ndim == 2:
        # Grayscale image (H x W)
        height, width = image.shape
        channels = 1
        ros_image.encoding = 'mono8' # Standard 8-bit grayscale
    elif image.ndim == 3:
        # Image with channels (H x W x C)
        height, width, channels = image.shape
        if channels == 1:
            # Grayscale image (H x W x 1)
            ros_image.encoding = 'mono8'
        elif channels == 3:
            # Color image (H x W x 3), assume BGR
            ros_image.encoding = 'bgr8'
        else:
            raise ValueError(f"Unsupported number of channels: {channels}. "
                             "Expected 1 (mono8) or 3 (bgr8).")
    else:
        raise ValueError(f"Unsupported image dimensions: {image.ndim}. "
                         "Expected 2 or 3 dimensions.")

    # Set dimensions
    ros_image.height = height
    ros_image.width = width

    # Set step (row stride in bytes) and data
    # itemsize is the size of one element (e.g., 1 byte for uint8)
    ros_image.step = width * channels * image.itemsize
    ros_image.data = image.tobytes()

    # Set timestamp
    ros_image.header.stamp = get_ros_timestamp(timestamp)

    return ros_image

# --- CompressedImage ---
def compressedimage_to_np(msg):
    image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
    return image, get_timestamp_unix(msg)

def np_to_compressedimage(image, timestamp=None):
    ros_image = CompressedImage()
    ros_image.header.stamp = get_ros_timestamp(timestamp)
    ros_image.format = "jpeg"
    ros_image.data = np.array(cv2.imencode('.jpg', image)[1]).tobytes()
    return ros_image

# --- MultiArray ---
def multiarray_to_np(msg, dtype=np.float32):
    """Converts a MultiArray message to a flat NumPy array (shape N,).

    Args:
        msg (std_msgs.msg.MultiArray): The input MultiArray message
                                       (e.g., Float32MultiArray, Int16MultiArray, etc.).

    Returns:
        numpy.ndarray: The flattened NumPy array.
    """
    return np.array(msg.data, dtype=dtype), get_timestamp_unix(msg)

# --- Imu ---
def imu_to_np(msg):
    """Converts an Imu message to a NumPy array (excluding orientation).

    Args:
        msg (Imu): The input Imu message.

    Returns:
        numpy.ndarray: A NumPy array containing linear acceleration (x, y, z)
                       and angular velocity (x, y, z). The shape of the array is (6,).
    """
    angular_velocity = msg.angular_velocity
    linear_acceleration = msg.linear_acceleration
    arr = np.array([linear_acceleration.x, linear_acceleration.y, linear_acceleration.z,
                     angular_velocity.x, angular_velocity.y, angular_velocity.z])
    
    return arr, get_timestamp_unix(msg)

def np_to_imu(array, frame_id='base_link', timestamp=None):
    """Converts a NumPy array to an Imu message (excluding orientation).

    Args:
        array (numpy.ndarray): A NumPy array of shape (6,) containing
                               linear acceleration (x, y, z) and angular velocity (x, y, z)
                               in that order.
        frame_id (str, optional): The frame ID for the Imu message. Defaults to 'base_link'.
        timestamp (rclpy.time.Time, optional): The timestamp for the Imu message.
                                               If None, the current time is used. Defaults to None.

    Returns:
        Imu: The converted Imu message.
    """
    msg = Imu()
    msg.header.frame_id = frame_id
    msg.header.stamp = get_ros_timestamp(timestamp)

    msg.linear_acceleration.x = float(array[0])
    msg.linear_acceleration.y = float(array[1])
    msg.linear_acceleration.z = float(array[2])

    msg.angular_velocity.x = float(array[3])
    msg.angular_velocity.y = float(array[4])
    msg.angular_velocity.z = float(array[5])

    return msg

# --- PointCloud2 ---

def np_to_pointcloud(points, frame_id='base_link', timestamp=None):
    """
    Create a ROS2 PointCloud2 message from a set of 2D or 3D points.
    """
    points = np.array(points)
    
    if points.shape[1] not in [2, 3]:
        raise ValueError("Points should have 2 or 3 columns representing [x, y] or [x, y, z].")
    
    if points.shape[1] == 2:
        points = np.hstack((points, np.zeros((points.shape[0], 1))))

    msg = PointCloud2()
    msg.header.stamp = get_ros_timestamp(timestamp)
    msg.header.frame_id = frame_id

    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
    ]
    msg.is_bigendian = False  # Indicates that the data is stored in little-endian byte order.
    msg.point_step = 12       # Specifies that each point in the data occupies 12 bytes.
    msg.is_dense = True       # Confirms that there are no invalid or missing points in the data.

    point_data = bytearray()
    for point in points:
        point_data.extend(struct.pack('fff', point[0], point[1], point[2]))

    msg.data = bytes(point_data)
    msg.row_step = msg.point_step * points.shape[0]
    msg.height = 1
    msg.width = points.shape[0]

    return msg

# --- LaserScan ---
def scan_to_np(msg) -> np.ndarray:
    """
    Converts LaserScan msg to [x, y, intensity] NumPy array.

    Filters out points with non-finite (inf, -inf, NaN) ranges.
    Assumes valid inputs (matching sizes, non-empty).

    Args:
        msg: sensor_msgs.msg.LaserScan message.

    Returns:
        np.ndarray: Nx3 array [x, y, intensity] or (0, 3) if no valid points.
    """
    ranges_np = np.array(msg.ranges, dtype=np.float32) # Get ranges array
    intensities_np = np.array(msg.intensities, dtype=np.float32) # Get intensities array

    angles_np = msg.angle_min + np.arange(len(ranges_np)) * msg.angle_increment # Calculate angle for each range

    valid_mask = np.isfinite(ranges_np) # Create mask for finite ranges only

    # Apply mask to all relevant arrays
    valid_ranges = ranges_np[valid_mask]
    valid_angles = angles_np[valid_mask]
    valid_intensities = intensities_np[valid_mask]

    if valid_ranges.size == 0: # Check if any points remain
        return np.empty((0, 3), dtype=np.float32) # Return empty if no valid points

    # Calculate x, y coordinates for valid points
    x_coords = valid_ranges * np.cos(valid_angles)
    y_coords = valid_ranges * np.sin(valid_angles)

    # Stack [x, y, intensity] columns into final array
    pointcloud_xyi = np.column_stack((x_coords, y_coords, valid_intensities)).astype(np.float32)

    return pointcloud_xyi, get_timestamp_unix(msg)

# --- AckermannDriveStamped ---
def from_ackermann(msg):
    """
    Extracts speed, steering angle, and Unix timestamp from an AckermannDriveStamped message.

    Args:
        msg: The input ackermann_msgs.msg.AckermannDriveStamped message.

    Returns:
        A tuple containing:
        - speed (float): Longitudinal speed in m/s.
        - steering_angle (float): Steering angle in radians.
        - timestamp (float): Timestamp of the message as Unix time (seconds).
    """ 
    speed = msg.drive.speed
    steering_angle = msg.drive.steering_angle
    return speed, steering_angle, get_timestamp_unix(msg)

def to_ackermann(speed, steering_angle, timestamp=None):
    """
    Creates an AckermannDriveStamped message from speed, steering angle, and an optional timestamp.

    Args:
        speed: The desired longitudinal speed (m/s).
        steering_angle: The desired steering angle (radians).
        timestamp: An optional rclpy.time.Time object for the header stamp.
                   If None, the current ROS time is used.

    Returns:
        An ackermann_msgs.msg.AckermannDriveStamped message populated with the provided data.
    """
    msg = AckermannDriveStamped()
    msg.header.stamp = get_ros_timestamp(timestamp)
    msg.drive.speed = float(speed)
    msg.drive.steering_angle = float(steering_angle)
    return msg

# --- Pose ---
def pose_to_np(msg):
    """
    Converts a ROS Pose message to a NumPy array and extracts the yaw angle.

    Parameters
    ----------
    msg : geometry_msgs.msg.Pose
        The input ROS Pose message.

    Returns
    -------
    tuple (numpy.ndarray, float):
        A tuple containing:
        - A NumPy array of shape (3,) representing the x, y, and z coordinates of the pose.
        - The yaw angle (rotation around the Z-axis) in radians.
    """
    # Extract position
    point = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    # Extract orientation and convert to yaw angle
    quaternion = msg.pose.orientation
    yaw_angle = quaternion_to_yaw(quaternion)  # Use the helper function

    return point, yaw_angle, get_timestamp_unix(msg)

def np_to_pose(point, yaw_angle, frame_id='base_link', timestamp=None):
    """
    Converts a NumPy array representing a 3D point and a yaw angle to a ROS PoseStamped message.

    The yaw angle represents the orientation of the pose around the Z-axis.

    Parameters
    ----------
    point : numpy.ndarray
        A NumPy array of shape (3,) representing the x, y, and z coordinates of the pose.
    yaw_angle : float
        The yaw angle in radians, representing the rotation around the Z-axis.
    frame_id : str, optional
        The frame ID for the PoseStamped message header. Defaults to 'base_link'.
    timestamp : rclpy.time.Time, optional
        The timestamp for the PoseStamped message header. If None, the current time is used.
        Defaults to None.

    Returns
    -------
    geometry_msgs.msg.PoseStamped
        A ROS PoseStamped message representing the 3D point and orientation.
    """
    msg = PoseStamped()

    # Handle timestamp
    msg.header.stamp = get_ros_timestamp(timestamp)

    # Set frame ID
    msg.header.frame_id = frame_id

    # Set position
    msg.pose.position.x = float(point[0])
    msg.pose.position.y = float(point[1])
    msg.pose.position.z = float(point[2]) if len(point) > 2 else 0.0

    # Set orientation using the helper function
    msg.pose.orientation = yaw_to_quaternion(yaw_angle)

    return msg

# --- Path ---
def np_to_path(waypoints, frame_id='base_link', timestamp=None):
    """
    Converts a list or NumPy array of waypoints into a nav_msgs/Path message.

    Each waypoint should consist of [x, y, theta], where x and y are coordinates
    and theta is the orientation (yaw) in radians.

    Args:
        waypoints: A NumPy array or list-like structure of shape (N, 3)
                   representing N waypoints as [x, y, theta]. A single waypoint
                   can also be provided as a 1D array or list of length 3.
        frame_id: The coordinate frame ID to set in the Path header.
                  Defaults to 'base_link'.
        timestamp: An optional rclpy.time.Time object for the Path header stamp.
                   If None, the current ROS time (Clock().now()) is used.

    Returns:
        A nav_msgs.msg.Path message populated with poses derived from the waypoints.

    Raises:
        ValueError: If the input waypoints do not result in an Nx3 shape
                    after conversion and reshaping (for single waypoint input).
    """
    # Ensure waypoints is a numpy array.
    waypoints = np.array(waypoints)

    # If a single waypoint is provided, reshape it.
    if waypoints.ndim == 1:
        waypoints = waypoints.reshape(1, -1)

    if waypoints.shape[1] != 3:
        raise ValueError("Waypoints should have 3 columns: x, y, and theta.")

    path_msg = Path()
    path_msg.header.stamp = get_ros_timestamp(timestamp)
    path_msg.header.frame_id = frame_id

    for point in waypoints:
        pose = np_to_pose(point[:2], point[2])
        pose.header.frame_id = frame_id
        path_msg.poses.append(pose)

    return path_msg

# --- MagneticField ---
def magneticfield_to_np(msg):
    """
    Converts a sensor_msgs/MagneticField message to a NumPy array.

    Args:
        msg (sensor_msgs.msg.MagneticField): The MagneticField message to convert.

    Returns:
        numpy.ndarray: A NumPy array of shape (3,) containing the magnetic field
                       values (x, y, z). Returns None if the input is None.
    """
    data = np.array([msg.magnetic_field.x, msg.magnetic_field.y, msg.magnetic_field.z])
    
    return data, get_timestamp_unix(msg)

def np_to_magneticfield(array, frame_id="base_link", timestamp=None):
    """
    Converts a NumPy array to a sensor_msgs/MagneticField message.

    Args:
        array (numpy.ndarray): A NumPy array of shape (3,) containing the
                               magnetic field values (x, y, z).
        frame_id (str, optional): The frame_id to set in the message header.
                                  Defaults to "base_link".
        timestamp (rospy.Time or None, optional): The timestamp to set in the
                                                 message header. If None, the
                                                 current time is used. Defaults to None.

    Returns:
        sensor_msgs.msg.MagneticField: The constructed MagneticField message.
                                       Returns None if the input array is invalid.
    """
    if not isinstance(array, np.ndarray) or array.shape != (3,):
        raise TypeError("Input array must be a NumPy array of shape (3,).")

    msg = MagneticField()
    msg.header.frame_id = frame_id
    msg.header.stamp = get_ros_timestamp(timestamp)
    msg.magnetic_field.x = float(array[0])
    msg.magnetic_field.y = float(array[1])
    msg.magnetic_field.z = float(array[2])
    return msg