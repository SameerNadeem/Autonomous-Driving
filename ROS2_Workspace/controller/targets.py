from collections import deque
import numpy as np
from ros2_numpy import to_ackermann
import time

class Target():
    def __init__(self, id, label, threshold=0.3, len_history=10, min_distance=1.5):
        self.id = id          # class ID
        self.label = label    # Descriptive label for the target (e.g., 'stop_sign')
        self.threshold = threshold  # Minimum ratio of positive detections to consider the target visible
        self.len_history = len_history  # Number of recent detection results to track
        self.history = deque([False] * self.len_history, maxlen=self.len_history)  # Detection history buffer
        self.has_reacted = False  # Flag to track if the system has already reacted to this target
        self.min_distance = min_distance  # Distance threshold to trigger a reaction (e.g., braking)
        self.position = None # Latest detected position of the target 

    @property
    def visible(self):
        # Compute visibility based on the average detection history
        return np.mean(self.history) > self.threshold

    @property
    def in_range(self):
        # Return True if position is known and x-distance is within the threshold
        return self.position is not None and self.position[0] <= self.min_distance


    def update(self, score, position, node):
        self.position = position

        # Update detection history based on current observation
        if score == 0.0:
            self.history.append(False)
        else:
            self.history.append(True)

        # Trigger action if target is visible and in range and not yet reacted
        if self.visible and not self.has_reacted:
            if self.in_range:
                self.has_reacted = self.react(node)  # Pass node here

        # Reset if target is no longer visible
        elif not any(self.history) and self.has_reacted:
            self.has_reacted = False

    def react(self, node):
        # Override this method in subclasses
        return False

class Vehicle():
    def __init__(self, id, label, min_distance = 1.0, max_distance = 2.5):
        self.id = id          # class ID
        self.label = label    # Descriptive label for the target (e.g., 'stop_sign')
        self.min_distance = min_distance
        self.max_distance = max_distance

    def update(self, score, position, node):

        if score == 0.0:
            return # dont change speed

        x = position[0]
        
        if x < self.min_distance:
            node.speed = 0.0
            node.get_logger().info(f"Leading vehicle too close at {x:.2f} meters. Stopping.")
        else:
            # Compute how far we are relative to the minimum safe distance (clamped between 0 and 1)
            distance_ratio = max(0.0, min((x - self.min_distance) / (self.max_distance - self.min_distance), 1.0))

            # Interpolate speed between min_speed and max_speed
            node.speed = node.min_speed + distance_ratio * (node.max_speed - node.min_speed)

            node.get_logger().info(f"Leading vehicle at {x:.2f} meters. Adjusting speed to {node.speed:.2f}.")

class StopSign(Target):
    def __init__(self, id, label, threshold=0.3, len_history=10, min_distance=1.5, duration=2.0):
        super().__init__(id, label, threshold, len_history, min_distance)
        self.duration = duration

    def react(self, node):
        node.get_logger().info(f"Stopping for stop sign at x={self.position[0]:.2f}")
        msg = to_ackermann(0.0, node.last_steering_angle)
        node.publisher.publish(msg)
        time.sleep(self.duration)
        node.get_logger().info(f"Resume driving...")
        return True

class SpeedSign(Target):
    def __init__(self, id, label, threshold=0.3, len_history=10, min_distance=1.5, speed=0.6):
        super().__init__(id, label, threshold, len_history, min_distance)
        self.speed = speed

    def react(self, node):
        node.get_logger().info(f"Setting speed to {self.speed:.2f} m/s at x={self.position[0]:.2f}")
        node.speed = self.speed  # Assuming this is consumed somewhere else in your control loop
        return True

class RedLight(Target):
    def __init__(self, id, label, threshold=0.3, len_history=10, min_distance=1.5):
        super().__init__(id, label, threshold, len_history, min_distance)

    def react(self, node):
        node.get_logger().info(f"Red light detected at x={self.position[0]:.2f}. Stopping...")
        node.speed = 0.0  # Stop vehicle
        return True

class YellowLight(Target):
    def __init__(self, id, label, threshold=0.3, len_history=10, min_distance=1.5):
        super().__init__(id, label, threshold, len_history, min_distance)

    def react(self, node):
        # Green light in range — resume driving
        node.get_logger().info(f"Yellow light detected at x={self.position[0]:.2f}. Reduce speed...")
        node.speed = node.mid_speed  # Reduce Speed
        return True

class GreenLight(Target):
    def __init__(self, id, label, threshold=0.3, len_history=10, min_distance=1.5):
        super().__init__(id, label, threshold, len_history, min_distance)

    def react(self, node):
        # Green light in range — resume driving
        node.get_logger().info(f"Green light detected at x={self.position[0]:.2f}. Resume driving...")
        node.speed = node.max_speed  # Resume driving
        return True