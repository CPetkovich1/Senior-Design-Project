import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class SnakeSearchBrain(Node):
    def __init__(self):
        super().__init__('snake_search_brain')
        self.bridge = CvBridge()
        
        # 1. Pub/Sub: We talk to your GPIO node via 'robot_cmd'
        self.cmd_pub = self.create_publisher(String, 'robot_cmd', 10)
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # 2. Tracking & State
        self.prev_gray = None
        self.prev_points = None
        self.accumulated_shift = 0.0
        
        # 3. Calibration (Tweak these!)
        self.TURN_THRESHOLD = 580.0  # Pixel shift for a 90-degree turn
        self.STRAIGHT_TIME = 4.0      # How many seconds to drive forward
        self.SIDE_TIME = 1.5          # How many seconds to move to the next row
        
        # 4. State Machine: DRIVE_FWD -> TURN_1 -> DRIVE_SIDE -> TURN_2 -> (Repeat)
        self.states = ["DRIVE_FWD", "TURN_1", "DRIVE_SIDE", "TURN_2"]
        self.current_state_idx = 0
        self.start_time = time.time()
        self.turn_direction = "left" # Starts by turning left

        self.get_logger().info("Snake Search Brain Started. Using Optical Flow for turns.")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # OPTICAL FLOW MATH
        if self.prev_gray is not None and self.prev_points is not None:
            new_points, status, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, self.prev_points, None)
            if new_points is not None and status is not None:
                good_new = new_points[status == 1]
                good_old = self.prev_points[status == 1]
                if len(good_new) > 10:
                    dx = np.mean(good_new[:, 0] - good_old[:, 0])
                    # Only track shift if we are currently in a turning state
                    if "TURN" in self.states[self.current_state_idx]:
                        self.accumulated_shift += dx

        self.run_logic()

        # UI & Point Refresh
        self.prev_gray = gray.copy()
        self.prev_points = cv2.goodFeaturesToTrack(gray, maxCorners=100, qualityLevel=0.3, minDistance=7)
        
        # Visual HUD
        cv_state = self.states[self.current_state_idx]
        cv2.putText(frame, f"TASK: {cv_state}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"SHIFT: {int(self.accumulated_shift)}", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow("Snake Search Camera", frame)
        cv2.waitKey(1)

    def send_cmd(self, cmd_str):
        msg = String()
        msg.data = cmd_str
        self.cmd_pub.publish(msg)

    def run_logic(self):
        state = self.states[self.current_state_idx]

        if state == "DRIVE_FWD":
            if time.time() - self.start_time < self.STRAIGHT_TIME:
                self.send_cmd("forward")
            else:
                self.next_state()

        elif state == "TURN_1" or state == "TURN_2":
            self.send_cmd(self.turn_direction)
            if abs(self.accumulated_shift) >= self.TURN_THRESHOLD:
                self.send_cmd("stop")
                # If we just finished TURN_2, reverse the direction for the next row
                if state == "TURN_2":
                    self.turn_direction = "right" if self.turn_direction == "left" else "left"
                self.next_state()

        elif state == "DRIVE_SIDE":
            if time.time() - self.start_time < self.SIDE_TIME:
                self.send_cmd("forward")
            else:
                self.next_state()

    def next_state(self):
        self.send_cmd("stop")
        self.current_state_idx = (self.current_state_idx + 1) % len(self.states)
        self.start_time = time.time()
        self.accumulated_shift = 0.0
        self.get_logger().info(f"Transitioning to: {self.states[self.current_state_idx]}")

def main():
    rclpy.init()
    node = SnakeSearchBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
