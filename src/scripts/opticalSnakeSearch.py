import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class SnakeSearchNode(Node):
    def __init__(self):
        super().__init__('snake_search_node')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # --- Optical Flow Vars ---
        self.prev_gray = None
        self.prev_points = None
        self.accumulated_shift = 0.0
        
        # --- Calibration (Adjust these!) ---
        self.TURN_THRESHOLD = 600.0  # The "90 degree" pixel shift
        self.ROW_TIME = 5.0          # Seconds to drive forward per row
        
        # --- State Machine ---
        self.state = "DRIVE_FORWARD"
        self.start_time = time.time()
        self.turn_direction = 1 # 1 for Left, -1 for Right

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 1. Optical Flow Processing
        shift_in_this_frame = 0
        if self.prev_gray is not None:
            new_points, status, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, self.prev_points, None)
            if new_points is not None and status is not None:
                good_new = new_points[status == 1]
                good_old = self.prev_points[status == 1]
                if len(good_new) > 10:
                    shift_in_this_frame = np.mean(good_new[:, 0] - good_old[:, 0])
                    self.accumulated_shift += shift_in_this_frame

        # 2. State Machine Logic
        self.run_snake_logic()

        # 3. Cleanup & UI
        self.prev_gray = gray.copy()
        self.prev_points = cv2.goodFeaturesToTrack(gray, maxCorners=100, qualityLevel=0.3, minDistance=7)
        
        # Overlay info
        cv2.putText(frame, f"State: {self.state}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Shift: {int(self.accumulated_shift)}", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow("Snake Search View", frame)
        cv2.waitKey(1)

    def run_snake_logic(self):
        move = Twist()

        if self.state == "DRIVE_FORWARD":
            if time.time() - self.start_time < self.ROW_TIME:
                move.linear.x = 0.2
            else:
                self.state = "PREP_TURN"
                self.accumulated_shift = 0.0

        elif self.state == "PREP_TURN":
            # Small pause to stabilize before turning
            self.state = "TURN_90"

        elif self.state == "TURN_90":
            # Rotate based on turn_direction
            move.angular.z = 0.4 * self.turn_direction
            
            # CHECK OPTICAL FLOW THRESHOLD
            if abs(self.accumulated_shift) >= self.TURN_THRESHOLD:
                self.pub.publish(Twist()) # Stop
                self.get_logger().info("90 Degree Turn Complete")
                self.state = "DRIVE_FORWARD" # Or DRIVE_SIDE for a full lawnmower
                self.start_time = time.time()
                self.accumulated_shift = 0.0
                # Reverse turn direction for next time? 
                # self.turn_direction *= -1 

        self.pub.publish(move)

def main():
    rclpy.init()
    node = SnakeSearchNode()
    rclpy.spin(node)
    rclpy.shutdown()
