import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32  # Added Float32 for Ultrasonic
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class SnakeSearchBrain(Node):
    def __init__(self):
        super().__init__('snake_search_brain')
        self.bridge = CvBridge()
        
        # 1. Publishers & Subscribers
        self.cmd_pub = self.create_publisher(String, 'robot_cmd', 10)
        self.img_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        # NEW: Listen to the Ultrasonic Node
        self.range_sub = self.create_subscription(Float32, 'range_data', self.range_callback, 10)

        # 2. Tracking & State
        self.prev_gray = None
        self.prev_points = None
        self.accumulated_shift = 0.0
        self.obstacle_detected = False
        
        # 3. Calibration
        self.TURN_THRESHOLD = 580.0  
        self.STRAIGHT_TIME = 4.0      
        self.SIDE_TIME = 1.5          
        
        # 4. State Machine
        self.states = ["DRIVE_FWD", "TURN_1", "DRIVE_SIDE", "TURN_2"]
        self.current_state_idx = 0
        self.start_time = time.time()
        self.turn_direction = "left"

        self.get_logger().info("Snake Search Brain with Ultrasonic Safety Active.")

    def range_callback(self, msg):
        # NEW: Safety logic. If something is closer than 20cm, flag it.
        if msg.data < 0.20:
            if not self.obstacle_detected:
                self.get_logger().warning("OBSTACLE! Pausing Search.")
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Optical Flow Math (Only if moving)
        if self.prev_gray is not None and self.prev_points is not None:
            new_points, status, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, self.prev_points, None)
            if new_points is not None and status is not None:
                good_new = new_points[status == 1]
                good_old = self.prev_points[status == 1]
                if len(good_new) > 10:
                    dx = np.mean(good_new[:, 0] - good_old[:, 0])
                    if "TURN" in self.states[self.current_state_idx] and not self.obstacle_detected:
                        self.accumulated_shift += dx

        # RUN LOGIC
        if self.obstacle_detected:
            self.send_cmd("stop")
            # We freeze the start_time so the robot doesn't "lose time" while waiting
            self.start_time += (1.0 / 30.0) # Assume 30fps; adjust timer offset
        else:
            self.run_logic()

        self.prev_gray = gray.copy()
        self.prev_points = cv2.goodFeaturesToTrack(gray, maxCorners=100, qualityLevel=0.3, minDistance=7)
        
        # HUD UI
        status_color = (0, 0, 255) if self.obstacle_detected else (0, 255, 0)
        cv_state = "PAUSED - OBSTACLE" if self.obstacle_detected else self.states[self.current_state_idx]
        
        cv2.putText(frame, f"TASK: {cv_state}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)
        cv2.putText(frame, f"SHIFT: {int(self.accumulated_shift)}", (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
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
        self.get_logger().info(f"Moving to: {self.states[self.current_state_idx]}")

def main():
    rclpy.init()
    node = SnakeSearchBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
