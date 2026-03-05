import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import OutputDevice
import cv2
import numpy as np

class SmartGpioSubscriber(Node):
    def __init__(self):
        super().__init__('smart_gpio_subscriber')

        # ROS2 Subscription
        self.subscription = self.create_subscription(String, 'robot_cmd', self.listener_callback, 10)
        self.get_logger().info("Smart Driver Ready with Camera Feedback")

        # GPIO Pins (Based on your parameters)
        self.pins = {
            'forward':  OutputDevice(17),
            'backward': OutputDevice(27),
            'left':     OutputDevice(22),
            'right':    OutputDevice(23),
            'found':    OutputDevice(24)  # Pin 32 on Arduino to unlock manual mode
        }

        # --- CAMERA / OPTICAL FLOW SETTINGS ---
        self.TURN_THRESHOLD = 500  # Adjust this via testing for 90 degrees
        self.cap = cv2.VideoCapture(0)
        self.feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)
        self.lk_params = dict(winSize=(15, 15), maxLevel=2, 
                              criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    def perform_precision_turn(self, direction):
        """Blocks and turns until the camera sees 90 degrees of movement."""
        self.get_logger().info(f"Starting precision {direction} turn...")
        
        # Tell Arduino we are taking control
        self.pins['found'].on()
        
        # Initialize OpenCV tracking
        ret, old_frame = self.cap.read()
        if not ret: return
        old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
        p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **self.feature_params)

        total_rotation = 0
        self.pins[direction].on()  # Set turn pin HIGH

        while total_rotation < self.TURN_THRESHOLD:
            ret, frame = self.cap.read()
            if not ret: break
            
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **self.lk_params)

            if p1 is not None:
                good_new = p1[st == 1]
                good_old = p0[st == 1]
                if len(good_new) > 0:
                    dx = np.mean(good_new[:, 0] - good_old[:, 0])
                    total_rotation += abs(dx)
                    
                    old_gray = frame_gray.copy()
                    p0 = good_new.reshape(-1, 1, 2)
                    
                if len(p0) < 10: # Refresh features if lost
                    p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **self.feature_params)

        # Stop turning
        self.pins[direction].off()
        self.pins['found'].off()
        self.get_logger().info(f"Precision turn finished. Total Flow: {total_rotation}")

    def listener_callback(self, msg):
        command = msg.data.lower()

        # If it's a turn command, use the precision logic
        if command in ['left', 'right']:
            self.perform_precision_turn(command)
        
        # Standard WASD logic for forward/backward/stop
        else:
            if command != 'stop':
                self.pins['found'].on()
            else:
                self.pins['found'].off()

            for cmd, device in self.pins.items():
                if cmd == command:
                    device.on()
                elif cmd != 'found': # Don't toggle 'found' here
                    device.off()

    def destroy_node(self):
        self.cap.release()
        for device in self.pins.values():
            device.off()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SmartGpioSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
