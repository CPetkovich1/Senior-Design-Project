import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import sys
import os

# Your virtual env path
venv_path = os.path.expanduser('~/ultralytics-env/lib/python3.12/site-packages')
if venv_path not in sys.path:
    sys.path.append(venv_path)

from ultralytics import YOLO
import cv2

class CubeDetector(Node): # Renamed for clarity
    def __init__(self):
        super().__init__('cube_detector')
        
        self.subscription = self.create_subscription(
            Image, 'raw_image', self.image_callback, 10)
        
        self.logic_pub = self.create_publisher(String, 'robot_commands', 10)
        
        # 1. Load your custom trained model
        # Ensure best.pt is in the same directory or provide the full path
        self.model = YOLO('best.pt') 
        self.bridge = CvBridge()
        
        # 2. Threshold for "Close Enough" (0.0 to 1.0)
        # If the cube takes up more than 40% of the frame width, hand over control
        self.close_threshold = 0.4 
        
        self.get_logger().info('Cube Detector Node started. Looking for "cube"... ')

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = frame.shape # Get frame dimensions

        # Run Inference
        results = self.model(frame, verbose=False, conf=0.6) # Increased confidence for reliability

        command = String()
        cube_detected = False
        close_enough = False

        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                label = self.model.names[class_id]
                
                # 3. Target the 'cube' label from your training
                if label.lower() == 'cube':
                    cube_detected = True
                    
                    # 4. Calculate Normalized Width
                    # box.xywhn[0] gives [x_center, y_center, width, height] in 0-1 range
                    box_width = box.xywhn[0][2] 
                    
                    if box_width > self.close_threshold:
                        close_enough = True

        # 5. Logic Branching
        if close_enough:
            command.data = "HANDOVER: Cube reached. Operator control required."
            self.get_logger().warn("TARGET REACHED: Switching to Operator")
        elif cube_detected:
            command.data = "APPROACH: Cube found, moving closer."
            self.get_logger().info("Cube sighted - Navigating...")
        else:
            command.data = "SEARCH: Scanning for cube..."

        self.logic_pub.publish(command)

        # 6. Debug view
        annotated_frame = results[0].plot()
        cv2.imshow("Robot Vision Feed", annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CubeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
