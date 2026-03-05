import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String # To publish logic commands
from cv_bridge import CvBridge
import sys
import os
# This tells the script to look into your virtual env site-packages
venv_path = os.path.expanduser('~/ultralytics-env/lib/python3.12/site-packages')
if venv_path not in sys.path:
    sys.path.append(venv_path)
from ultralytics import YOLO
import cv2

class Vision(Node):
    def __init__(self):
        super().__init__('vision')
        
        # Subscribe to the Pi's camera feed
        self.subscription = self.create_subscription(
            Image, 'raw_image', self.image_callback, 10)
        
        # Publisher for robot control logic (e.g., "STOP" or "GO")
        self.logic_pub = self.create_publisher(String, 'robot_commands', 10)
        
        # Load YOLO11 model (n = nano version for speed)
        self.model = YOLO('best.pt') 
        self.bridge = CvBridge()
        self.get_logger().info('YOLO11 Detector Node has started.')

    def image_callback(self, msg):
        # 1. Convert ROS image back to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 2. Run YOLO11 Inference
        results = self.model(frame, verbose=False, conf=0.5)

        # 3. Process Detections & Logic Control
        command = String()
        found_person = False

        for result in results:
            for box in result.boxes:
                class_id = int(box.cls[0])
                label = self.model.names[class_id]
                
                # Logic Control Example: Stop if a 'person' is detected
                if label == 'person':
                    found_person = True

        if found_person:
            command.data = "STOP: Person detected!"
            self.get_logger().warn("Logic Triggered: STOP")
        else:
            command.data = "PROCEED: Path clear"
        
        self.logic_pub.publish(command)

        # 4. Optional: Display the annotated frame
        annotated_frame = results[0].plot()
        cv2.imshow("YOLO11 Laptop Feed", annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = Vision()
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
