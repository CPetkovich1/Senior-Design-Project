import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class Camera(Node):
    def __init__(self):
        super().__init__('camera')
        # We publish to 'raw_image' topic
        self.publisher_ = self.create_publisher(Image, 'raw_image', 10)
        
        # Pi 5 handles higher frame rates well; let's aim for 30 FPS
        self.timer = self.create_timer(0.033, self.timer_callback)
        
        self.cap = cv2.VideoCapture(0) # 0 is usually the Pi Cam or USB cam
        self.bridge = CvBridge()
        self.get_logger().info('Pi 5 Camera Streamer has started.')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert OpenCV BGR image to ROS Image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Camera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
