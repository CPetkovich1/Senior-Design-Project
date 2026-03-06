import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage  # Faster than raw 'Image'
import cv2
import numpy as np

class CameraStreamer(Node):
    def __init__(self):
        super().__init__('camera_streamer')
        
        # 1. Topic: We use /compressed to significantly lower network lag
        self.publisher_ = self.create_publisher(CompressedImage, 'raw_image/compressed', 10)
        
        # 2. Timer: 0.033s = ~30 FPS
        self.timer = self.create_timer(0.033, self.timer_callback)
        
        # 3. Hardware Setup
        self.cap = cv2.VideoCapture(0)
        
        # PRINCIPLE: Hardware Resizing (Less data to process = faster FPS)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        
        self.get_logger().info('Pi 5 Streamer started at 320x240 (Compressed).')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # 4. PRINCIPLE: JPEG Compression
            # We encode the image as a '.jpg' to shrink it by ~90%
            # [quality, 80] balances image clarity for YOLO vs speed
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            
            # Encode and convert to byte array
            success, encoded_image = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            
            if success:
                msg.data = np.array(encoded_image).tobytes()
                self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraStreamer()
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
