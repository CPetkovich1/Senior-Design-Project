import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, LaserScan
import math

class RangeToScan(Node):
    def __init__(self):
        super().__init__('range_to_scan')
        self.sub = self.create_subscription(Range, '/ultrasonic', self.callback, 10)
        self.pub = self.create_publisher(LaserScan, '/scan', 10)

    def callback(self, msg):
        scan = LaserScan()
        scan.header = msg.header
        scan.angle_min = -msg.field_of_view / 2
        scan.angle_max = msg.field_of_view / 2
        scan.angle_increment = msg.field_of_view
        scan.range_min = msg.min_range
        scan.range_max = msg.max_range
        # Fill the scan with the single reading from the ultrasonic
        scan.ranges = [msg.range]
        self.pub.publish(scan)

def main():
    rclpy.init()
    rclpy.spin(RangeToScan())
    rclpy.shutdown()
