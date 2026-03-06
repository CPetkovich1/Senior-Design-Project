import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from gpiozero import DistanceSensor
import time

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')

        # 1. Setup Pins (Adjust these to your actual wiring)
        # Echo is the output from sensor (5V to 3.3V divider recommended)
        # Trigger is the input to sensor
        self.declare_parameter('pin_trigger', 24)
        self.declare_parameter('pin_echo', 25)

        trig = self.get_parameter('pin_trigger').value
        echo = self.get_parameter('pin_echo').value

        try:
            self.sensor = DistanceSensor(echo=echo, trigger=trig, max_distance=2.0)
            self.get_logger().info(f"Ultrasonic Sensor initialized on Trig:{trig} Echo:{echo}")
        except Exception as e:
            self.get_logger().error(f"Failed to start sensor: {e}")

        # 2. Publisher for distance (in meters)
        self.publisher_ = self.create_publisher(Float32, 'range_data', 10)

        # 3. Timer to read sensor at 10Hz (every 0.1 seconds)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = Float32()
        # sensor.distance returns a value between 0 and 1 (as a ratio of max_distance)
        # but we want the actual distance in meters
        msg.data = self.sensor.distance * self.sensor.max_distance
        
        self.publisher_.publish(msg)

        # Log warning if something is closer than 20cm
        if msg.data < 0.2:
            self.get_logger().warning(f"Obstacle Detected! Distance: {msg.data:.2f}m")

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
