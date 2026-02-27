import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import Servo
import sys

class GpioSubscriber(Node):
    def __init__(self):
        super().__init__('gpio_subscriber')
        
        # Subscribe to the 'robot_cmd' topic
        self.subscription = self.create_subscription(
            String, 
            'robot_cmd', 
            self.listener_callback, 
            10)
        
        # Declare the GPIO pin (Default 18)
        self.declare_parameter('pin_servo', 18)
        servo_pin = self.get_parameter('pin_servo').value
        
        # Initialize the Servo
        try:
            self.gripper_servo = Servo(servo_pin)
            self.get_logger().info(f"Servo initialized on GPIO {servo_pin}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Servo: {e}")

        self.get_logger().info("Ready. 'o' = Open, 'x' = Close. (Holding position enabled)")

    def listener_callback(self, msg):
        # Convert incoming message to lowercase
        command = msg.data.lower()

        if command == 'open':
            self.get_logger().info("Action: OPEN (Holding...)")
            self.gripper_servo.min()  # Moves to -1 position
        
        elif command == 'close':
            self.get_logger().info("Action: CLOSE (Holding...)")
            self.gripper_servo.max()  # Moves to +1 position
        
        # Note: 'stop' commands from the publisher are now ignored here

    def destroy_node(self):
        # Safety: The only time the servo relaxes is when the script stops
        self.get_logger().info("Shutting down: Releasing servo torque.")
        self.gripper_servo.detach()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GpioSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
