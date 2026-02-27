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

        self.get_logger().info("Ready. 'o'=Open, 'x'=Close. Release keys to Stop/Relax.")

    def listener_callback(self, msg):
        command = msg.data.lower()

        if command == 'open':
            self.get_logger().info("Status: OPENING")
            self.gripper_servo.min()  # Moves to -1 position
        
        elif command == 'close':
            self.get_logger().info("Status: CLOSING")
            self.gripper_servo.max()  # Moves to +1 position
            
        elif command == 'stop':
            # This 'detaches' the PWM signal. 
            # The servo stays where it is but stops drawing 'holding' power.
            self.gripper_servo.detach()

    def destroy_node(self):
        # Final safety cleanup
        self.gripper_servo.detach()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GpioSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
