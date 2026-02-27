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
        
        # Initialize the Servo with extended pulse widths for a tighter "lock"
        # min_pulse_width and max_pulse_width (in seconds) help reach the full 180 degrees
        try:
            self.gripper_servo = Servo(
                servo_pin, 
                min_pulse_width=.5/1000, 
                max_pulse_width=2.5/1000
            )
            self.get_logger().info(f"Servo initialized on GPIO {servo_pin}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Servo: {e}")

        self.get_logger().info("Ready. Press 'o' to Open, 'x' to Lock Closed.")

    def listener_callback(self, msg):
        command = msg.data.lower()

        if command == 'open':
            self.get_logger().info("Executing: OPEN")
            self.gripper_servo.max()  # Moves to 0° and stays energized
        
        elif command == 'close':
            self.get_logger().info("Executing: CLOSE & LOCK")
            self.gripper_servo.min()  # Moves to 180° and stays energized (locked)
        
        # Any other commands (like 'stop' when keys are released) are ignored,
        # ensuring the servo holds its last position with full torque.

    def destroy_node(self):
        # The only time the lock is released is when the program is shut down
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
