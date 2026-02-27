import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import Servo
import sys

class GripperSubscriber(Node):
    def __init__(self):
        super().__init__('gripper_subscriber')
        
        # Subscribe to the 'robot_cmd' topic
        self.subscription = self.create_subscription(
            String, 
            'robot_cmd', 
            self.listener_callback, 
            10)
        
        # Declare the GPIO pin parameter
        self.declare_parameter('pin_servo', 18)
        servo_pin = self.get_parameter('pin_servo').value
        
        # Initialize the Servo
        try:
            self.gripper_servo = Servo(servo_pin)
            self.get_logger().info(f"Servo initialized on GPIO {servo_pin}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Servo: {e}")

        self.get_logger().info("Ready. Use 'o' to Open and 'x' to Close.")

    def listener_callback(self, msg):
        # Convert incoming message to lowercase to avoid matching errors
        command = msg.data.lower()

        if command == 'open':
            self.get_logger().info("Executing: OPEN")
            self.gripper_servo.min()  # Moves to -1 position (0 degrees)
        
        elif command == 'close':
            self.get_logger().info("Executing: CLOSE")
            self.gripper_servo.max()  # Moves to +1 position (180 degrees)

    def destroy_node(self):
        # Even without 'stop', we detach on exit for hardware safety
        self.get_logger().info("Node destroying: Releasing servo.")
        self.gripper_servo.detach()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GripperSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
