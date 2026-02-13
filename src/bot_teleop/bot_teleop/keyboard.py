import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import select
import tty
import termios

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(String, 'robot_cmd', 10)
        
        # Save original terminal settings
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.current_key = None
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Create a second timer to poll the keyboard frequently
        self.input_timer = self.create_timer(0.01, self.poll_keyboard)
        self.get_logger().info("WSL2 Keyboard Node Started. Use WASD to move. ESC to quit.")

    def get_key(self):
        # Set terminal to raw mode to capture single keypresses
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = None
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def poll_keyboard(self):
        key = self.get_key()
        if key:
            if key in ['w', 'a', 's', 'd']:
                self.current_key = key
            elif key == '\x1b':  # ESC key
                self.get_logger().info("Shutting down...")
                exit()
        else:
            # If no key is being pressed in this poll, we stop
            # Note: This makes it "pulse". For "hold to move", 
            # you might need a small timeout logic.
            self.current_key = None

    def timer_callback(self):
        msg = String()
        mapping = {'w': 'forward', 's': 'backward', 'a': 'left', 'd': 'right'}
        msg.data = mapping.get(self.current_key, 'stop')
        self.publisher_.publish(msg)

# ... main function remains the same ...
def main(args=None):
	rclpy.init(args=args)
	node = KeyboardPublisher()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	node.destroy_node()
	rclpy.shutdown()
