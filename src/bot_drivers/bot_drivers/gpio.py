import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from gpiozero import OutputDevice

class GpioSubscriber(Node):
	def __init__(self):
		super().__init__('gpio_subscriber')

		self.subscription = self.create_subscription(String, 'robot_cmd', self.listener_callback, 10)

		self.get_logger().info("Pi subscriber start listening for input")

		self.declare_parameter('pin_fwd', 17)
		self.declare_parameter('pin_bwd', 27)
		self.declare_parameter('pin_left', 22)
		self.declare_parameter('pin_right', 23)

		self.pins = {
			'forward': OutputDevice(self.get_parameter('pin_fwd').value),
			'backward': OutputDevice(self.get_parameter('pin_bwd').value),
			'left': OutputDevice(self.get_parameter('pin_left').value),
			'right': OutputDevice(self.get_parameter('pin_right').value)
		}

		self.watchdog = self.create_timer(0.2, self.emergency_stop)
		self.get_logger().info("Driver ready with heartbeat watchdog.")

	def listener_callback(self, msg):
		self.watchdog.reset()

		command = msg.data.lower()

		for cmd, device in self.pins.items():
			if cmd == command:
				device.on()
			else:
				device.off()

		if command in self.pins:
			self.pins[command].on()
			self.get_logger().info(f"Executing: {command}")
		elif command == 'stop':
			self.get_logger().info("Robot Stopping")

	def emergency_stop(self):
		for device in self.pins.values():
			device.off()

	def destroy_node(self):
		for device in self.pins.values():
			device.off()
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
