import rclpy
from rclpy.node import Node


class AutoControl(Node):
    def __init__(self):
        super().__init__('auto_control')
        self.get_logger().info('AutoControl node has been started.')
def main(args=None):
    rclpy.init(args=args)
    auto_control = AutoControl()
    try:
        rclpy.spin(auto_control)
    except KeyboardInterrupt:
        auto_control.destroy_node()
        rclpy.shutdown()
