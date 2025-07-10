import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Pieline(Node):

    def __init__(self):
        super().__init__('pieline')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.get_clock().now().nanoseconds
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    pieline = Pieline()

    try:
        rclpy.spin(pieline)
    except KeyboardInterrupt:
        pieline.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        pieline.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
