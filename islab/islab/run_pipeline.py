import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PipelineNode(Node):

    def __init__(self):
        super().__init__('pipeline_node')
        self.publisher_ = self.create_publisher(String, 'pipeline_topic', 10)
        self.subscription = self.create_subscription(
            String,
            'pipeline_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.timer = self.create_timer(1.0, self.timer_callback)

    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello from the pipeline'
        self.publisher_.publish(msg)
        self.get_logger().info('Published: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    pipeline_node = PipelineNode()
    rclpy.spin(pipeline_node)
    pipeline_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

