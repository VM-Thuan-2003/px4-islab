import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

from islab_msgs.msg import IslabDropBall

class DropBallPub(Node):
    def __init__(self):
        super().__init__('drop_ball_publisher')
        self.pub = self.create_publisher(IslabDropBall, '/islab/dropball', 10)
        self.timer = self.create_timer(1.0, self.publish_msg)  # 1 Hz

    def publish_msg(self):
        msg = IslabDropBall()
        msg.timestamp = self.get_clock().now().nanoseconds
        msg.timestamp_sample = msg.timestamp

        # Example data for 5 balls
        msg.ball_id = [1, 2, 3, 4, 5]
        msg.drop = [True, True, False, False, False]  # Drop the first ball

        self.pub.publish(msg)
        self.get_logger().info(f"Published: {msg}")

def main(args=None):
    rclpy.init(args=args)
    node = DropBallPub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
