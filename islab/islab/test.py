import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import DeleteEntity
import time

class BallDropper(Node):
    def __init__(self):
        super().__init__('ball_dropper')
        self.cli = self.create_client(DeleteEntity, '/delete_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for delete_entity service...')
        self.balls = [f"islab_balls::ball_joint_{i}" for i in range(1, 6)]
        self.index = 0
        self.timer = self.create_timer(2.0, self.drop_next_ball)

    def drop_next_ball(self):
        if self.index >= len(self.balls):
            self.get_logger().info("All balls dropped.")
            self.destroy_timer(self.timer)
            return
        req = DeleteEntity.Request()
        req.name = self.balls[self.index]
        self.get_logger().info(f"Dropping: {req.name}")
        self.cli.call_async(req)
        self.index += 1

def main(args=None):
    rclpy.init(args=args)
    node = BallDropper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()