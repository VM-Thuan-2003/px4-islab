import rclpy
from rclpy.node import Node

class IslabDropball(Node):
    def __init__(self):
        super().__init__("islab_drop_ball_node")
        self.get_logger().info("Hello ROS 2!")

def main():
    rclpy.init()
    node = None
    try:
        node = IslabDropball()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info("KeyboardInterrupt, shutting down node")
    except Exception as e:
        if node is not None:
            node.get_logger().error(f"Exception: {e}")
        else:
            print(f"Exception before node started: {e}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
