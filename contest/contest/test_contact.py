import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

class RingCollisionDetector(Node):
    def __init__(self):
        super().__init__('ring_collision_detector')

        # Vị trí trung tâm ring
        self.ring_center = (5.0, 3.0, 1.5)
        self.ring_radius = 0.5
        self.tolerance = 0.1  # ± sai số cho phép

        self.subscription = self.create_subscription(
            Odometry,
            '/UAV/odom',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        drone_x = msg.pose.pose.position.x
        drone_y = msg.pose.pose.position.y
        drone_z = msg.pose.pose.position.z

        dx = drone_x - self.ring_center[0]
        dy = drone_y - self.ring_center[1]
        dz = drone_z - self.ring_center[2]

        horizontal_dist = math.sqrt(dx**2 + dy**2)
        vertical_diff = abs(dz)
        print(abs(horizontal_dist - self.ring_radius), vertical_diff)
        if (abs(horizontal_dist - self.ring_radius) < self.tolerance and
            vertical_diff < self.tolerance):
            self.get_logger().info(
                f"🚨 Drone is touching the ring at ({drone_x:.2f}, {drone_y:.2f}, {drone_z:.2f})"
            )

def main(args=None):
    rclpy.init(args=args)
    node = RingCollisionDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
