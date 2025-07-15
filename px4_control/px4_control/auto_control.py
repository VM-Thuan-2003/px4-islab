import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleStatus, VehicleLocalPosition
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


class AutoControl(Node):
    def __init__(self):
        super().__init__('auto_control')
        self.get_logger().info('Auto control node started.')

        # === State ===
        self.mode = "Manual"
        self.armed = False
        self.nav_state = None
        self.arm_toggle = False
        self.ball_dropped = False

        # === Position from /UAV/odom ===
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_z = 0.0

        # === Movement timers ===
        self.forward_timer = 0
        self.forward_duration = 10 * 30  # 10 seconds @ 30Hz
        self.fps = 30

        # === QoS ===
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # === Publishers ===
        self.velocity_pub = self.create_publisher(Twist, '/offboard_velocity_cmd', qos_profile)
        self.arm_pub = self.create_publisher(Bool, '/arm_message', qos_profile)
        self.drop_pub = self.create_publisher(String, '/drop_ball', 10)

        # === Subscribers ===
        self.create_subscription(String, '/event/mode', self.mode_callback, 10)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_callback, qos_profile)
        self.create_subscription(Odometry, '/UAV/odom', self.odom_callback, 10)

        # === Timer loop ===
        self.create_timer(1.0 / self.fps, self.main_autopilot_callback)

    def mode_callback(self, msg: String):
        self.mode = "Auto" if msg.data == "Auto" else "Manual"
        self.get_logger().info(f"[Mode] Switched to: {self.mode}")

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.armed = msg.arming_state == 2
        self.nav_state = msg.nav_state

    def local_position_callback(self, msg: VehicleLocalPosition):
        self.altitude = msg.z

    def odom_callback(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_z = msg.pose.pose.position.z
        self.get_logger().info(f"[Odom] x={self.odom_x:.2f}, y={self.odom_y:.2f}, z={self.odom_z:.2f}")

    def drop_ball(self, ball_id: str):
        msg = String()
        msg.data = f"{ball_id},{self.odom_x:.2f},{self.odom_y:.2f},{(self.odom_z-0.4):.2f}"
        self.drop_pub.publish(msg)
        self.get_logger().info(f"[Drop] Sent: {msg.data}")

    def main_autopilot_callback(self):
        if self.mode != "Auto":
            return

        if not self.armed and not self.arm_toggle:
            self.get_logger().info("[ARM] Sending arm command...")
            self.arm_pub.publish(Bool(data=True))
            self.arm_toggle = True
            return

        if self.armed and self.nav_state == 14:  # OFFBOARD
            twist = Twist()

            if self.forward_timer < self.forward_duration:
                twist.linear.y = 0.5
                self.forward_timer += 1
                self.get_logger().info(f"[Action] Moving forward... {self.forward_timer / self.fps:.1f}s")
            else:
                if self.altitude > -0.1:
                    twist.linear.z = 0.0
                    if not self.ball_dropped:
                        self.drop_ball("ball_2")
                        self.ball_dropped = True
                else:
                    twist.linear.z = -0.5
                    self.get_logger().info(f"[Action] Descending... z = {self.altitude:.2f}")

            self.velocity_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = AutoControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("AutoControl node interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
