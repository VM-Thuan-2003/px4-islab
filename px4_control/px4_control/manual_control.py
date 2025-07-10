import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import json
from geometry_msgs.msg import Twist

class ManualControl(Node):
    def __init__(self):
        super().__init__('manual_control')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.pub = self.create_publisher(Twist, '/offboard_velocity_cmd', qos_profile)

        self.arm_toggle = False
        self.arm_pub = self.create_publisher(Bool, '/arm_message', qos_profile)

        # Create subscriber to /event/keyboard topic
        self.subscription = self.create_subscription(
            String,
            '/event/keyboard',
            self.keyboard_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

    def keyboard_callback(self, msg):
        try:
            key_states = json.loads(msg.data)

            twist = Twist()
            twist.linear.z = 1.0 if key_states.get("W") else (-1.0 if key_states.get("S") else 0.0)
            twist.angular.z = 1.0 if key_states.get("D") else (-1.0 if key_states.get("A") else 0.0)

            twist.linear.x = 1.0 if key_states.get("J") else (-1.0 if key_states.get("L") else 0.0)  # Pitch
            twist.linear.y = 1.0 if key_states.get("I") else (-1.0 if key_states.get("K") else 0.0)  # Roll

            self.pub.publish(twist)

            # Takeoff (ARM)
            if key_states.get("SPACE") and not self.arm_toggle:
                self.arm_toggle = True
                self.arm_pub.publish(Bool(data=True))
                self.get_logger().info("Takeoff (ARM) command sent")

            # Land (DISARM)
            if key_states.get("R") and self.arm_toggle:
                self.arm_toggle = False
                self.arm_pub.publish(Bool(data=False))
                self.get_logger().info("Land (DISARM) command sent")

            # Reset toggle when released
            if not key_states.get("SPACE") and not key_states.get("R"):
                self.arm_toggle = False

        except json.JSONDecodeError as e:
            self.get_logger().error(f"JSON parse error: {e}")

def main(args=None):
    rclpy.init(args=args)

    manual_control = ManualControl()

    try:
        rclpy.spin(manual_control)
    except KeyboardInterrupt:
        manual_control.get_logger().info('Manual control node interrupted.')
    finally:
        # Destroy the node explicitly
        manual_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()