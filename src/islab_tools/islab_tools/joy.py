#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pygame

def apply_deadzone(v: float, dz: float) -> float:
    """Clamp small joystick values to zero."""
    return 0.0 if abs(v) < dz else v

class JoyNode(Node):
    def __init__(self):
        super().__init__("bt_joy_node_windows")

        # --- Declare Parameters ---
        self.declare_parameter("pub.topic", "/joy")
        self.declare_parameter("pub.rate", 50.0)     # Hz
        self.declare_parameter("deadzone", 0.05)

        # --- Retrieve Parameters ---
        topic = self.get_parameter("pub.topic").get_parameter_value().string_value
        rate_hz = self.get_parameter("pub.rate").get_parameter_value().double_value
        self.deadzone = self.get_parameter("deadzone").get_parameter_value().double_value

        # --- ROS Publisher ---
        self.pub = self.create_publisher(Joy, topic, 10)
        self.timer = self.create_timer(1.0 / rate_hz, self.tick)
        self.get_logger().info(f"Publishing to topic: {topic} at {rate_hz:.1f} Hz")

        # --- Pygame Joystick Init ---
        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("No joystick detected. Pair your Bluetooth controller first.")

        self.js = pygame.joystick.Joystick(0)
        self.js.init()
        self.get_logger().info(f"Using joystick: {self.js.get_name()}")

    def tick(self):
        pygame.event.pump()

        # Axes
        axes = [apply_deadzone(self.js.get_axis(i), self.deadzone)
                for i in range(self.js.get_numaxes())]

        # Buttons
        btns = [self.js.get_button(i) for i in range(self.js.get_numbuttons())]

        # D-pad (hat)
        for i in range(self.js.get_numhats()):
            hx, hy = self.js.get_hat(i)
            axes += [float(hx), float(-hy)]

        # Publish Joy message
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = axes
        msg.buttons = btns
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
