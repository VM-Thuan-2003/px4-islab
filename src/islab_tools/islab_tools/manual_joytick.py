#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Keyboard -> sensor_msgs/msg/Joy (ROS 2 Humble)
- STEP=1.0: each tap snaps axis to +/-1
- Buttons are one-shot ticks and can be pressed in parallel (multiple keys same cycle)
"""

import sys
import termios
import tty
import select
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from sensor_msgs.msg import Joy

# --- Behavior constants ---
STEP   = 0.4    # snap to +/-1 per tap
DECAY  = 0.02   # smooth return to 0 when no key held
CENTER = 0.0

HELP = """
Keyboard Joy Controls
---------------------
Left stick (axes[0], axes[1]):  A/D (←/→), W/S (↑/↓)
Right stick (axes[2], axes[3]): J/L (←/→), I/K (↑/↓)

Buttons (one-shot, parallel OK):
  R,T,Y,U,I,O,P,[,],\\  -> buttons[0..9]
  1,2                   -> buttons[10], buttons[11]

SPACE: zero all axes + buttons
h:     print help
q:     quit

Publishing: /joy   (sensor_msgs/Joy)
"""

class KeyboardJoy(Node):
    def __init__(self):
        super().__init__("keyboard_joy")

        # --- Parameters ---
        self.declare_parameter("topic", "/joy")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("num_axes", 4)
        self.declare_parameter("num_buttons", 12)

        topic = self.get_parameter("topic").get_parameter_value().string_value
        rate_hz = float(self.get_parameter("rate_hz").value)
        self.num_axes = int(self.get_parameter("num_axes").value)
        self.num_buttons = int(self.get_parameter("num_buttons").value)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.pub = self.create_publisher(Joy, topic, qos)
        self.timer = self.create_timer(max(1e-3, 1.0 / rate_hz), self._on_timer)

        # --- State ---
        self.axes: List[float] = [CENTER] * self.num_axes
        self.buttons: List[int] = [0] * self.num_buttons
        self._tick_indices: List[int] = []

        # Direction flags
        self.key_state = {
            "left": False, "right": False, "up": False, "down": False,     # WASD
            "lleft": False, "lright": False, "lup": False, "ldown": False, # IJKL
        }

        # Map many keys -> button indices (one-shot). Parallel presses OK.
        self.button_map = {
            'r': 0, 't': 1, 'y': 2, 'u': 3, 'i': 4, 'o': 5, 'p': 6,
            '[': 7, ']': 8, '\\': 9,
            '1': 10, '2': 11,
        }

        # Terminal raw mode
        self._fd = sys.stdin.fileno()
        self._old_tattr = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)

        self.get_logger().info(HELP.strip())

    # --- Non-blocking key read ---
    def _kb_hit(self) -> bool:
        return select.select([sys.stdin], [], [], 0)[0] != []

    def _read_key(self) -> str | None:
        if not self._kb_hit():
            return None
        return sys.stdin.read(1)

    # --- Handle key press ---
    def _handle_key(self, ch: str):
        c = ch.lower()

        # Left stick (WASD)
        if c == "w": self.key_state["up"] = True
        if c == "s": self.key_state["down"] = True
        if c == "a": self.key_state["left"] = True
        if c == "d": self.key_state["right"] = True

        # Right stick (IJKL) — NOTE: 'i' also mapped to button[4] below, but here it's for axis.
        if c == "i": self.key_state["lup"] = True
        if c == "k": self.key_state["ldown"] = True
        if c == "j": self.key_state["lleft"] = True
        if c == "l": self.key_state["lright"] = True

        # One-shot buttons (allow multiple in same cycle)
        if c in self.button_map:
            self._press_button(self.button_map[c])

        if ch == " ":
            self._reset_all()
        elif c == "h":
            print(HELP)
        elif c == "q":
            self.get_logger().info("Quit.")
            rclpy.shutdown()

    def _press_button(self, idx: int):
        if 0 <= idx < self.num_buttons:
            self.buttons[idx] = 1
            # keep track to clear after publish (one-shot)
            if idx not in self._tick_indices:
                self._tick_indices.append(idx)

    def _clear_ticks(self):
        for idx in self._tick_indices:
            if 0 <= idx < self.num_buttons:
                self.buttons[idx] = 0
        self._tick_indices.clear()

    def _release_dirs_each_cycle(self):
        for k in self.key_state:
            self.key_state[k] = False

    def _reset_all(self):
        for i in range(self.num_axes):
            self.axes[i] = CENTER
        for i in range(self.num_buttons):
            self.buttons[i] = 0
        self._tick_indices.clear()
        self._release_dirs_each_cycle()

    # --- Axis update logic ---
    def _decay_to_center(self, v: float) -> float:
        """Move axis smoothly toward 0.0"""
        if v > CENTER:
            return max(CENTER, v - DECAY)
        elif v < CENTER:
            return min(CENTER, v + DECAY)
        return v

    def _on_timer(self):
        # Drain all available keypresses this cycle (enables parallel button ticks)
        while True:
            k = self._read_key()
            if k is None:
                break
            self._handle_key(k)

        # --- Left stick (WASD) ---
        if self.key_state["right"] and not self.key_state["left"]:
            self.axes[0] = min(1.0, self.axes[0] + STEP)
        elif self.key_state["left"] and not self.key_state["right"]:
            self.axes[0] = max(-1.0, self.axes[0] - STEP)
        else:
            self.axes[0] = self._decay_to_center(self.axes[0])

        if self.key_state["up"] and not self.key_state["down"]:
            self.axes[1] = min(1.0, self.axes[1] + STEP)
        elif self.key_state["down"] and not self.key_state["up"]:
            self.axes[1] = max(-1.0, self.axes[1] - STEP)
        else:
            self.axes[1] = self._decay_to_center(self.axes[1])

        # --- Right stick (IJKL) ---
        if self.key_state["lright"] and not self.key_state["lleft"]:
            self.axes[2] = min(1.0, self.axes[2] + STEP)
        elif self.key_state["lleft"] and not self.key_state["lright"]:
            self.axes[2] = max(-1.0, self.axes[2] - STEP)
        else:
            self.axes[2] = self._decay_to_center(self.axes[2])

        if self.key_state["lup"] and not self.key_state["ldown"]:
            self.axes[3] = min(1.0, self.axes[3] + STEP)
        elif self.key_state["ldown"] and not self.key_state["lup"]:
            self.axes[3] = max(-1.0, self.axes[3] - STEP)
        else:
            self.axes[3] = self._decay_to_center(self.axes[3])

        # --- Publish Joy ---
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = list(self.axes)
        msg.buttons = list(self.buttons)
        self.pub.publish(msg)

        # Clear one-shot ticks (but after publish → supports many pressed in parallel)
        self._clear_ticks()
        self._release_dirs_each_cycle()

    def destroy_node(self):
        try:
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_tattr)
        except Exception:
            pass
        return super().destroy_node()


def main():
    rclpy.init()
    node = KeyboardJoy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
