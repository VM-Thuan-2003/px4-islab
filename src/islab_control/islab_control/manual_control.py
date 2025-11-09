#!/usr/bin/env python3
# ROS 2: sensor_msgs/Joy  →  px4_msgs/ManualControlSetpoint
import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Joy
from px4_msgs.msg import ManualControlSetpoint


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def deadzone(x: float, dz: float) -> float:
    return 0.0 if abs(x) < dz else x

def any_stick_moving(vals: List[float], eps: float) -> bool:
    return any(abs(v) > eps for v in vals)

class JoyToManual(Node):
    """
    Map joystick axes to PX4 manual control:
      roll     <- axes[AX_ROLL]     (right = +1)
      pitch    <- axes[AX_PITCH]    (forward = +1)  NOTE: PX4 expects forward as + (nose down)
      yaw      <- axes[AX_YAW]      (clockwise/top-down = +1)
      throttle <- axes[AX_THR]      (-1..+1, where -1 = 0% thrust, +1 = 100%)
    Buttons mapped into a uint16 bitmask.
    """

    def __init__(self):
        super().__init__('joy_to_manual')

        # QoS
        qos_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        qos_sub = qos_pub

        # Parameters (change to match your joystick)
        self.declare_parameter('AX_ROLL',   2)   # left/right stick
        self.declare_parameter('AX_PITCH',  3)   # forward/back stick
        self.declare_parameter('AX_YAW',    0)
        self.declare_parameter('AX_THR',    1)
        self.declare_parameter('deadzone',  0.05)
        self.declare_parameter('invert_roll',   False)
        self.declare_parameter('invert_pitch',  False)   # many gamepads: forward = -1, so invert to make forward positive
        self.declare_parameter('invert_yaw',    False)
        self.declare_parameter('invert_thr',    False)
        self.declare_parameter('throttle_mode', 'minus_one_to_one')
        # throttle_mode: 'minus_one_to_one' (stick already -1..+1)
        #                'zero_to_one'      (convert 0..1 to [-1..+1])
        #                'one_to_zero'      (some sliders are 1..0; convert then to [-1..+1])

        # SOURCE_* choice
        self.declare_parameter('data_source', int(ManualControlSetpoint.SOURCE_MAVLINK_0))  # 2
        self.declare_parameter('valid_when_buttons_only', True)  # publish valid even if only buttons are used

        self.pub = self.create_publisher(ManualControlSetpoint, '/fmu/in/manual_control_input', qos_pub)
        self.sub = self.create_subscription(Joy, '/joy', self.on_joy, qos_sub)

        self.get_logger().info('Publishing ManualControlSetpoint to /fmu/in/manual_control_input')

    # Helpers to read params quickly
    def P(self, name: str):
        return self.get_parameter(name).get_parameter_value()

    def on_joy(self, joy: Joy):
        # Read params
        ax_roll  = self.P('AX_ROLL').integer_value
        ax_pitch = self.P('AX_PITCH').integer_value
        ax_yaw   = self.P('AX_YAW').integer_value
        ax_thr   = self.P('AX_THR').integer_value
        dz       = float(self.P('deadzone').double_value or 0.0)

        inv_roll  = self.P('invert_roll').bool_value
        inv_pitch = self.P('invert_pitch').bool_value
        inv_yaw   = self.P('invert_yaw').bool_value
        inv_thr   = self.P('invert_thr').bool_value
        thr_mode  = self.P('throttle_mode').string_value or 'minus_one_to_one'
        data_src  = self.P('data_source').integer_value
        valid_btn = self.P('valid_when_buttons_only').bool_value

        # Defensive access
        def get_axis(i: int) -> float:
            return float(joy.axes[i]) if 0 <= i < len(joy.axes) else 0.0

        roll  = get_axis(ax_roll)
        pitch = get_axis(ax_pitch)
        yaw   = get_axis(ax_yaw)
        thr   = get_axis(ax_thr)

        # Apply deadzone
        roll  = deadzone(roll, dz)
        pitch = deadzone(pitch, dz)
        yaw   = deadzone(yaw, dz)
        thr   = deadzone(thr, dz)

        # Inversions
        if inv_roll:  roll  = -roll
        if inv_pitch: pitch = -pitch
        if inv_yaw:   yaw   = -yaw
        if inv_thr:   thr   = -thr

        # Throttle normalization
        # PX4 expects throttle in [-1, +1] (then maps to 0..100%)
        if thr_mode == 'zero_to_one':
            # Input 0..1 → map to [-1..+1]
            thr = clamp(thr, 0.0, 1.0)
            thr = thr * 2.0 - 1.0
        elif thr_mode == 'one_to_zero':
            # Input 1..0 (reverse slider) → first flip to 0..1 then map
            thr = 1.0 - clamp(thr, 0.0, 1.0)
            thr = thr * 2.0 - 1.0
        else:
            # Already -1..+1; clamp
            thr = clamp(thr, -1.0, 1.0)

        # Buttons → uint16 bitmask
        buttons = 0
        for i, b in enumerate(joy.buttons[:16]):
            if b:
                buttons |= (1 << i)

        # Sticks moving?
        sticks_vals = [roll, pitch, yaw, thr]
        # moving = any_stick_moving(sticks_vals, eps=0.02)
        moving = True
        any_btn = buttons != 0
        valid = moving or (valid_btn and any_btn)

        # Build message
        msg = ManualControlSetpoint()
        now_us = int(self.get_clock().now().nanoseconds / 1000)

        # Prefer Joy timestamp if provided; fall back to now
        if joy.header.stamp.sec != 0 or joy.header.stamp.nanosec != 0:
            ts_sample_us = int(joy.header.stamp.sec * 1e6 + joy.header.stamp.nanosec / 1e3)
        else:
            ts_sample_us = now_us

        msg.timestamp = now_us
        msg.timestamp_sample = ts_sample_us

        msg.valid = bool(valid)
        msg.data_source = int(data_src)

        # New-style fields
        msg.roll = float(clamp(roll,  -1.0, 1.0))
        msg.pitch = float(clamp(pitch, -1.0, 1.0))
        msg.yaw = float(clamp(yaw,   -1.0, 1.0))
        msg.throttle = float(clamp(thr, -1.0, 1.0))

        # Optional extras (leave 0 if unused)
        msg.flaps = 0.0
        msg.aux1 = 0.0
        msg.aux2 = 0.0
        msg.aux3 = 0.0
        msg.aux4 = 0.0
        msg.aux5 = 0.0
        msg.aux6 = 0.0

        msg.sticks_moving = bool(moving)
        msg.buttons = int(buttons)

        # (Not using deprecated x/y/z/r fields)

        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(JoyToManual())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
