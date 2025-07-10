#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleLocalPosition

import math
from scipy.spatial.transform import Rotation as R


class PX4StateReader(Node):
    def __init__(self):
        super().__init__('px4_state_reader')

        # Create QoS Profile compatible with PX4 (Best Effort)
        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        # PX4 mode mapping from NAV_STATE enum
        self.nav_state_dict = {
            0: 'Manual',
            1: 'Altitude',
            2: 'Position',
            3: 'Mission',
            4: 'Hold',
            5: 'Return',
            6: 'Offboard',
            7: 'Stabilized',
            8: 'Acro',
            9: 'Rattitude',
            10: 'Takeoff',
            11: 'Land',
            12: 'Follow Target',
            13: 'Precision Land',
            14: 'Orbit',
            15: 'Descend',
            16: 'Terminate',
            17: 'Takeoff',
            18: 'Landing',
        }

        # Subscribe to PX4 topics
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos)
        self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_callback, qos)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_position_callback, qos)

        self.get_logger().info("PX4 State Reader Node Initialized")

    def status_callback(self, msg: VehicleStatus):
        # Read arming and navigation state
        armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        nav_state = msg.nav_state
        nav_mode = self.nav_state_dict.get(nav_state, f"Unknown ({nav_state})")

        # Print readable status
        self.get_logger().info(f"[Status] Armed: {armed}, Mode: {nav_mode}")

    def attitude_callback(self, msg: VehicleAttitude):
        # Convert quaternion to yaw (heading)
        q = [msg.q[0], msg.q[1], msg.q[2], msg.q[3]]
        rot = R.from_quat([q[1], q[2], q[3], q[0]])  # PX4 uses [w, x, y, z]
        euler = rot.as_euler('xyz', degrees=False)
        yaw = euler[2] * 180 / math.pi
        self.get_logger().info(f"[Attitude] Heading (Yaw): {yaw:.2f}°")

    def local_position_callback(self, msg: VehicleLocalPosition):
        # Read position and velocity in ENU frame
        x = msg.x
        y = msg.y
        z = -msg.z  # Convert NED to ENU
        vx = msg.vx
        vy = msg.vy
        vz = msg.vz
        self.get_logger().info(f"[Local Position] Position: ({x:.2f}, {y:.2f}, {z:.2f}) m | Velocity: ({vx:.2f}, {vy:.2f}, {vz:.2f}) m/s")


def main(args=None):
    rclpy.init(args=args)
    node = PX4StateReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
