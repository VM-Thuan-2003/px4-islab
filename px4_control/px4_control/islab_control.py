import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from px4_msgs.msg import VehicleStatus, OffboardControlMode, VehicleCommand, TrajectorySetpoint, VehicleLocalPosition
from px4_msgs.msg import VehicleAttitude
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import math
import json
import numpy as np


class IslabControl(Node):
    def __init__(self):
        super().__init__('islab_control')

        self.enable_logging = False

        # Setup QoS
        self.qos_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.qos_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", self.qos_pub)
        self.publisher_vehicle_command = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", self.qos_pub)
        self.publisher_trajectory_setpoint = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", self.qos_pub)
        self.publisher_drop = self.create_publisher(String, '/drop_ball', 10)
        self.publisher_drop_state = self.create_publisher(String, '/drop_ball/state', 10)
        self.publisher_islab_status = self.create_publisher(String, '/event/islab_status', 10)

        # Subscribers
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.vehicle_status_callback, self.qos_sub)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.local_position_callback, self.qos_sub)
        self.create_subscription(Odometry, '/UAV/odom', self.odom_callback, 10)
        self.create_subscription(String, '/event/islab_control', self.islab_control_callback, 10)
        self.attitude_sub = self.create_subscription(VehicleAttitude,'/fmu/out/vehicle_attitude',self.attitude_callback,self.qos_pub)
        
        # Timer
        self.fps_send_control = 20  # 20 Hz
        self.create_timer(1 / self.fps_send_control, self.send_offboard_control_mode)

        # State variables
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.pre_arm_check = False
        self.local_position = None
        self.odom_x = self.odom_y = self.odom_z = 0.0

        self.drop_balls = {f'ball_{i}': False for i in range(1, 6)}

        self.trueYaw = 0.0

    def get_timestamp(self):
        return self.get_clock().now().nanoseconds // 1000

    def islab_control_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            
            control = data.get("control", "").lower()

            if control == "arm":
                # Arming
                if data.get("arm"):
                    if self.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                        self.arm()
                elif self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                    self.get_logger().info("Disarming UAV")
                    if self.arming_state != VehicleStatus.ARMING_STATE_DISARMED:
                        self.disarm()
            
            elif control == "mode":
                # Mode handling
                mode = data.get("mode", "").lower()
                if mode == "offboard":
                    self.change_mode(VehicleCommand.VEHICLE_CMD_NAV_GUIDED_MASTER)
                elif mode == "takeoff":
                    self.take_off()
                elif mode == "land":
                    self.land()

            elif control == "ball":
                # Ball drop
                for ball_id, drop in data.get("drop_ball", {}).items():
                    if drop:
                        self.drop_ball(ball_id)

            elif control == "velocity":
                # Velocity control
                vel = data.get("velocity", {})
                self.send_velocity_yaw(
                    vel.get("x", 0.0),
                    vel.get("y", 0.0),
                    vel.get("z", 0.0),
                    vel.get("yaw", 0.0)
                )

            if self.enable_logging:
                self.get_logger().info(f"[Control] {data}")
        except json.JSONDecodeError:
            self.get_logger().error("Invalid JSON in control message.")

    def drop_ball(self, ball_id: str):
        if self.drop_balls.get(ball_id, True):
            self.get_logger().warn(f"[Drop] Ball {ball_id} already dropped or invalid.")
            return

        self.drop_balls[ball_id] = True
        msg = String()
        msg.data = f"{ball_id},{self.odom_x:.2f},{self.odom_y:.2f},{(self.odom_z - 0.4):.2f}"
        self.publisher_drop.publish(msg)

        state_msg = String()
        state_msg.data = json.dumps(self.drop_balls)
        self.publisher_drop_state.publish(state_msg)

        if self.enable_logging:
            self.get_logger().info(f"[Drop] {msg.data}")

    def attitude_callback(self, msg):
        orientation_q = msg.q

        self.trueYaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
                        1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))

    def send_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.timestamp = self.get_timestamp()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.publisher_offboard_mode.publish(msg)

    def send_velocity_yaw(self, vx, vy, vz, yaw_deg):

        cos_yaw = np.cos(self.trueYaw)
        sin_yaw = np.sin(self.trueYaw)

        velocity_world_x = (vx * cos_yaw - vy * sin_yaw)
        velocity_world_y = (vx * sin_yaw + vy * cos_yaw)

        msg = TrajectorySetpoint()
        msg.timestamp = self.get_timestamp()
        msg.velocity = [velocity_world_x, velocity_world_y, vz]
        msg.yaw = float('nan')
        msg.yawspeed = yaw_deg
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        self.publisher_trajectory_setpoint.publish(msg)

        if self.enable_logging:
            self.get_logger().info(f"[Velocity] vx={vx}, vy={vy}, vz={vz}, yaw={yaw_deg:.1f}°")

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state
        self.pre_arm_check = msg.pre_flight_checks_pass

        status = {
            "nav_state": self.nav_state,
            "arming_state": self.arming_state,
            "pre_arm_check": self.pre_arm_check
        }

        if self.local_position:
            yaw_deg = math.degrees(self.local_position.heading) if self.local_position.heading is not None else 0.0

            status["local_position"] = {
                "x": round(self.local_position.x, 2),
                "y": round(self.local_position.y, 2),
                "z": round(self.local_position.z, 2),
                "vx": round(self.local_position.vx, 2),
                "vy": round(self.local_position.vy, 2),
                "vz": round(self.local_position.vz, 2),
                "yaw": round(yaw_deg, 2)
            }

        msg_status = String()
        msg_status.data = json.dumps(status)
        self.publisher_islab_status.publish(msg_status)
    
    def local_position_callback(self, msg: VehicleLocalPosition):
        self.local_position = msg

    def odom_callback(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_z = msg.pose.pose.position.z

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("ARM command sent.")

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info("DISARM command sent.")

    def take_off(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1=1.0, param7=1.0)
        self.get_logger().info("TAKEOFF command sent.")

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("LAND command sent.")

    def change_mode(self, mode: int):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=float(mode), param2=6.0
        )
        self.get_logger().info(f"MODE change command sent: mode={mode}")

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_timestamp()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.publisher_vehicle_command.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(IslabControl())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
