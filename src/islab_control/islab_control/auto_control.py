import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from islab_msgs.msg import IslabControl, IslabChangeMode, IslabDropBall, StatusDropBall, IslabFlag
from px4_msgs.msg import VehicleControlMode, VehicleLocalPosition, VehicleStatus
import numpy as np
import math
import cv2
from time import time

class PIDController:
    def __init__(self, kp, ki, kd, dt, output_limits=(None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.previous_error = 0.0
        self.min_output, self.max_output = output_limits

    def compute(self, setpoint, measured):
        error = setpoint - measured
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Clamp output
        if self.max_output is not None:
            output = min(output, self.max_output)
        if self.min_output is not None:
            output = max(output, self.min_output)

        self.previous_error = error
        return output

class IslabAutoControl(Node):
    def __init__(self):
        super().__init__("islab_auto_node")
        self.get_logger().info("Islab Auto Node")
        
        self.bridge = CvBridge()
        
        self.qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher
        self.ball_publisher = self.create_publisher(IslabDropBall, '/islab/dropball', 10)
        self.velocity_publisher = self.create_publisher(IslabControl, 'islab/velocity', self.qos_profile_pub)
        self.change_mode_publisher = self.create_publisher(IslabChangeMode, 'islab/change_mode', self.qos_profile_pub)

        # Subscribers
        self.create_subscription(StatusDropBall, '/islab/status_dropball', self.drop_status_callback, 10)
        self.create_subscription(IslabFlag, '/islab/flag_mode', self.flag_mode_callback, self.qos_profile_sub)
        self.create_subscription(Image, '/islab/down_camera/image_raw', self.bottom_camera_callback, 10)
        self.create_subscription(Image, '/islab/forward_camera/image_raw', self.forward_camera_callback, 10)
        self.create_subscription(VehicleControlMode, '/fmu/out/vehicle_control_mode', self.status_callback, self.qos_profile_sub)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.status_vehicle_callback, self.qos_profile_sub)
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_mode_callback, self.qos_profile_sub)
        
        self.is_auto_mode = False
        self.is_start_auto = False
        self.is_stop_auto = False

        self.count_check = 0
        self.landed = False

        self.frame_down = None
        self.frame_forward = None

        self.ball_ids = [1, 2, 3, 4, 5]
        self.ball_dropped = [False, False, False, False, False]
        
        self.local_position = {"x": 0.0, "y": 0.0, "z": 0.0, "vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw": 0.0}
        self.drop_flags = {f"ball_{i}": False for i in range(1, 6)}
        self.velocity_cmd = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
        
        self.yaw_target = -90
        self.nav_state = None
        self.armed = False
        self.action_onboard = False
        self.take_off_done = False
        self.altitude_target = 2.0
        self.fps = 200.0 
        
        self.status_stage = {
            "start":{
                "time": 0.0,
                "status": False,
            },
            "yaw": {
                "1": {
                    "time": 0.0,
                    "status": False,
                    "count": 0,
                },
                "2": {
                    "time": 0.0,
                    "status": False,
                    "count": 0,
                },
                "3": {
                    "time": 0.0,
                    "status": False,
                    "count": 0,
                },
            },
            "forward": {
                "1": {
                    "time": 0.0,
                    "status": False,
                },
                "2": {
                    "time": 0.0,
                    "status": False,
                },
                "3": {
                    "time": 0.0,
                    "status": False,
                },
                "4": {
                    "time": 0.0,
                    "status": False,
                },
                "5": {
                    "time": 0.0,
                    "status": False,
                },
            },
            "left": {
                "1": {
                    "time": 0.0,
                    "status": False,
                },
                "2": {
                    "time": 0.0,
                    "status": False,
                },
                "3": {
                    "time": 0.0,
                    "status": False,
                },
                "4": {
                    "time": 0.0,
                    "status": False,
                },
                "5": {
                    "time": 0.0,
                    "status": False,
                },
            },
            "right": {
                "1": {
                    "time": 0.0,
                    "status": False,
                },
                "2": {
                    "time": 0.0,
                    "status": False,
                },
                "3": {
                    "time": 0.0,
                    "status": False,
                },
                "4": {
                    "time": 0.0,
                    "status": False,
                },
                "5": {
                    "time": 0.0,
                    "status": False,
                },
            },
            "land": {
                "time": 0.0,
                "status": False,
            },
            "drop_ball": {
                "1": { 
                    "time": 0.0,
                    "status": False
                },
                "2": { 
                    "time": 0.0,
                    "status": False
                },
                "3": { 
                    "time": 0.0,
                    "status": False
                },
                "4": { 
                    "time": 0.0,
                    "status": False
                },
                "5": { 
                    "time": 0.0,
                    "status": False
                }
            }
        }
        
        self.altitude_pid = PIDController(kp=1.5, ki=0.0, kd=0.8, dt=1.0 / self.fps, output_limits=(-1.0, 1.0))
        self.create_timer(1.0/self.fps, self.main)
    
    def send_velocity(self):
        msg = IslabControl()
        msg.vx = float(self.velocity_cmd["x"])
        msg.vy = float(self.velocity_cmd["y"])
        msg.vz = float(self.velocity_cmd["z"])
        msg.vyaw = float(self.velocity_cmd["yaw"])
        self.velocity_publisher.publish(msg)
    
    def send_change_mode(self, mode = 2, altitude = 2.0, arm = False, handel = 0, source = 0):
        msg = IslabChangeMode()
        msg.timestamp = self.get_clock().now().nanoseconds
        msg.timestamp_sample = self.get_clock().now().nanoseconds
        msg.mode = mode
        msg.altitude = altitude
        msg.arm = arm
        msg.handel = handel
        msg.source = source
        self.change_mode_publisher.publish(msg)
    
    def deg_to_rad_per_sec(self, deg_per_sec):
        return deg_per_sec * math.pi / 180
    
    def drop_ball(self, ball_id = 1):
        msg = IslabDropBall()
        msg.timestamp = self.get_clock().now().nanoseconds
        msg.timestamp_sample = msg.timestamp

        drop_ball = self.ball_dropped
        num_drop = ball_id - 1
        drop_ball[num_drop] = True
        
        msg.ball_id = self.ball_ids
        msg.drop = drop_ball

        self.ball_publisher.publish(msg)
    
    def take_off_control(self, altitude_target = 2.0):
        
        ############# arm ###################
        if self.armed is not True:
            self.send_change_mode(arm = True, handel = 2)
        
        ############# takeoff ###################
        if self.action_onboard is not True:
            self.send_change_mode(altitude = altitude_target, handel = 1)
        
        current_altitude = - self.local_position.z
        error_altitude = altitude_target - current_altitude
        
        if math.fabs(error_altitude) < 1.0 or current_altitude > altitude_target:
            for i in range(10):
                self.send_change_mode(mode=4, handel=0)
            self.status_stage["start"]["time"] = time()
            return True
        else:
            # self.count_check += 1
            # if self.count_check == 500:
            #     self.count_check = 0
            #     for i in range(10):
            #         self.send_change_mode(mode=4, handel=0)
            #     return True
            return False
    
    def process(self):
        if not self.take_off_done:
            self.take_off_done = self.take_off_control(altitude_target=self.altitude_target)
            return
        
        current_altitude = - self.local_position.z
        self.velocity_cmd["z"] = - self.altitude_pid.compute(self.altitude_target, current_altitude)
        
        if self.status_stage["start"]["status"] is False:
            curr_time = time()
            hold_time = 10  # seconds
            elapsed = curr_time - self.status_stage["start"]["time"]

            if elapsed < hold_time:
                pass
            else:
                self.status_stage["start"]["time"] = curr_time
                self.status_stage["start"]["status"] = True
            self.send_velocity()
            return
        elif self.status_stage["yaw"]["1"]["status"] is False:
            yaw_speed = 30
            self.yaw_target = 90
            error_yaw = self.yaw_target - math.degrees(self.local_position.heading)
            if error_yaw < 0:
                # rotate left
                self.velocity_cmd["yaw"] = -yaw_speed
            elif error_yaw > 0:
                # rotate right
                self.velocity_cmd["yaw"] = yaw_speed
            if math.fabs(error_yaw) < 4.0:
                self.status_stage["yaw"]["1"]["count"] += 1
                if self.status_stage["yaw"]["1"]["count"] > 10:
                    self.status_stage["yaw"]["1"]["time"] = time()
                    self.velocity_cmd["yaw"] = 0.0
                    self.status_stage["yaw"]["1"]["status"] = True
                    self.status_stage["forward"]["1"]["time"] = time()
            self.send_velocity()
            return
        elif self.status_stage["forward"]["1"]["status"] is False:
            curr_time = time()
            distance_target = 3.3  # meters
            velocity_forward = 0.5 # m/s
            if curr_time - self.status_stage["forward"]["1"]["time"] < distance_target / velocity_forward:
                self.velocity_cmd["x"] = - velocity_forward
                self.velocity_cmd["y"] = 0.0
            else:
                self.status_stage["forward"]["1"]["time"] = curr_time
                self.velocity_cmd["x"] = 0.0
                self.velocity_cmd["y"] = 0.0
                self.status_stage["forward"]["1"]["status"] = True
                self.status_stage["drop_ball"]["1"]["time"] = time()
            self.send_velocity()
            return
        elif self.status_stage["drop_ball"]["1"]["status"] is False:
            curr_time = time()
            hold_time = 10  # seconds
            elapsed = curr_time - self.status_stage["drop_ball"]["1"]["time"]

            if hold_time - 4 <= elapsed <= hold_time - 3:
                self.drop_ball(1)
            elif elapsed < hold_time:
                pass
            else:
                self.status_stage["drop_ball"]["1"]["time"] = curr_time
                self.status_stage["drop_ball"]["1"]["status"] = True
                self.status_stage["forward"]["2"]["time"] = curr_time
            self.send_velocity()
            return
        elif self.status_stage["forward"]["2"]["status"] is False:
            curr_time = time()
            distance_target = 5.8 # meters
            velocity_forward = 0.5 # m/s
            if curr_time - self.status_stage["forward"]["2"]["time"] < distance_target / velocity_forward:
                self.velocity_cmd["x"] = - velocity_forward
                self.velocity_cmd["y"] = 0.0
            else:
                self.status_stage["forward"]["2"]["time"] = curr_time
                self.velocity_cmd["x"] = 0.0
                self.velocity_cmd["y"] = 0.0
                self.status_stage["forward"]["2"]["status"] = True
                self.status_stage["drop_ball"]["2"]["time"] = time()
            self.send_velocity()
            return
        elif self.status_stage["drop_ball"]["2"]["status"] is False:
            curr_time = time()
            hold_time = 10  # seconds
            elapsed = curr_time - self.status_stage["drop_ball"]["2"]["time"]

            if hold_time - 4 <= elapsed <= hold_time - 3:
                self.drop_ball(2)
            elif elapsed < hold_time:
                pass
            else:
                self.status_stage["drop_ball"]["2"]["time"] = curr_time
                self.status_stage["drop_ball"]["2"]["status"] = True
                self.status_stage["left"]["1"]["time"] = time()
            self.send_velocity()
            return
        elif self.status_stage["left"]["1"]["status"] is False:
            curr_time = time()
            distance_target = 3.0 # meters
            velocity_left = 0.5 # m/s
            if curr_time - self.status_stage["left"]["1"]["time"] < distance_target / velocity_left:
                self.velocity_cmd["x"] = 0.0
                self.velocity_cmd["y"] = velocity_left
            else:
                self.status_stage["left"]["1"]["time"] = curr_time
                self.velocity_cmd["x"] = 0.0
                self.velocity_cmd["y"] = 0.0
                self.status_stage["left"]["1"]["status"] = True
                self.status_stage["drop_ball"]["3"]["time"] = time()
            self.send_velocity()
            return
        elif self.status_stage["drop_ball"]["3"]["status"] is False:
            curr_time = time()
            hold_time = 10  # seconds
            elapsed = curr_time - self.status_stage["drop_ball"]["3"]["time"]

            if hold_time - 4 <= elapsed <= hold_time - 3:
                self.drop_ball(3)
            elif elapsed < hold_time:
                pass
            else:
                self.status_stage["drop_ball"]["3"]["time"] = curr_time
                self.status_stage["drop_ball"]["3"]["status"] = True
                self.status_stage["right"]["1"]["time"] = time()
            self.send_velocity()
            return
        elif self.status_stage["right"]["1"]["status"] is False:
            curr_time = time()
            distance_target = 6.0 # meters
            velocity_right = 0.5 # m/s
            if curr_time - self.status_stage["right"]["1"]["time"] < distance_target / velocity_right:
                self.velocity_cmd["x"] = 0.0
                self.velocity_cmd["y"] = - velocity_right
            else:
                self.status_stage["right"]["1"]["time"] = curr_time
                self.velocity_cmd["x"] = 0.0
                self.velocity_cmd["y"] = 0.0
                self.status_stage["right"]["1"]["status"] = True
                self.status_stage["drop_ball"]["4"]["time"] = time()
            self.send_velocity()
            return
        elif self.status_stage["drop_ball"]["4"]["status"] is False:
            curr_time = time()
            hold_time = 10  # seconds
            elapsed = curr_time - self.status_stage["drop_ball"]["4"]["time"]

            if hold_time - 4 <= elapsed <= hold_time - 3:
                self.drop_ball(4)
            elif elapsed < hold_time:
                pass
            else:
                self.status_stage["drop_ball"]["4"]["time"] = curr_time
                self.status_stage["drop_ball"]["4"]["status"] = True
                self.status_stage["left"]["2"]["time"] = curr_time
            self.send_velocity()
            return
        elif self.status_stage["left"]["2"]["status"] is False:
            curr_time = time()
            distance_target = 3.0 # meters
            velocity_left = 0.5 # m/s
            if curr_time - self.status_stage["left"]["2"]["time"] < distance_target / velocity_left:
                self.velocity_cmd["x"] = 0.0
                self.velocity_cmd["y"] = velocity_left
            else:
                self.status_stage["left"]["2"]["time"] = curr_time
                self.velocity_cmd["x"] = 0.0
                self.velocity_cmd["y"] = 0.0
                self.status_stage["left"]["2"]["status"] = True
                self.status_stage["forward"]["3"]["time"] = time()
            self.send_velocity()
            return
        elif self.status_stage["forward"]["3"]["status"] is False:
            curr_time = time()
            distance_target = 5.0 # meters
            velocity_forward = 0.5 # m/s
            if curr_time - self.status_stage["forward"]["3"]["time"] < distance_target / velocity_forward:
                self.velocity_cmd["x"] = - velocity_forward
                self.velocity_cmd["y"] = 0.0
            else:
                self.status_stage["forward"]["3"]["time"] = curr_time
                self.velocity_cmd["x"] = 0.0
                self.velocity_cmd["y"] = 0.0
                self.status_stage["forward"]["3"]["status"] = True
                self.status_stage["drop_ball"]["5"]["time"] = time()
            self.send_velocity()
            return
        elif self.status_stage["drop_ball"]["5"]["status"] is False:
            curr_time = time()
            hold_time = 10  # seconds
            elapsed = curr_time - self.status_stage["drop_ball"]["5"]["time"]

            if hold_time - 4 <= elapsed <= hold_time - 3:
                self.drop_ball(5)
            elif elapsed < hold_time:
                pass
            else:
                self.status_stage["drop_ball"]["5"]["time"] = curr_time
                self.status_stage["drop_ball"]["5"]["status"] = True
                self.status_stage["forward"]["4"]["time"] = curr_time
            self.send_velocity()
            return
        elif self.status_stage["forward"]["4"]["status"] is False:
            curr_time = time()
            distance_target = 5.0 # meters
            velocity_forward = 0.5 # m/s
            if curr_time - self.status_stage["forward"]["4"]["time"] < distance_target / velocity_forward:
                self.velocity_cmd["x"] = - velocity_forward
                self.velocity_cmd["y"] = 0.0
            else:
                self.status_stage["forward"]["4"]["time"] = curr_time
                self.velocity_cmd["x"] = 0.0
                self.velocity_cmd["y"] = 0.0
                self.status_stage["forward"]["4"]["status"] = True
            self.send_velocity()
            return
        elif self.status_stage["land"]["status"] is False:
            if not self.landed:
                for i in range(50):
                    self.send_change_mode(mode=6, handel=0)
                self.landed = True
                self.stage_2_done = True
                self.status_stage["land"]["status"] = True
            return

    def main(self):
        # if self.nav_state == 14 or self.nav_state == 17:
        if self.is_auto_mode and self.is_start_auto:
            self.process()

    def status_mode_callback(self, msg:VehicleStatus):
        try:
            self.nav_state = msg.nav_state
        except Exception as e:
            self.get_logger().error(f"[Status Mode] {e}") 
    
    def status_vehicle_callback(self, msg:VehicleLocalPosition):
        try:
            self.local_position = msg
        except Exception as e:
            self.get_logger().error(f"[Status Vehicle] {e}") 
    
    def status_callback(self, msg:VehicleControlMode):
        try:
            self.armed = msg.flag_armed
            self.action_onboard = msg.flag_control_auto_enabled
        except Exception as e:
            self.get_logger().error(f"[Status Vehicle Control] {e}") 
    
    def flag_mode_callback(self, msg:IslabFlag):
        try:
            self.is_auto_mode = msg.flag_auto
            self.is_start_auto = msg.flag_start_auto
            self.is_stop_auto = msg.flag_stop_auto
        except Exception as e:
            self.get_logger().error(f"[Flag Mode] {e}")

    def drop_status_callback(self, msg: StatusDropBall):
        try:
            self.ball_ids = msg.ball_id
            self.ball_dropped = msg.dropped
        except Exception as e:
            self.get_logger().error(f"[Status DropBall] {e}") 
    
    def bottom_camera_callback(self, msg: Image):
        try:
            self.frame_down = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = cv2.resize(self.frame_down, (0, 0), fx=0.5, fy=0.5)
            cv2.imshow("Bottom Camera", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"[Camera Bottom] {e}")

    def forward_camera_callback(self, msg: Image):
        try:
            self.frame_forward = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = cv2.resize(self.frame_forward, (0, 0), fx=0.5, fy=0.5)
            cv2.imshow("Forward Camera", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"[Camera Forward] {e}")
        
def main():
    rclpy.init()
    node = None
    try:
        node = IslabAutoControl()
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
