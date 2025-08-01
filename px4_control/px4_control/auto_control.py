from time import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import json
import math
import numpy as np

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

class IslabControlUI(Node):
    def __init__(self):
        super().__init__('islab_control_ui')

        self.bridge = CvBridge()
        self.control_publisher = self.create_publisher(String, '/event/islab_control', 10)

        # Subscribers
        self.create_subscription(String, '/drop_ball/state', self.drop_status_callback, 10)
        self.create_subscription(String, '/event/islab_status', self.islab_status_callback, 10)
        self.create_subscription(Image, '/UAV/bottom/image_raw', self.bottom_camera_callback, 10)
        self.create_subscription(Image, '/UAV/forward/image_raw', self.forward_camera_callback, 10)

        # State
        self.is_arm = False
        self.control_msg = "none"
        self.flight_mode = "none"
        self.nav_state = None
        self.armed_state = None
        self.pre_arm_check = None

        self.drop_flags = {f"ball_{i}": False for i in range(1, 6)}
        self.velocity_cmd = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
        self.local_position = {"x": 0.0, "y": 0.0, "z": 0.0, "vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw": 0.0}

        self.altitude_target = 2.0  # Downward in NED frame
        self.yaw_target = 0.0

        self.fps = 30.0
        self.altitude_pid = PIDController(kp=1.5, ki=0.0, kd=0.8, dt=1.0 / self.fps, output_limits=(-1.0, 1.0))

        self.timer_offboard_mode = self.create_timer(1.0 / self.fps, self.main_control_loop)

        self.get_logger().info("IslabControlUI node started.")

        self.error_yellow = None
        self.detect_yellow = False
        self.count_yellow = 0

        self.error_home = None
        self.dir = False

        self.stage_1_done = False
        self.stage_2_done = False
        self.take_off_done = False

        self.time_check_30s = 0

        self.status_stage_1 = {
            "yaw": {
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
            },
            "land": {
                "time": 0.0,
                "status": False,
            },
        }

        self.status_stage_2 = {
            "yaw": {
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
            },
            "land": {
                "time": 0.0,
                "status": False,
            },
            "drop_ball": {
                "1": { 
                    "time": 0.0,
                    "status": False
                }
            }
        }

    def take_off(self, altitude_target=2.0):
        if self.nav_state != 14:
            self.select_mode("offboard")
            return False
        elif self.nav_state == 14:
            if self.armed_state == 1 and self.pre_arm_check:
                self.control_msg = "arm"
                self.is_arm = True
                self.send_control_message()
                return False
            elif self.armed_state == 2:
                self.control_msg = "velocity"
                current_altitude = - self.local_position.get("z", 0.0)
                error_altitude = altitude_target - current_altitude
                if math.fabs(error_altitude) < 0.4:
                    self.velocity_cmd["z"] = 0.0
                    self.send_control_message()
                    return True
                else:
                    self.velocity_cmd["z"] = - self.altitude_pid.compute(altitude_target, current_altitude)
                    self.send_control_message()
                    return False
            return False
        return False

    def drop_ball(self, ball_id: str):
        self.control_msg = "ball"
        self.drop_flags[ball_id] = True
        self.send_control_message()

    def deg_to_rad_per_sec(self, deg_per_sec):
        return deg_per_sec * math.pi / 180

    def stage_1(self):

        ###### Take off ######
        if not self.take_off_done:
            self.take_off_done = self.take_off(altitude_target=self.altitude_target)
            return
        
        self.control_msg = "velocity"

        ###### pid altitude control ######
        current_altitude = - self.local_position.get("z", 0.0)
        self.velocity_cmd["z"] = - self.altitude_pid.compute(self.altitude_target, current_altitude)
        
        if self.status_stage_1["yaw"]["1"]["status"] is False:
            yaw_speed = 6.0
            self.yaw_target = 72.0
            error_yaw = self.yaw_target - self.local_position.get("yaw", 0.0)
            if error_yaw < 0:
                # rotate left
                self.velocity_cmd["yaw"] = self.deg_to_rad_per_sec(-yaw_speed)
            elif error_yaw > 0:
                # rotate right
                self.velocity_cmd["yaw"] = self.deg_to_rad_per_sec(yaw_speed)
            if math.fabs(error_yaw) < 4.0:
                self.velocity_cmd["yaw"] = 0.0
                self.status_stage_1["yaw"]["1"]["status"] = True
                self.status_stage_1["forward"]["1"]["time"] = time()
            self.send_control_message()
            return
        
        elif self.status_stage_1["forward"]["1"]["status"] is False:
            curr_time = time()
            distance_target = 5.0  # meters
            velocity_forward = 0.5 # m/s
            if curr_time - self.status_stage_1["forward"]["1"]["time"] < distance_target / velocity_forward:
                self.velocity_cmd["x"] = - velocity_forward
                self.velocity_cmd["y"] = 0.0
            else:
                self.status_stage_1["forward"]["1"]["time"] = curr_time
                self.velocity_cmd["x"] = 0.0
                self.velocity_cmd["y"] = 0.0
                self.status_stage_1["forward"]["1"]["status"] = True
            self.send_control_message()
            return

        elif self.status_stage_1["yaw"]["2"]["status"] is False:
            yaw_speed = 6.0
            self.yaw_target = 115.0
            error_yaw = self.yaw_target - self.local_position.get("yaw", 0.0)
            if error_yaw < 0:
                # rotate left
                self.velocity_cmd["yaw"] = self.deg_to_rad_per_sec(-yaw_speed)
            elif error_yaw > 0:
                # rotate right
                self.velocity_cmd["yaw"] = self.deg_to_rad_per_sec(yaw_speed)
            if math.fabs(error_yaw) < 4.0:
                self.velocity_cmd["yaw"] = 0.0
                self.status_stage_1["yaw"]["2"]["status"] = True
                self.status_stage_1["forward"]["2"]["time"] = time()
            self.send_control_message()
            return
        
        elif self.status_stage_1["forward"]["2"]["status"] is False:
            curr_time = time()
            distance_target = 8.0  # meters
            velocity_forward = 0.5 # m/s
            if curr_time - self.status_stage_1["forward"]["2"]["time"] < distance_target / velocity_forward:
                self.velocity_cmd["x"] = - velocity_forward
                self.velocity_cmd["y"] = 0.0
            else:
                self.status_stage_1["forward"]["2"]["time"] = curr_time
                self.velocity_cmd["x"] = 0.0
                self.velocity_cmd["y"] = 0.0
                self.status_stage_1["forward"]["2"]["status"] = True
            self.send_control_message()
            return
        
        elif self.status_stage_1["yaw"]["3"]["status"] is False:
            yaw_speed = 6.0
            self.yaw_target = 72.0
            error_yaw = self.yaw_target - self.local_position.get("yaw", 0.0)
            if error_yaw < 0:
                # rotate left
                self.velocity_cmd["yaw"] = self.deg_to_rad_per_sec(-yaw_speed)
            elif error_yaw > 0:
                # rotate right
                self.velocity_cmd["yaw"] = self.deg_to_rad_per_sec(yaw_speed)
            if math.fabs(error_yaw) < 4.0:
                self.velocity_cmd["yaw"] = 0.0
                self.status_stage_1["yaw"]["3"]["status"] = True
                self.status_stage_1["forward"]["3"]["time"] = time()
            self.send_control_message()
            return
        
        elif self.status_stage_1["forward"]["3"]["status"] is False:
            curr_time = time()
            distance_target = 3.0  # meters
            velocity_forward = 0.5 # m/s
            if curr_time - self.status_stage_1["forward"]["3"]["time"] < distance_target / velocity_forward:
                self.velocity_cmd["x"] = - velocity_forward
                self.velocity_cmd["y"] = 0.0
            else:
                self.status_stage_1["forward"]["3"]["time"] = curr_time
                self.velocity_cmd["x"] = 0.0
                self.velocity_cmd["y"] = 0.0
                self.status_stage_1["forward"]["3"]["status"] = True
            self.send_control_message()
            return
        
        elif self.status_stage_1["land"]["status"] is False:
            self.control_msg = "mode"
            self.flight_mode = "land"
            self.send_control_message()
            if self.armed_state == 1:
                self.stage_1_done = True
                self.take_off_done = False
                self.status_stage_1["land"]["status"] = True
            return

        self.send_control_message()

    def stage_2(self):

        ###### Take off ######
        if not self.take_off_done:
            self.take_off_done = self.take_off(altitude_target=self.altitude_target)
            return

        self.control_msg = "velocity"

        ###### pid altitude control ######
        current_altitude = - self.local_position.get("z", 0.0)
        self.velocity_cmd["z"] = - self.altitude_pid.compute(self.altitude_target, current_altitude)
        
        if self.status_stage_2["yaw"]["1"]["status"] is False:
            yaw_speed = 6.0
            self.yaw_target = -90
            error_yaw = self.yaw_target - self.local_position.get("yaw", 0.0)
            if error_yaw < 0:
                # rotate left
                self.velocity_cmd["yaw"] = self.deg_to_rad_per_sec(-yaw_speed)
            elif error_yaw > 0:
                # rotate right
                self.velocity_cmd["yaw"] = self.deg_to_rad_per_sec(yaw_speed)
            if math.fabs(error_yaw) < 4.0:
                self.velocity_cmd["yaw"] = 0.0
                self.status_stage_2["yaw"]["1"]["status"] = True
                self.status_stage_2["forward"]["1"]["time"] = time()
            self.send_control_message()
            return
        
        elif self.status_stage_2["forward"]["1"]["status"] is False:
            curr_time = time()
            distance_target = 6.3  # meters
            velocity_forward = 0.5 # m/s
            if curr_time - self.status_stage_2["forward"]["1"]["time"] < distance_target / velocity_forward:
                self.velocity_cmd["x"] = - velocity_forward
                self.velocity_cmd["y"] = 0.0
            else:
                self.status_stage_2["forward"]["1"]["time"] = curr_time
                self.velocity_cmd["x"] = 0.0
                self.velocity_cmd["y"] = 0.0
                self.status_stage_2["forward"]["1"]["status"] = True
                self.status_stage_2["drop_ball"]["1"]["time"] = time()
            self.send_control_message()
            return
        
        elif self.status_stage_2["drop_ball"]["1"]["status"] is False:
            curr_time = time()
            hold_time = 10  # seconds
            elapsed = curr_time - self.status_stage_2["drop_ball"]["1"]["time"]

            if hold_time - 4 <= elapsed <= hold_time - 3:
                self.drop_ball("ball_1")
            elif elapsed < hold_time:
                pass
            else:
                self.status_stage_2["drop_ball"]["1"]["time"] = curr_time
                self.status_stage_2["drop_ball"]["1"]["status"] = True
                self.status_stage_2["forward"]["2"]["time"] = curr_time
            self.send_control_message()
            return
        
        elif self.status_stage_2["forward"]["2"]["status"] is False:
            curr_time = time()
            distance_target = 10.0  # meters
            velocity_forward = 0.5 # m/s
            if curr_time - self.status_stage_2["forward"]["2"]["time"] < distance_target / velocity_forward:
                self.velocity_cmd["x"] = - velocity_forward
                self.velocity_cmd["y"] = 0.0
            else:
                self.status_stage_2["forward"]["2"]["time"] = curr_time
                self.velocity_cmd["x"] = 0.0
                self.velocity_cmd["y"] = 0.0
                self.status_stage_2["forward"]["2"]["status"] = True
            self.send_control_message()
            return

        elif self.status_stage_2["land"]["status"] is False:
            self.control_msg = "mode"
            self.flight_mode = "land"
            self.send_control_message()
            self.stage_2_done = True
            return 

        self.send_control_message()

    def main_trajectory(self):
        
        if not self.stage_1_done:
            self.stage_1()
            return
        
        if not self.stage_2_done:
            self.stage_2()
            return
    
    def select_mode(self, mode):
        self.control_msg = "mode"
        self.flight_mode = mode
        self.send_control_message()
    
    def main_control_loop(self):
        self.main_trajectory()

    def send_control_message(self):
        control_msg = {
            "arm": self.is_arm,
            "control": self.control_msg,
            "mode": self.flight_mode,
            "drop_ball": self.drop_flags,
            "velocity": self.velocity_cmd
        }
        msg = String()
        msg.data = json.dumps(control_msg)
        self.control_publisher.publish(msg)

    def drop_status_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            # self.get_logger().info(f"[Drop Status] {data}")
        except json.JSONDecodeError:
            self.get_logger().error("[Drop Status] Invalid JSON")

    def islab_status_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.nav_state = data.get("nav_state")
            self.armed_state = data.get("arming_state")
            self.pre_arm_check = data.get("pre_arm_check")

            local = data.get("local_position", {})
            for key in self.local_position:
                self.local_position[key] = local.get(key, 0.0)

            # self.get_logger().info(
            #     f"[Status] Nav:{self.nav_state} Arm:{self.armed_state} Pre:{self.pre_arm_check} | "
            #     f"Z:{- self.local_position['z']:.2f} Vz:{self.local_position['vz']:.2f}"
            # )
        except json.JSONDecodeError:
            self.get_logger().error("[Status] Invalid JSON")

    def bottom_camera_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Bottom Camera", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"[Camera Bottom] {e}")

    def forward_camera_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Forward Camera", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"[Camera Forward] {e}")

def main(args=None):
    rclpy.init(args=args)
    node = IslabControlUI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("IslabControlUI interrupted.")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
