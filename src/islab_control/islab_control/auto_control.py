import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from islab_msgs.msg import IslabControl, IslabChangeMode, IslabDropBall, StatusDropBall
import numpy as np
import math
import cv2

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
        self.get_logger().info("Hello ROS 2!")
        
        self.bridge = CvBridge()
        
        self.qos_profile_pub = QoSProfile(
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
        self.create_subscription(Image, '/UAV/bottom/image_raw', self.bottom_camera_callback, 10)
        self.create_subscription(Image, '/UAV/forward/image_raw', self.forward_camera_callback, 10)

        self.frame_down = None
        self.frame_forward = None

        self.ball_ids = [1, 2, 3, 4, 5]
        self.ball_dropped = [False, False, False, False, False]
        
        self.drop_flags = {f"ball_{i}": False for i in range(1, 6)}
        self.velocity_cmd = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
        
    def main(self):
        pass
    
    def drop_status_callback(self, msg: StatusDropBall):
        try:
            self.ball_ids = msg.ball_id
            self.ball_dropped = msg.dropped
        except Exception as e:
            self.get_logger().error(f"[Status DropBall] {e}") 
    
    def bottom_camera_callback(self, msg: Image):
        try:
            self.frame_down = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Bottom Camera", self.frame_down)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"[Camera Bottom] {e}")

    def forward_camera_callback(self, msg: Image):
        try:
            self.frame_forward = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.imshow("Forward Camera", self.frame_forward)
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
