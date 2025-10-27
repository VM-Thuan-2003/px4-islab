from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from islab_msgs.msg import IslabDropBall
from nav_msgs.msg import Odometry
from time import time

class IslabScore(Node):
    def __init__(self):
        super().__init__("islab_score_node")
        self.get_logger().info("Hello ROS 2!")

        self.qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.status_odom_callback, self.qos_profile_sub)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        
        self.odom_x = self.odom_y = self.odom_z = 0.0
        
        self.height = None
        self.height2check = 2.0
        self.safe_height = 0.2
        self.time_height = time()
        self.time_safe_height = 10 #10s
        self.count_alarm_check_height = 0
        self.max_count_check_height = 3
        
        self.loss = False
        
        self.fps = 30
        self.create_timer(1.0/self.fps, self.main)

    def check_height(self):
        if self.height2check - self.height > self.safe_height:
            curr_time = time()
            if curr_time - self.time_height >= self.time_safe_height:
                self.time_height = time()
                if self.count_alarm_check_height >= self.max_count_check_height:
                    self.loss = True
                else:
                    self.count_alarm_check_height += 1
            else:
                pass
        else:
            self.time_height = time()
            
    
    def check_ball(self):
        pass
    
    def check_land_home(self):
        pass
    
    def main(self):
        self.check_height()
        # print(f"loss: {self.loss}- {self.height2check - self.height} - {time() - self.time_height} - {self.count_alarm_check_height}")
    
    def odom_callback(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_z = msg.pose.pose.position.z

    def status_odom_callback(self, msg:VehicleOdometry):
        try:
            self.height = -msg.position[2]
        except Exception as e:
            self.get_logger().error(f"[Status Odometry] {e}") 
    
    
def main():
    rclpy.init()
    node = None
    try:
        node = IslabScore()
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
