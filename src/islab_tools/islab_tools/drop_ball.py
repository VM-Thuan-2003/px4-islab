import rclpy
from rclpy.node import Node
from islab_msgs.msg import IslabDropBall, StatusDropBall
import subprocess
import os

from nav_msgs.msg import Odometry

curr_path = os.path.dirname(os.path.abspath(__file__))
islab_px4_path = os.path.abspath(os.path.join(curr_path, '..', '..', '..', '..', '..', '..', '..', 'islab_px4/src/islab_tools'))

class IslabDropball(Node):
    def __init__(self):
        super().__init__("islab_drop_ball_node")

        # --- Declare parameters for topic names ---
        self.declare_parameter('dropball_topic', '/islab/dropball')
        self.declare_parameter('odom_topic', '/UAV/odom')
        self.declare_parameter('status_dropball_topic', '/islab/status_dropball')

        # --- Get params ---
        self.dropball_topic = self.get_parameter('dropball_topic').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.status_dropball_topic = self.get_parameter('status_dropball_topic').get_parameter_value().string_value

        self.sdf_path = islab_px4_path + "/model/ball/ball.sdf"

        self.subscription = self.create_subscription(IslabDropBall, self.dropball_topic, self.dropball_cb, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.pub_status = self.create_publisher(StatusDropBall, self.status_dropball_topic, 10)

        self.odom_x = self.odom_y = self.odom_z = 0.0
        self.offset_z = -0.4

        self.ball_ids = [1, 2, 3, 4, 5]
        self.dropped = [False, False, False, False, False]

        self.timer = self.create_timer(1.0, self.pub_status_dropball)

    def odom_callback(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_z = msg.pose.pose.position.z

    def drop_action(self, id, pos):
        try:
            string_id = str(id)
            x, y, z = pos
            with open(self.sdf_path, 'r') as f:
                sdf_content = f.read().replace("'", "''")
            service_call = (
                f"ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "
                f"\"{{name: '{string_id}', xml: '{sdf_content}', "
                f"initial_pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}, reference_frame: 'world'}}\""
            )
            result = subprocess.run(service_call, shell=True, capture_output=True, text=True)
            if result.returncode == 0:
                self.get_logger().info(f"Success:\n{result.stdout}")
                return True
            else:
                self.get_logger().error(f"Error:\n{result.stderr}")
                return False
        except Exception as e:
            self.get_logger().error(f"Exception in drop_action: {e}")
            return False

    def pub_status_dropball(self):
        msg = StatusDropBall()
        msg.ball_id = self.ball_ids
        msg.dropped = self.dropped
        self.pub_status.publish(msg)

    def dropball_cb(self, msg : IslabDropBall):
        self.ball_ids = msg.ball_id
        drops = msg.drop
        pos = self.odom_x, self.odom_y, self.odom_z + self.offset_z
        for drop in range(len(drops)):
           if drops[drop] == True and self.dropped[drop] == False:
               print(f"pos: {pos}")
               status = self.drop_action(self.ball_ids[drop], pos)
               if status is True:
                   self.dropped[drop] = True

def main():
    rclpy.init()
    node = None
    try:
        node = IslabDropball()
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
