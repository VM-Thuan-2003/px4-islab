import rclpy
from rclpy.node import Node
from islab_msgs.msg import IslabDropBall, StatusDropBall
import subprocess
import os
from typing import Optional

from nav_msgs.msg import Odometry

def find_target_in_workspace(
    current_path: str,
    target: str,
    root_marker: str = "ISLAB-PX4-AUTOPILOT",
    max_up_levels: int = 20,
    fallback_levels: Optional[int] = 3,
    ensure_exists: bool = False,
) -> Optional[str]:
    """
    Find and return an absolute path to `root_marker/target` by walking up from current_path.
    If `root_marker` is not found within `max_up_levels`, a fallback is attempted by going up
    `fallback_levels` from current_path and joining `target`. If fallback_levels is None,
    function returns None when marker not found.

    Args:
        current_path (str): file or directory path to start searching from.
        target (str): directory or file name to join to the found root.
        root_marker (str): folder name that identifies the workspace root.
        max_up_levels (int): maximum number of parent directories to inspect for the root marker.
        fallback_levels (Optional[int]): if marker not found, go up this many levels and join target.
                                         If None, do not attempt fallback and return None.
        ensure_exists (bool): if True and resulting path doesn't exist, create directories (os.makedirs).

    Returns:
        Optional[str]: absolute path to the joined target, or None if not found and no fallback.
    """

    # normalize and make absolute
    path = os.path.abspath(current_path)

    # If the path is a file, use its directory
    if os.path.isfile(path):
        path = os.path.dirname(path)

    # split into components to quickly check if marker is in path
    parts = path.split(os.sep)

    # quick direct check: if marker already in path, compute immediately
    if root_marker in parts:
        idx = parts.index(root_marker)
        root = os.sep.join(parts[: idx + 1 ]) or os.sep
        result = os.path.join(root, target)
        result = os.path.abspath(result)
        if ensure_exists and not os.path.exists(result):
            os.makedirs(result, exist_ok=True)
        return result

    # otherwise, walk up step-by-step until we hit filesystem root or max_up_levels
    cur = path
    for i in range(max_up_levels):
        parent = os.path.dirname(cur)
        if not parent or parent == cur:
            break  # reached filesystem root
        # check if parent's name equals marker
        if os.path.basename(parent) == root_marker:
            root = parent
            result = os.path.abspath(os.path.join(root, target))
            if ensure_exists and not os.path.exists(result):
                os.makedirs(result, exist_ok=True)
            return result
        cur = parent

    # fallback behavior: optionally go up fallback_levels and join target
    if fallback_levels is not None:
        cur = path
        for _ in range(fallback_levels):
            cur_parent = os.path.dirname(cur)
            if not cur_parent or cur_parent == cur:
                break
            cur = cur_parent
        result = os.path.abspath(os.path.join(cur, target))
        if ensure_exists and not os.path.exists(result):
            os.makedirs(result, exist_ok=True)
        return result

    # no marker found and no fallback requested
    return None

curr_path = os.path.dirname(os.path.abspath(__file__))
islab_px4_path = find_target_in_workspace(current_path=curr_path, target='islab_px4/src/islab_tools')

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
