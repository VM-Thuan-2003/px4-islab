import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os

# Command test to run:
# ros2 topic pub /drop_ball std_msgs/String "data: 'ball_2,0.1,0.0,1.0'"
# Expected format: id,x,y,z
# Example: 'ball_1,0.0,0.0,1.0'

curr_path = os.path.dirname(os.path.abspath(__file__))
islab_px4_path = os.path.abspath(os.path.join(curr_path, '..', '..', '..', '..', '..', '..', '..', 'islab_px4'))


class BallSpawner(Node):
    def __init__(self):
        super().__init__('ball_spawner_shell_command')

        self.sdf_path = islab_px4_path + "/contest/model/ball/ball.sdf"

        self.subscription = self.create_subscription(
            String,
            '/drop_ball',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        try:
            parts = msg.data.strip().split(',')
            if len(parts) != 4:
                raise ValueError("Expected format: id,x,y,z")

            ball_id = parts[0].strip()
            x = float(parts[1])
            y = float(parts[2])
            z = float(parts[3])

            with open(self.sdf_path, 'r') as f:
                sdf_content = f.read().replace("'", "''")

            service_call = (
                f"ros2 service call /spawn_entity gazebo_msgs/srv/SpawnEntity "
                f"\"{{name: '{ball_id}', xml: '{sdf_content}', "
                f"initial_pose: {{position: {{x: {x}, y: {y}, z: {z}}}}}, reference_frame: 'world'}}\""
            )

            self.get_logger().info(f"Running: {ball_id} at ({x}, {y}, {z})")
            result = subprocess.run(service_call, shell=True, capture_output=True, text=True)

            if result.returncode == 0:
                self.get_logger().info(f"Success:\n{result.stdout}")
            else:
                self.get_logger().error(f"Error:\n{result.stderr}")

        except Exception as e:
            self.get_logger().error(f"Exception: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BallSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
