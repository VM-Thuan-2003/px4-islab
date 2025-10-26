import rclpy
from rclpy.node import Node
try:
    from lib.islab import Islab, IslabVariable
except ImportError:
    import sys, os
    sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../lib'))
    from islab import Islab, IslabVariable

class IslabMain(Node):
    def __init__(self):
        super().__init__('islab_main_node')
        self.get_logger().info("IslabMain Node has been started.")

        param_list = {
            'vehicle_id': 'islab',
            'sub.topic_velocity': 'islab/velocity',
            'sub.topic_change_mode': 'islab/change_mode',
            'sub.topic_attitude': '/fmu/out/vehicle_attitude',
            'sub.topic_global_position': '/fmu/out/vehicle_global_position',
            'sub.topic_odometry': '/fmu/out/vehicle_odometry',
            'pub.offboard_control_node': '/fmu/in/offboard_control_mode',
            'pub.vehicle_command': '/fmu/in/vehicle_command',
            'pub.trajectory_setpoint': '/fmu/in/trajectory_setpoint',
        }

        # Declare params and create IslabVariable objects
        config_vars = []
        for name, default in param_list.items():
            self.declare_parameter(name, default)
            value = self.get_parameter(name).value
            config_vars.append(IslabVariable(name, value))

        # Pass list of IslabVariable objects to Islab
        self.islab = Islab(self, config_vars, logger=self.get_logger())


def main(args=None):
    rclpy.init(args=args)
    node = IslabMain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
