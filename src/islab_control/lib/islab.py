from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from islab_msgs.msg import IslabControl, IslabChangeMode
from px4_msgs.msg import OffboardControlMode, VehicleCommand, TrajectorySetpoint, VehicleAttitude
from px4_msgs.msg import VehicleGlobalPosition, VehicleOdometry
from functools import partial

try:
    from lib.islab_middleware import IslabMiddleware
    from lib.islab_trajectory import IslabTrajectory
except ImportError:
    import sys, os
    sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../lib'))
    from islab_middleware import IslabMiddleware
    from islab_trajectory import IslabTrajectory

class IslabVariable:
    """Represents a configuration variable for Islab."""
    def __init__(self, name, value):
        self.name = name
        self.value = value

class IslabSetup:
    """Handles subscriber and publisher setup for Islab."""

    def __init__(self, logger=None):
        self.logger = logger

    def _make_qos(self, qos_depth_or_profile):
        """Converts an int (queue size) or QoSProfile into a QoSProfile."""
        if isinstance(qos_depth_or_profile, QoSProfile):
            return qos_depth_or_profile
        return QoSProfile(depth=qos_depth_or_profile)

    def setup_sub(self, node, topic, msg_type, callback, qos_depth_or_profile=10):
        qos = self._make_qos(qos_depth_or_profile)
        if self.logger:
            self.logger.info(f"Subscribing to topic: {topic} (QoS: {qos})")
        return node.create_subscription(msg_type, topic, callback, qos)

    def setup_pub(self, node, topic, msg_type, qos_depth_or_profile=10):
        qos = self._make_qos(qos_depth_or_profile)
        if self.logger:
            self.logger.info(f"Creating publisher on topic: {topic} (QoS: {qos})")
        return node.create_publisher(msg_type, topic, qos)

class Islab:
    """Main Islab logic, receives config as list of IslabVariable."""
    def __init__(self, node, config_vars, logger=None):
        self.node = node
        self.logger = logger

        self.config_dict = {var.name: var.value for var in config_vars}
        self.vehicle_id = self.config_dict.get('vehicle_id', 'unknown')

        self.sub_topic_velocity = self.config_dict.get('sub.topic_velocity', 'unknown')
        self.sub_topic_change_mode = self.config_dict.get('sub.topic_change_mode', 'unknown')
        self.sub_topic_attitude = self.config_dict.get('sub.topic_attitude', 'unknown')
        self.sub_topic_global_position = self.config_dict.get('sub.topic_global_position', 'unknown')
        self.sub_topic_odometry = self.config_dict.get('sub.topic_odometry', 'unknown')

        self.pub_offboard_control_node = self.config_dict.get('pub.offboard_control_node', 'unknown')
        self.pub_vehicle_command = self.config_dict.get('pub.vehicle_command', 'unknown')
        self.pub_trajectory_setpoint = self.config_dict.get('pub.trajectory_setpoint', 'unknown')

        if self.logger:
            self.logger.info("Islab initialized.")
            self.logger.info(f"Vehicle ID: {self.vehicle_id}")

        self.setup = IslabSetup(logger=self.logger)
        self.middleware = IslabMiddleware(self, logger=self.logger)
        self.trajectory = IslabTrajectory(logger=self.logger)

        self.qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pub_vehicle_command = self.setup.setup_pub(self.node, self.pub_vehicle_command, VehicleCommand, self.qos_profile_pub)
        self.pub_trajectory_setpoint = self.setup.setup_pub(self.node, self.pub_trajectory_setpoint, TrajectorySetpoint, self.qos_profile_pub)
        self.pub_offboard_control = self.setup.setup_pub(self.node, self.pub_offboard_control_node, OffboardControlMode, self.qos_profile_pub)

        self.setup_sub_velocity = self.setup.setup_sub(self.node, self.sub_topic_velocity, IslabControl, partial(self.trajectory.control_velocity, ctx=self), self.qos_profile_sub)
        self.setup_sub_change_mode = self.setup.setup_sub(self.node, self.sub_topic_change_mode, IslabChangeMode, self.middleware.handle_islab_change_mode, self.qos_profile_sub)
        self.setup_sub_attitude = self.setup.setup_sub(self.node, self.sub_topic_attitude, VehicleAttitude, self.trajectory._on_attitude, self.qos_profile_sub)
        self.setup_sub_global_position = self.setup.setup_sub(self.node, self.sub_topic_global_position, VehicleGlobalPosition, self.middleware._on_global_position, self.qos_profile_sub)
        self.setup_sub_odometry = self.setup.setup_sub(self.node, self.sub_topic_odometry, VehicleOdometry, self.middleware._on_odometry, self.qos_profile_sub)

        self.timer_offboard = self.node.create_timer(1.0 / 30.0, self.send_offboard_control_mode)

    def get_timestamp(self):
        return self.node.get_clock().now().nanoseconds // 1000

    def send_offboard_control_mode(self):
        # if self.logger:
        #     self.logger.info("Sending Offboard Control Mode message.")
        msg = OffboardControlMode()
        msg.timestamp = self.get_timestamp()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.pub_offboard_control.publish(msg)