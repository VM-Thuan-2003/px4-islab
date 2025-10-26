import math
import numpy as np
from islab_msgs.msg import IslabControl
from px4_msgs.msg import VehicleAttitude, TrajectorySetpoint

class IslabTrajectory:
    """Handles ROS2 subscriptions and publications with QoS settings."""
    def __init__(self, logger=None):
        
        self.logger = logger

        self.trueYaw = 0.0

    def control_velocity(self, msg:IslabControl, ctx):
        if self.logger:
            self.logger.info(f"Controlling velocity with message: {msg}")
        vyaw = math.radians(msg.vyaw)
        vx = msg.vx
        vy = msg.vy
        vz = msg.vz
        vx_world = vx * math.cos(self.trueYaw) - vy * math.sin(self.trueYaw)
        vy_world = vx * math.sin(self.trueYaw) + vy * math.cos(self.trueYaw)
        msg = TrajectorySetpoint()
        msg.timestamp = msg.timestamp
        msg.velocity = [vx_world, vy_world, vz]
        msg.yaw = float('nan')
        msg.yawspeed = vyaw
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.acceleration = [float('nan'), float('nan'), float('nan')]
        ctx.pub_trajectory_setpoint.publish(msg)

    def _on_attitude(self, msg:VehicleAttitude):
        # if self.logger:
        #     self.logger.info(f"Received attitude message: {msg}")
        
        orientation_q = msg.q
        self.trueYaw = -(np.arctan2(2.0*(orientation_q[3]*orientation_q[0] + orientation_q[1]*orientation_q[2]), 
                        1.0 - 2.0*(orientation_q[0]*orientation_q[0] + orientation_q[1]*orientation_q[1])))
