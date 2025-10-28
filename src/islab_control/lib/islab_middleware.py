from islab_msgs.msg import IslabChangeMode
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleGlobalPosition, VehicleOdometry

from tf_transformations import euler_from_quaternion

class ListMode:
    # Custom modes
    STABILIZEMODE = 0
    ALTMODE = 1
    POSHOLDMODE = 2
    AUTOMODE = 3
    OFFBOARDMODE = 4
    RTHMODE = 5
    LANDMODE = 6

    # Sources
    GROUNDSOURCE = 0
    WEBSOURCE = 1
    USERSOURCE = 2

    # Mapping from custom mode to (PX4 main mode, PX4 sub mode)
    MODE_TO_PX4 = {
        STABILIZEMODE: (7, 0),   # STABILIZED (your mapping)
        ALTMODE: (2, 0),         # ALTCTL
        POSHOLDMODE: (3, 0),     # POSCTL
        AUTOMODE: (4, 4),        # AUTO.MISSION
        OFFBOARDMODE: (6, 0),    # OFFBOARD
        RTHMODE: (4, 5),         # AUTO.RTL (your mapping)
        LANDMODE: (4, 6),        # AUTO.LAND (your mapping)
    }

class IslabMiddleware:
    def __init__(self, node=None, logger=None):
        self.node = node
        self.logger = logger

        self._alt_local = None

        self.vehicle_global_position = [0.0, 0.0, 0.0]  # [lat, lon, alt]
        self.vehicle_start_global_position = [0.0, 0.0, 0.0]  # [lat, lon, alt]
        self.get_start_global_position = False
        
        self.vehicle_odometry_position = [0.0, 0.0, 0.0]  # [x, y, z]
        self.vehicle_start_odometry_position = [0.0, 0.0, 0.0]  # [x, y, z]
        self.vehicle_odometry_orientation = [0.0, 0.0, 0.0]  # [roll, pitch, yaw]
        self.vehicle_start_odometry_orientation = [0.0, 0.0, 0.0]  # [roll, pitch, yaw]
        self.get_start_ground_position = False

    def _on_global_position(self, msg: VehicleGlobalPosition):
        self.vehicle_global_position[0] = msg.lat # Latitude, (degrees)
        self.vehicle_global_position[1] = msg.lon # Longitude, (degrees)
        self.vehicle_global_position[2] = msg.alt # Altitude AMSL, (meters)
        if(self.get_start_global_position==False):
            self.vehicle_start_global_position[0] = msg.lat # Latitude, (degrees)
            self.vehicle_start_global_position[1] = msg.lon # Longitude, (degrees)
            self.vehicle_start_global_position[2] = msg.alt # Altitude AMSL, (meters)
            self.get_start_global_position=True

    def _on_odometry(self, msg: VehicleOdometry):
        self.vehicle_odometry_position[0] = msg.position[0]
        self.vehicle_odometry_position[1] = -msg.position[1]
        self.vehicle_odometry_position[2] = -msg.position[2]
        self.vehicle_odometry_orientation=euler_from_quaternion(msg.q)
        if(self.get_start_ground_position==False):
            self.vehicle_start_odometry_position[0] = self.vehicle_odometry_position[0]
            self.vehicle_start_odometry_position[1] = self.vehicle_odometry_position[1]
            self.vehicle_start_odometry_position[2] = self.vehicle_odometry_position[2]
            self.vehicle_start_odometry_orientation = self.vehicle_odometry_orientation
            self.get_start_ground_position=True

    def handle_islab_change_mode(self, msg: IslabChangeMode):
        if hasattr(msg, "handel"):
            if msg.handel == 0:
                self.change_mode(msg)
            elif msg.handel == 1:
                self.control_takeoff(msg.altitude if hasattr(msg, "altitude") else 2.0)
            elif msg.handel == 2:
                self.control_arm_disarm(bool(msg.arm))
        else:
            self.change_mode(msg)

    def change_mode(self, msg: IslabChangeMode):
        mode = msg.mode
        main_mode, sub_mode = ListMode.MODE_TO_PX4.get(mode, (4, 4))  # default: AUTO.MISSION
        if self.logger:
            self.logger.info(f"Mapped custom mode {mode} to PX4 main_mode {main_mode}, sub_mode {sub_mode}")
        self.publish_vehicle_command(
            command=VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,               # Custom main mode flag
            param2=float(main_mode),  # PX4 main mode
            param3=float(sub_mode)    # PX4 sub mode
        )

    def control_arm_disarm(self, arm: bool):
        if arm is True:
            self.get_start_global_position = False
            self.get_start_ground_position = False
        command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        param1 = 1.0 if arm else 0.0
        self.publish_vehicle_command(command=command, param1=param1)
    
    def control_takeoff(self, requested_altitude: float):
        command = VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF
        self.publish_vehicle_command(command=command,
                                    param1=self.vehicle_start_odometry_orientation[1], #desired pitch
                                    param4=self.vehicle_start_odometry_orientation[2],#Yaw angle
                                    param5=self.vehicle_start_global_position[0],#Latitude
                                    param6=self.vehicle_start_global_position[1],#Longitude
                                    param7=self.vehicle_start_global_position[2]+requested_altitude)#Altitude

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.node.get_timestamp()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.node.pub_vehicle_command.publish(msg)
        if self.logger:
            self.logger.info(
                f"Sent VehicleCommand: command={command}, param1={param1}, param2={param2}, param3={param3}, param7={param7}"
            )
