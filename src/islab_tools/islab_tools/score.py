#!/usr/bin/env python3
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry
from islab_msgs.msg import IslabDropBall
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from time import time


class IslabScore(Node):
    def __init__(self):
        super().__init__("islab_score_node")
        self.get_logger().info("IslabScore node started")

        # --- QoS profiles ---
        self.qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- Subscriptions ---
        self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self.status_odom_callback,
            self.qos_profile_sub,
        )
        self.create_subscription(Odometry, "/UAV/odom", self.odom_callback, 10)

        # Home pad contacts (renamed H1 / H2)
        self.create_subscription(Bool, "/islab/home/H1/is_contact", self.contact_H1_callback, 10)
        self.create_subscription(Bool, "/islab/home/H2/is_contact", self.contact_H2_callback, 10)

        # Target contacts (T1 -> T5)
        self.target_topics = [f"/islab/targets/T{i}/is_contact" for i in range(1, 6)]
        self.target_contacts = {topic: False for topic in self.target_topics}
        for topic in self.target_topics:
            self.create_subscription(Bool, topic, lambda msg, t=topic: self.target_contact_callback(msg, t), 10)

        # --- State variables ---
        self.odom_x = self.odom_y = self.odom_z = 0.0
        self.height = None
        self.height2check = 2.0
        self.safe_height = 0.2
        self.time_height = time()
        self.time_safe_height = 10  # seconds
        self.count_alarm_check_height = 0
        self.max_count_check_height = 3
        self.loss = False
        
        self.total_score = 0
        self.score_point = 10

        # Home pad states (renamed)
        self.home_H1_contact = False
        self.home_H2_contact = False
        self.home_contact_detected = False

        # --- Timer ---
        self.fps = 30
        self.create_timer(1.0 / self.fps, self.main)

    # ====================== #
    #   HEIGHT CHECK LOGIC   #
    # ====================== #
    def check_height(self):
        if self.height is None:
            return

        if self.height2check - self.height > self.safe_height:
            curr_time = time()
            if curr_time - self.time_height >= self.time_safe_height:
                self.time_height = time()
                if self.count_alarm_check_height >= self.max_count_check_height:
                    self.loss = True
                    self.get_logger().warn("Height loss detected!")
                else:
                    self.count_alarm_check_height += 1
            else:
                pass
        else:
            self.time_height = time()

    # ====================== #
    #  TARGET / LANDING LOGIC #
    # ====================== #
    def check_targets(self):
        """Check if UAV or dropped ball has made contact with any target."""
        for topic, state in self.target_contacts.items():
            if state:
                # self.get_logger().info(f"[TARGET HIT] {topic}")
                pass
                # Add scoring or validation logic here if needed

    def check_land_home(self):
        """Check if UAV has landed on a home pad."""
        if self.home_H1_contact or self.home_H2_contact:
            if not self.home_contact_detected:
                self.home_contact_detected = True
                self.get_logger().info("[LANDING] UAV touched home pad!")
        else:
            if self.home_contact_detected:
                self.home_contact_detected = False
                self.get_logger().info("[TAKEOFF] UAV left home pad!")

    # ====================== #
    #        MAIN LOOP       #
    # ====================== #
    def main(self):
        self.check_height()
        self.check_land_home()
        self.check_targets()

    # ====================== #
    #      CALLBACKS         #
    # ====================== #
    def odom_callback(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.odom_z = msg.pose.pose.position.z

    def status_odom_callback(self, msg: VehicleOdometry):
        try:
            self.height = -msg.position[2]
        except Exception as e:
            self.get_logger().error(f"[Status Odometry] {e}")

    def contact_H1_callback(self, msg: Bool):
        """Triggered when home H1 pad contact changes."""
        self.home_H1_contact = msg.data

    def contact_H2_callback(self, msg: Bool):
        """Triggered when home H2 pad contact changes."""
        self.home_H2_contact = msg.data

    def target_contact_callback(self, msg: Bool, topic: str):
        """Triggered when a target contact changes."""
        prev_state = self.target_contacts[topic]
        self.target_contacts[topic] = msg.data
        if msg.data and not prev_state:
            self.get_logger().info(f"[CONTACT START] {topic}")
        elif not msg.data and prev_state:
            self.get_logger().info(f"[CONTACT END] {topic}")


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


if __name__ == "__main__":
    main()
