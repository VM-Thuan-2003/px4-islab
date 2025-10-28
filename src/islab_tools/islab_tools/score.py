#!/usr/bin/env python3
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleOdometry, VehicleStatus
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from islab_msgs.msg import IslabScore
from time import time


class IslabScoreNode(Node):
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
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self.status_mode_callback,
            self.qos_profile_sub,
        )
        self.create_subscription(Odometry, "/UAV/odom", self.odom_callback, 10)

        # Home pad contacts (H1 / H2)
        self.create_subscription(Bool, "/islab/home/H1/is_contact", self.contact_H1_callback, 10)
        self.create_subscription(Bool, "/islab/home/H2/is_contact", self.contact_H2_callback, 10)

        # (Optional) Target contacts (T1 -> T5)
        self.target_topics = [f"/islab/targets/T{i}/is_contact" for i in range(1, 6)]
        self.target_contacts = {topic: False for topic in self.target_topics}
        for topic in self.target_topics:
            self.create_subscription(
                Bool,
                topic,
                lambda msg, t=topic: self.target_contact_callback(msg, t),
                10,
            )

        # --- Publisher ---
        self.score_pub = self.create_publisher(IslabScore, "/islab/score/status", 10)

        # --- PX4 mode/status ---
        self.nav_state = None

        # --- Position & height tracking ---
        self.odom_x = self.odom_y = self.odom_z = 0.0
        self.height = None
        self.height2check = 2.0
        self.safe_height = 0.2
        self.time_height = time()
        self.time_safe_height = 10.0
        self.count_alarm_check_height = 0
        self.max_count_check_height = 3
        self.loss = False

        # --- Scoring ---
        self.total_score = 0
        self.score_point = 10
        self.alarm_rule_height = 0

        # --- Time accumulation ---
        self.time_total = 0.0            # accumulated flight time (s)
        self._last_time_update = time()
        self._offboard_active = False    # currently in OFFBOARD
        self._land_detected = False      # currently in LAND mode

        # --- Cooldowns ---
        self.hit_cooldown_s = 0.5
        self._last_award_time = {t: 0.0 for t in self.target_topics}

        # --- Home pads ---
        self._initialized_edges = False
        self.home_H1_contact = False
        self.home_H2_contact = False
        self._prev_h1 = False
        self._prev_h2 = False
        self.h1_takeoff_awarded = False
        self.h2_cooldown_s = 0.5
        self._last_h2_award = 0.0

        # --- Timers ---
        self.fps = 30
        self.create_timer(1.0 / self.fps, self.main)
        self.create_timer(0.1, self.publish_score_message)  # 10 Hz publisher

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
                    self.alarm_rule_height = 1
                    self.get_logger().warn("Height alarm triggered!")
                else:
                    self.count_alarm_check_height += 1
        else:
            self.time_height = time()
            self.alarm_rule_height = 0

    # ====================== #
    #  HOME PAD SCORING LOGIC#
    # ====================== #
    def check_land_home(self):
        now = time()

        if not self._initialized_edges:
            self._prev_h1 = self.home_H1_contact
            self._prev_h2 = self.home_H2_contact
            self._initialized_edges = True
            return

        # --- TAKEOFF (H1 contact -> no contact) ---
        if self._prev_h1 and not self.home_H1_contact:
            if not self.home_H2_contact and not self.h1_takeoff_awarded:
                self.h1_takeoff_awarded = True
                self.total_score += self.score_point
                self.get_logger().info(f"[SCORE] +{self.score_point} TAKEOFF from H1 -> total={self.total_score}")
                self.publish_score_message()

        # --- LANDING (H2 no contact -> contact) ---
        if (not self._prev_h2) and self.home_H2_contact:
            if not self.home_H1_contact:
                if now - self._last_h2_award >= self.h2_cooldown_s:
                    self._last_h2_award = now
                    self.total_score += self.score_point
                    self.get_logger().info(f"[SCORE] +{self.score_point} LAND on H2 -> total={self.total_score}")
                    self.publish_score_message()

        self._prev_h1 = self.home_H1_contact
        self._prev_h2 = self.home_H2_contact

    # ====================== #
    #   TARGET HIT SCORING   #
    # ====================== #
    def target_contact_callback(self, msg: Bool, topic: str):
        prev_state = self.target_contacts[topic]
        self.target_contacts[topic] = msg.data

        if msg.data and not prev_state:
            now = time()
            if now - self._last_award_time[topic] >= self.hit_cooldown_s:
                self._last_award_time[topic] = now
                self.total_score += self.score_point
                self.get_logger().info(f"[SCORE] +{self.score_point} on {topic} -> total={self.total_score}")
                self.publish_score_message()

    # ====================== #
    #        MAIN LOOP       #
    # ====================== #
    def main(self):
        now = time()
        dt = now - self._last_time_update
        self._last_time_update = now

        # --- Accumulate time when in OFFBOARD (14) ---
        if self.nav_state == 14:  # OFFBOARD active
            if not self._offboard_active:
                self._offboard_active = True
                self._land_detected = False
                self.get_logger().info("OFFBOARD active — start accumulating time.")
            self.time_total += dt

        # --- Pause counting when entering LAND mode (18) ---
        elif self.nav_state == 18:  # LAND mode detected
            if not self._land_detected:
                self._land_detected = True
                if self._offboard_active:
                    self._offboard_active = False
                    self.get_logger().info("LAND mode detected — pause time accumulation.")
            # No accumulation in LAND

        # --- Stop accumulation if any other mode ---
        else:
            if self._offboard_active:
                self._offboard_active = False
                self.get_logger().info("OFFBOARD ended — stop accumulating time.")

        # --- Additional checks ---
        if self.nav_state == 14:
            self.check_height()
        self.check_land_home()

    # ====================== #
    #     PUBLISH SCORE      #
    # ====================== #
    def publish_score_message(self):
        msg = IslabScore()
        now_us = int(time() * 1e6)
        msg.timestamp = now_us
        msg.timestamp_sample = now_us
        msg.total_score = int(self.total_score)
        msg.score_point = int(self.score_point)
        msg.alarm_rule_height = int(self.alarm_rule_height)
        msg.time_total = int(self.time_total)
        self.score_pub.publish(msg)

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

    def status_mode_callback(self, msg: VehicleStatus):
        try:
            self.nav_state = msg.nav_state
        except Exception as e:
            self.get_logger().error(f"[Status Mode] {e}")

    def contact_H1_callback(self, msg: Bool):
        self.home_H1_contact = msg.data

    def contact_H2_callback(self, msg: Bool):
        self.home_H2_contact = msg.data


def main():
    rclpy.init()
    node = None
    try:
        node = IslabScoreNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("KeyboardInterrupt — shutting down node.")
    except Exception as e:
        if node:
            node.get_logger().error(f"Exception: {e}")
        else:
            print(f"Exception before node started: {e}")
        raise
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
