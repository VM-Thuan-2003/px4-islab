#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IslabScoreNode (ROS 2)
----------------------
- Computes total score and flight time.
- Starts scoring/timing only when:
    1) PX4 is in OFFBOARD mode (nav_state == 14)
    2) Auto mode is selected and started (via /islab/flag_mode IslabFlag)
- Stops or pauses scoring when:
    - LAND mode (nav_state == 18)
    - Manual mode is selected
    - Stop Auto flag received
"""

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
from islab_msgs.msg import IslabScore, IslabFlag
from time import time


class IslabScoreNode(Node):
    def __init__(self):
        super().__init__("islab_score_node")
        self.get_logger().info("IslabScore node started")

        # ---------------- QoS configuration ---------------- #
        self.qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---------------- Subscriptions ---------------- #
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.status_odom_callback, self.qos_profile_sub)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.status_mode_callback, self.qos_profile_sub)
        self.create_subscription(Odometry, "/UAV/odom", self.odom_callback, 10)

        # Home pad contacts (H1 / H2)
        self.create_subscription(Bool, "/islab/home/H1/is_contact", self.contact_H1_callback, 10)
        self.create_subscription(Bool, "/islab/home/H2/is_contact", self.contact_H2_callback, 10)

        # Optional target contacts (T1 → T5)
        self.target_topics = [f"/islab/targets/T{i}/is_contact" for i in range(1, 6)]
        self.target_contacts = {topic: False for topic in self.target_topics}
        for topic in self.target_topics:
            self.create_subscription(Bool, topic, lambda msg, t=topic: self.target_contact_callback(msg, t), 10)

        # Flag mode control (from GUI)
        self.create_subscription(IslabFlag, "/islab/flag_mode", self.flag_mode_callback, self.qos_profile_sub)

        # ---------------- Publishers ---------------- #
        self.score_pub = self.create_publisher(IslabScore, "/islab/score/status", 10)

        # ---------------- PX4 state ---------------- #
        self.nav_state = None

        # ---------------- Altitude and safety ---------------- #
        self.height = None
        self.height_check_target = 2.0
        self.safe_margin = 0.2
        self.last_height_check_time = time()
        self.height_alarm_delay = 10.0
        self.height_alarm_count = 0
        self.height_alarm_max = 3
        self.height_alarm_triggered = False

        # ---------------- Scoring system ---------------- #
        self.total_score = 0
        self.score_point = 10
        self.alarm_rule_height = 0

        # ---------------- Flight time tracking ---------------- #
        self.time_total = 0.0
        self._last_time_update = time()
        self._offboard_active = False
        self._land_detected = False

        # ---------------- State flags ---------------- #
        self.auto_selected = False   # True if Auto mode selected
        self.auto_running = False    # True if Auto Start flag received

        # ---------------- Cooldowns ---------------- #
        self.hit_cooldown_s = 0.5
        self._last_award_time = {t: 0.0 for t in self.target_topics}

        # ---------------- Home pads ---------------- #
        self._edges_initialized = False
        self.home_H1_contact = False
        self.home_H2_contact = False
        self._prev_h1 = False
        self._prev_h2 = False
        self.h1_takeoff_awarded = False
        self.h2_cooldown_s = 0.5
        self._last_h2_award = 0.0

        # ---------------- Timers ---------------- #
        self.fps = 30
        self.create_timer(1.0 / self.fps, self.main)
        self.create_timer(0.1, self.publish_score_message)  # 10 Hz publisher

    # ============================================================
    # FLAG MODE CALLBACK
    # ============================================================
    def flag_mode_callback(self, msg: IslabFlag):
        """Handles GUI mode flag messages to control start/stop of scoring."""
        if msg.flag_auto:
            self.auto_selected = True
            self.get_logger().info("[FLAG] Auto mode selected")

        if msg.flag_manual:
            self.auto_selected = False
            self.auto_running = False
            self.get_logger().info("[FLAG] Manual mode selected → stop Auto scoring")

        if msg.flag_start_auto:
            self.auto_selected = True
            self.auto_running = True
            self.get_logger().info("[FLAG] Start Auto scoring/timing")
            # Uncomment below if you want to reset score/time when starting
            # self.total_score = 0
            # self.time_total = 0.0

        if msg.flag_stop_auto:
            self.auto_running = False
            self.get_logger().info("[FLAG] Stop Auto scoring/timing")

        if msg.flag_start_manual or msg.flag_stop_manual:
            self.auto_running = False
            self.get_logger().info("[FLAG] Manual start/stop → stop Auto scoring")

    # ============================================================
    # HEIGHT CHECK
    # ============================================================
    def check_height(self):
        """Checks if altitude falls below the safe limit during Auto run."""
        if not self.auto_running or self.height is None:
            return

        if self.height_check_target - self.height > self.safe_margin:
            now = time()
            if now - self.last_height_check_time >= self.height_alarm_delay:
                self.last_height_check_time = now
                if self.height_alarm_count >= self.height_alarm_max:
                    self.height_alarm_triggered = True
                    self.alarm_rule_height = 1
                    self.get_logger().warn("Height alarm triggered!")
                else:
                    self.height_alarm_count += 1
        else:
            self.last_height_check_time = time()
            self.alarm_rule_height = 0

    # ============================================================
    # HOME PAD SCORING
    # ============================================================
    def check_home_pads(self):
        """Detect takeoff (H1) and landing (H2) events for scoring."""
        now = time()

        if not self._edges_initialized:
            self._prev_h1 = self.home_H1_contact
            self._prev_h2 = self.home_H2_contact
            self._edges_initialized = True
            return

        # Only score when Auto is running
        if not self.auto_running:
            self._prev_h1 = self.home_H1_contact
            self._prev_h2 = self.home_H2_contact
            return

        # --- TAKEOFF: H1 contact → no contact ---
        if self._prev_h1 and not self.home_H1_contact:
            if not self.home_H2_contact and not self.h1_takeoff_awarded:
                self.h1_takeoff_awarded = True
                self.total_score += self.score_point
                self.get_logger().info(f"[SCORE] +{self.score_point} TAKEOFF (H1) → total={self.total_score}")
                self.publish_score_message()

        # --- LANDING: H2 no contact → contact ---
        if (not self._prev_h2) and self.home_H2_contact:
            if not self.home_H1_contact and (now - self._last_h2_award >= self.h2_cooldown_s):
                self._last_h2_award = now
                self.total_score += self.score_point
                self.get_logger().info(f"[SCORE] +{self.score_point} LAND (H2) → total={self.total_score}")
                self.publish_score_message()

        self._prev_h1 = self.home_H1_contact
        self._prev_h2 = self.home_H2_contact

    # ============================================================
    # TARGET HIT SCORING
    # ============================================================
    def target_contact_callback(self, msg: Bool, topic: str):
        """Adds score when a target (T1–T5) is hit."""
        prev = self.target_contacts[topic]
        self.target_contacts[topic] = msg.data

        if not self.auto_running:
            return

        if msg.data and not prev:
            now = time()
            if now - self._last_award_time[topic] >= self.hit_cooldown_s:
                self._last_award_time[topic] = now
                self.total_score += self.score_point
                self.get_logger().info(f"[SCORE] +{self.score_point} hit {topic} → total={self.total_score}")
                self.publish_score_message()

    # ============================================================
    # MAIN LOOP
    # ============================================================
    def main(self):
        """Main periodic function for scoring and timing."""
        now = time()
        dt = now - self._last_time_update
        self._last_time_update = now

        # Scoring gate: must be in OFFBOARD + Auto running
        run_gate = self.auto_selected and self.auto_running

        # --- Time accumulation logic ---
        if run_gate:
            if not self._offboard_active:
                self._offboard_active = True
                self._land_detected = False
                self.get_logger().info("OFFBOARD + Auto active → start timing")
            self.time_total += dt

        elif self.nav_state == 18:
            if not self._land_detected:
                self._land_detected = True
                if self._offboard_active:
                    self._offboard_active = False
                    self.get_logger().info("LAND mode detected → pause timing")

        else:
            if self._offboard_active:
                self._offboard_active = False
                self.get_logger().info("OFFBOARD ended → stop timing")

        # Run checks and scoring
        if run_gate:
            self.check_height()
        self.check_home_pads()

    # ============================================================
    # PUBLISH SCORE MESSAGE
    # ============================================================
    def publish_score_message(self):
        """Publishes IslabScore to /islab/score/status."""
        msg = IslabScore()
        now_us = int(time() * 1e6)
        msg.timestamp = now_us
        msg.timestamp_sample = now_us
        msg.total_score = int(self.total_score)
        msg.score_point = int(self.score_point)
        msg.alarm_rule_height = int(self.alarm_rule_height)
        msg.time_total = int(self.time_total)
        self.score_pub.publish(msg)

    # ============================================================
    # CALLBACKS
    # ============================================================
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


# ============================================================
# ENTRYPOINT
# ============================================================
def main():
    rclpy.init()
    node = None
    try:
        node = IslabScoreNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("KeyboardInterrupt → shutting down node")
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
