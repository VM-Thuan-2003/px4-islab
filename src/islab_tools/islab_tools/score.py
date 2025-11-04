#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IslabScoreNode (ROS 2)
----------------------
Responsibilities
- Aggregate mission score and total flight time.
- Count time only when:
    * AUTO is selected and running (via /islab/flag_mode IslabFlag), AND
    * PX4 nav_state == OFFBOARD (14).
- Pause time when:
    * PX4 enters LAND (18), or
    * OFFBOARD is not active, or
    * AUTO is not running/selected.
- Score events:
    * Takeoff from H1 (edge: contact -> no contact).
    * Landing on H2 (edge: no contact -> contact) with cooldown.
    * Target hits T1..T5 with cooldown.
- Height safety rule with delayed alarm while AUTO is running.

Topics (sub)
- /fmu/out/vehicle_status        : px4_msgs/VehicleStatus
- /fmu/out/vehicle_odometry      : px4_msgs/VehicleOdometry
- /UAV/odom                      : nav_msgs/Odometry (optional, for reference)
- /islab/home/H1/is_contact      : std_msgs/Bool
- /islab/home/H2/is_contact      : std_msgs/Bool
- /islab/targets/T{1..5}/is_contact : std_msgs/Bool
- /islab/flag_mode               : islab_msgs/IslabFlag

Topics (pub)
- /islab/score/status            : islab_msgs/IslabScore

Notes
- OFFBOARD = 14, LAND = 18 (PX4 nav_state constants).
- Time is an integer number of seconds in IslabScore for compatibility.
"""

from __future__ import annotations

from time import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)

from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry, VehicleStatus
from islab_msgs.msg import IslabScore, IslabFlag


# ------------------------ PX4 nav_state constants ------------------------ #
NAV_OFFBOARD = 14
NAV_LAND = 18


class IslabScoreNode(Node):
    def __init__(self) -> None:
        super().__init__("islab_score_node")
        self.get_logger().info("IslabScore node started")

        # ===================== QoS configuration ===================== #
        # Best-effort is sufficient for UI-like telemetry.
        self.qos_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ========================= Subscriptions ========================= #
        # PX4 state/pose
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self._cb_vehicle_status, self.qos_sub
        )
        self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self._cb_vehicle_odom, self.qos_sub
        )
        # Optional external odometry (not used for scoring, kept for potential extensions)
        self.create_subscription(Odometry, "/UAV/odom", self._cb_odom_ext, 10)

        # Home pads
        self.create_subscription(Bool, "/islab/home/H1/is_contact", self._cb_home_h1, 10)
        self.create_subscription(Bool, "/islab/home/H2/is_contact", self._cb_home_h2, 10)

        # Targets T1..T5 (edge-based scoring with cooldown)
        self.target_topics = [f"/islab/targets/T{i}/is_contact" for i in range(1, 6)]
        self.target_contacts: Dict[str, bool] = {t: False for t in self.target_topics}
        self._last_award_time: Dict[str, float] = {t: 0.0 for t in self.target_topics}
        for topic in self.target_topics:
            # Bind topic into lambda default arg to avoid late-binding pitfall
            self.create_subscription(
                Bool, topic, lambda msg, t=topic: self._cb_target_contact(msg, t), 10
            )

        # GUI flags (AUTO/MANUAL selection & start/stop)
        self.create_subscription(IslabFlag, "/islab/flag_mode", self._cb_flag_mode, self.qos_sub)

        # ========================== Publisher ========================== #
        self.score_pub = self.create_publisher(IslabScore, "/islab/score/status", 10)

        # ======================== Internal state ======================= #
        # PX4 mode state
        self.nav_state: Optional[int] = None

        # Height / safety rule
        self.height: Optional[float] = None  # meters (up positive)
        self.height_check_target: float = 2.0  # required minimum height
        self.safe_margin: float = 0.2          # hysteresis margin
        self.height_alarm_delay: float = 10.0  # seconds below threshold before alarm
        self.height_alarm_count: int = 0
        self.height_alarm_max: int = 3
        self.height_alarm_triggered: bool = False
        self._last_height_check_time: float = time()
        self.alarm_rule_height: int = 0  # exported in IslabScore (0/1)

        # Scoring values
        self.total_score: int = 0
        self.score_point: int = 10

        # Flight-time accounting (seconds)
        self.time_total: float = 0.0
        self._last_time_update: float = time()
        self._offboard_timing_active: bool = False
        self._land_seen_edge: bool = False

        # GUI flag state
        self.auto_selected: bool = False
        self.auto_running: bool = False

        # Home pad contact state, edges, and cooldowns
        self.home_H1_contact: bool = False
        self.home_H2_contact: bool = False
        self._prev_h1: bool = False
        self._prev_h2: bool = False
        self._edges_initialized: bool = False
        self.h1_takeoff_awarded: bool = False
        self.h2_cooldown_s: float = 0.5
        self._last_h2_award: float = 0.0

        # Target hit cooldown (per target) in seconds
        self.hit_cooldown_s: float = 0.5

        # =========================== Timers =========================== #
        # Main loop (scoring + timing + checks)
        self._fps: float = 30.0
        self.create_timer(1.0 / self._fps, self._main)

        # Score publisher (10 Hz is enough for UI)
        self.create_timer(0.1, self._publish_score)

    # ------------------------------------------------------------------ #
    # Subscriptions (callbacks)
    # ------------------------------------------------------------------ #

    def _cb_flag_mode(self, msg: IslabFlag) -> None:
        """Handle GUI flags for AUTO/MANUAL selection and start/stop."""
        if msg.flag_auto:
            self.auto_selected = True
            self.get_logger().info("[FLAG] Auto selected")

        if msg.flag_manual:
            self.auto_selected = False
            self.auto_running = False
            self.get_logger().info("[FLAG] Manual selected → stop Auto")

        if msg.flag_start_auto:
            self.auto_selected = True
            self.auto_running = True
            self.get_logger().info("[FLAG] Start Auto scoring/timing")
            # Uncomment if you want to reset score/time each time Auto starts
            # self.total_score = 0
            # self.time_total = 0.0

        if msg.flag_stop_auto:
            self.auto_running = False
            self.get_logger().info("[FLAG] Stop Auto scoring/timing")

        if msg.flag_start_manual or msg.flag_stop_manual:
            self.auto_running = False
            self.get_logger().info("[FLAG] Manual start/stop → stop Auto timing")

    def _cb_vehicle_status(self, msg: VehicleStatus) -> None:
        """Track PX4 nav_state (OFFBOARD=14, LAND=18) used for the timing gate/pause."""
        self.nav_state = int(msg.nav_state)

    def _cb_vehicle_odom(self, msg: VehicleOdometry) -> None:
        """PX4 NED: z is down. Height (up-positive) = -position[2]."""
        try:
            self.height = float(-msg.position[2])
        except Exception as e:
            self.get_logger().error(f"[VehicleOdometry] {e}")

    def _cb_odom_ext(self, msg: Odometry) -> None:
        """External odometry (optional). Not used for scoring, kept for future needs."""
        # Example:
        # x = msg.pose.pose.position.x
        # y = msg.pose.pose.position.y
        # z = msg.pose.pose.position.z
        pass

    def _cb_home_h1(self, msg: Bool) -> None:
        self.home_H1_contact = bool(msg.data)

    def _cb_home_h2(self, msg: Bool) -> None:
        self.home_H2_contact = bool(msg.data)

    def _cb_target_contact(self, msg: Bool, topic: str) -> None:
        """Edge detector + cooldown for T1..T5 hits while Auto is running."""
        prev = self.target_contacts[topic]
        cur = bool(msg.data)
        self.target_contacts[topic] = cur

        if not self.auto_running:
            return

        # Rising edge with cooldown → award
        if cur and not prev:
            now = time()
            if now - self._last_award_time[topic] >= self.hit_cooldown_s:
                self._last_award_time[topic] = now
                self.total_score += self.score_point
                self.get_logger().info(f"[SCORE] +{self.score_point} target {topic} → total={self.total_score}")
                self._publish_score()

    # ------------------------------------------------------------------ #
    # Core logic
    # ------------------------------------------------------------------ #

    def _main(self) -> None:
        """
        Periodic loop:
        - Accumulate time only if: AUTO is selected & running AND PX4 is in OFFBOARD (14).
        - Pause timing when LAND (18), or whenever OFFBOARD is not active, or AUTO not running.
        - Run height safety check only while timing is active.
        - Home pad scoring always evaluates edges, but awards only while Auto is running.
        """
        now = time()
        dt = now - self._last_time_update
        self._last_time_update = now

        is_offboard = (self.nav_state == NAV_OFFBOARD)
        is_land = (self.nav_state == NAV_LAND)

        # Time-gate: OFFBOARD + Auto flags
        run_gate = self.auto_selected and self.auto_running and is_offboard

        # ---- Timing state machine ----
        if run_gate:
            if not self._offboard_timing_active:
                self._offboard_timing_active = True
                self._land_seen_edge = False
                self.get_logger().info("OFFBOARD + Auto active → start/resume timing")
            self.time_total += dt
        else:
            if self._offboard_timing_active:
                self._offboard_timing_active = False
                reason = "LAND mode" if is_land else "leaving OFFBOARD / AUTO not running"
                self.get_logger().info(f"{reason} → pause timing")

            if is_land and not self._land_seen_edge:
                self._land_seen_edge = True  # mark LAND edge (optional)

        # ---- Safety & scoring ----
        if run_gate:
            self._check_height()

        self._check_home_pads()

    def _check_height(self) -> None:
        """
        Height safety rule:
        - While AUTO timing is active, if height stays below (height_check_target - safe_margin)
          for >= height_alarm_delay, increment alarm counter.
        - Once exceeded height_alarm_max, trigger alarm (exported via alarm_rule_height = 1).
        """
        if self.height is None:
            return

        below = (self.height_check_target - self.height) > self.safe_margin
        now = time()

        if below:
            if now - self._last_height_check_time >= self.height_alarm_delay:
                self._last_height_check_time = now
                if self.height_alarm_count >= self.height_alarm_max:
                    if not self.height_alarm_triggered:
                        self.height_alarm_triggered = True
                        self.alarm_rule_height = 1
                        self.get_logger().warn("Height alarm triggered!")
                else:
                    self.height_alarm_count += 1
        else:
            # Reset window while above threshold
            self._last_height_check_time = now
            self.alarm_rule_height = 0

    def _check_home_pads(self) -> None:
        """
        Home pads logic (edge-based):
        - TAKEOFF (H1): contact -> no contact  → +score (once per mission start).
        - LAND   (H2): no contact -> contact  → +score (with small cooldown).
        Awards require Auto running (enforced in _award_* helpers).
        """
        now = time()

        if not self._edges_initialized:
            self._prev_h1 = self.home_H1_contact
            self._prev_h2 = self.home_H2_contact
            self._edges_initialized = True
            return

        # Detect edges
        h1_rising = (not self._prev_h1) and self.home_H1_contact
        h1_falling = self._prev_h1 and (not self.home_H1_contact)
        h2_rising = (not self._prev_h2) and self.home_H2_contact
        # h2_falling = self._prev_h2 and (not self.home_H2_contact)  # not used

        # TAKEOFF award: H1 edge contact -> no contact (falling)
        if h1_falling:
            self._award_takeoff_h1()

        # LAND award: H2 edge no contact -> contact (rising) with cooldown
        if h2_rising:
            self._award_land_h2(now)

        # Update edges
        self._prev_h1 = self.home_H1_contact
        self._prev_h2 = self.home_H2_contact

    def _award_takeoff_h1(self) -> None:
        """Award takeoff (H1). Only once until reset; requires Auto running."""
        if not self.auto_running:
            return
        if self.home_H2_contact:
            return  # ignore if touching H2
        if self.h1_takeoff_awarded:
            return

        self.h1_takeoff_awarded = True
        self.total_score += self.score_point
        self.get_logger().info(f"[SCORE] +{self.score_point} TAKEOFF (H1) → total={self.total_score}")
        self._publish_score()

    def _award_land_h2(self, now: float) -> None:
        """Award landing (H2) with cooldown; requires Auto running."""
        if not self.auto_running:
            return
        if self.home_H1_contact:
            return  # ignore if touching H1
        if now - self._last_h2_award < self.h2_cooldown_s:
            return

        self._last_h2_award = now
        self.total_score += self.score_point
        self.get_logger().info(f"[SCORE] +{self.score_point} LAND (H2) → total={self.total_score}")
        self._publish_score()

    # ------------------------------------------------------------------ #
    # Publishing
    # ------------------------------------------------------------------ #

    def _publish_score(self) -> None:
        """Publish IslabScore (10 Hz timer + on award events)."""
        msg = IslabScore()
        now_us = int(time() * 1e6)
        msg.timestamp = now_us
        msg.timestamp_sample = now_us
        msg.total_score = int(self.total_score)
        msg.score_point = int(self.score_point)
        msg.alarm_rule_height = int(self.alarm_rule_height)
        msg.time_total = int(self.time_total)  # seconds (rounded down)
        self.score_pub.publish(msg)


# ============================== Entrypoint ============================== #

def main() -> None:
    rclpy.init()
    node: Optional[IslabScoreNode] = None
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
