#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IslabScoreNode (ROS 2)
----------------------
Timing rules
- Time starts counting when:
    * AUTO flag becomes true (flag_auto rising edge), OR
    * flag_start_auto is pressed.
- After that, time increases continuously every loop WHILE the session is active.
- The session (and timing) stops COMPLETELY when:
    * PX4 nav_state == LAND (18), OR
    * A STOP flag is received (flag_stop_auto, manual flags, or AUTO turned off).
- Once stopped, time does NOT resume automatically.
  To start a new timing session, AUTO must be enabled again or flag_start_auto used.

Score rules (logical points)
- TAKEOFF from H1:         2.5 points  (contact -> no contact, once per session)
- LAND on H2:              2.5 points  (no contact -> contact, cooldown)
- Target hits T1..T5:      5.0 points  (each hit, cooldown)
- Height safety rule checked while session is active.

Because IslabScore.total_score is an int, we use a scaled integer:
- SCORE_SCALE = 10
- Stored score = logical_score * SCORE_SCALE
  (e.g. 2.5 -> 25, 5.0 -> 50)

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
NAV_LAND = 18


# ------------------------ Score configuration class ---------------------- #
class ScoreConfig:
    """
    Score configuration.

    Logical points:
      - TAKEOFF_H1_POINTS: score for takeoff from H1.
      - LAND_H2_POINTS:    score for landing on H2.
      - TARGET_HIT_POINTS: score for each hit on T1..T5.

    Stored as:
      stored_score = logical_score * SCORE_SCALE
    """

    SCORE_SCALE: int = 10

    TAKEOFF_H1_POINTS: float = 2.5
    LAND_H2_POINTS: float = 2.5
    TARGET_HIT_POINTS: float = 5.0

    TAKEOFF_H1: int = int(TAKEOFF_H1_POINTS * SCORE_SCALE)   # 25
    LAND_H2: int = int(LAND_H2_POINTS * SCORE_SCALE)         # 25
    TARGET_HIT: int = int(TARGET_HIT_POINTS * SCORE_SCALE)   # 50


class IslabScoreNode(Node):
    def __init__(self) -> None:
        super().__init__("islab_score_node")
        self.get_logger().info("IslabScore node started")

        # ===================== QoS configuration ===================== #
        self.qos_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ========================= Subscriptions ========================= #
        # PX4 state/pose
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self._cb_vehicle_status,
            self.qos_sub,
        )
        self.create_subscription(
            VehicleOdometry,
            "/fmu/out/vehicle_odometry",
            self._cb_vehicle_odom,
            self.qos_sub,
        )

        # Optional external odometry (not used for scoring)
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
                Bool,
                topic,
                lambda msg, t=topic: self._cb_target_contact(msg, t),
                10,
            )

        # GUI flags (AUTO/MANUAL, start/stop)
        self.create_subscription(
            IslabFlag,
            "/islab/flag_mode",
            self._cb_flag_mode,
            self.qos_sub,
        )

        # ========================== Publisher ========================== #
        self.score_pub = self.create_publisher(IslabScore, "/islab/score/status", 10)

        # ======================== Internal state ======================= #
        # PX4 nav_state
        self.nav_state: Optional[int] = None

        # Height / safety rule
        self.height: Optional[float] = None  # meters (up positive)
        self.height_check_target: float = 2.0
        self.safe_margin: float = 0.2
        self.height_alarm_delay: float = 10.0
        self.height_alarm_count: int = 0
        self.height_alarm_max: int = 3
        self.height_alarm_triggered: bool = False
        self._last_height_check_time: float = time()
        self.alarm_rule_height: int = 0  # exported (0/1)

        # Scoring values (scaled integer)
        self.total_score_scaled: int = 0
        self.score_point_scaled: int = ScoreConfig.TARGET_HIT

        self.scored_H1 = False
        self.scored_H2 = False

        # Timing
        self.time_total: float = 0.0        # seconds (float, internal)
        self._last_time_update: float = time()

        # AUTO flag from GUI
        self.auto_selected: bool = False    # reflect flag_auto

        # Session state: when True, time is counting continuously
        self.session_active: bool = False
        self._land_latched: bool = False    # avoid multi-log on LAND

        # Home pad / edges / cooldown
        self.home_H1_contact: bool = False
        self.home_H2_contact: bool = False
        self._prev_h1: bool = False
        self._prev_h2: bool = False
        self._edges_initialized: bool = False
        self.h1_takeoff_awarded: bool = False
        self.h2_cooldown_s: float = 2.0
        self._last_h2_award: float = 0.0

        # Target cooldown (per target)
        self.hit_cooldown_s: float = 0.5

        # =========================== Timers =========================== #
        # Main loop
        self._fps: float = 30.0
        self.create_timer(1.0 / self._fps, self._main)

        # Periodic publisher (UI smoothing)
        self.create_timer(0.1, self._publish_score)

    # ------------------------------------------------------------------ #
    # Callbacks
    # ------------------------------------------------------------------ #

    def _cb_flag_mode(self, msg: IslabFlag) -> None:
        """
        Timing rule (the behavior you asked):

        - Thời gian bắt đầu đếm khi cờ AUTO = true (rising edge) hoặc nhấn START.
        - Khi đã bắt đầu, thời gian đếm liên tục (mỗi vòng timer) cho đến khi:
            * PX4 vào LAND, hoặc
            * Có cờ STOP / MANUAL / AUTO tắt.
        - Khi đã dừng session → không resume, phải start session mới (AUTO lên lại
          hoặc flag_start_auto).
        """
        prev_auto = self.auto_selected

        # Update AUTO/MANUAL flags
        if msg.flag_auto:
            self.auto_selected = True
        if msg.flag_manual:
            self.auto_selected = False

        # STOP conditions from flags → end session
        if msg.flag_stop_auto or msg.flag_start_manual or msg.flag_stop_manual:
            if self.session_active:
                self.get_logger().info("[FLAG] Stop/Manual → end timing session")
            self.session_active = False

        # Start new session when:
        # - AUTO has rising edge (false -> true), or
        # - flag_start_auto is pressed.
        start_new_session = False

        if msg.flag_auto and not prev_auto:
            start_new_session = True

        if msg.flag_start_auto:
            start_new_session = True

        if start_new_session:
            self.session_active = True
            self._land_latched = False

            # Reset timer to 0 at the beginning of session
            self.time_total = 0.0
            self._last_time_update = time()

            # Optional: reset score per session (comment out if you want accumulate across sessions)
            # self.total_score_scaled = 0

            # Reset pad scoring & height alarm
            self.h1_takeoff_awarded = False
            self.height_alarm_count = 0
            self.height_alarm_triggered = False
            self.alarm_rule_height = 0

            self.get_logger().info("[FLAG] New timing session started → time_total reset to 0")

        # Nếu AUTO bị tắt trong khi session đang active → cũng end session
        if not self.auto_selected and self.session_active:
            self.get_logger().info("[FLAG] AUTO disabled → end timing session")
            self.session_active = False

    def _cb_vehicle_status(self, msg: VehicleStatus) -> None:
        """Track PX4 nav_state (LAND is used to stop timing)."""
        self.nav_state = int(msg.nav_state)

    def _cb_vehicle_odom(self, msg: VehicleOdometry) -> None:
        """PX4 NED: z is down. Height (up-positive) = -position[2]."""
        try:
            self.height = float(-msg.position[2])
        except Exception as e:
            self.get_logger().error(f"[VehicleOdometry] {e}")

    def _cb_odom_ext(self, msg: Odometry) -> None:
        """External odometry (currently unused)."""
        pass

    def _cb_home_h1(self, msg: Bool) -> None:
        self.home_H1_contact = bool(msg.data)

    def _cb_home_h2(self, msg: Bool) -> None:
        self.home_H2_contact = bool(msg.data)

    def _cb_target_contact(self, msg: Bool, topic: str) -> None:
        """Edge detector + cooldown for T1..T5 hits, only while session is active."""
        prev = self.target_contacts[topic]
        cur = bool(msg.data)
        self.target_contacts[topic] = cur

        if not self.session_active:
            return

        if cur and not prev:
            now = time()
            if now - self._last_award_time[topic] >= self.hit_cooldown_s:
                self._last_award_time[topic] = now
                self.total_score_scaled += ScoreConfig.TARGET_HIT
                logical_score = self.total_score_scaled / ScoreConfig.SCORE_SCALE
                self.get_logger().info(
                    f"[SCORE] +{ScoreConfig.TARGET_HIT_POINTS} target {topic} → total={logical_score}"
                )
                self._publish_score()

    # ------------------------------------------------------------------ #
    # Main loop
    # ------------------------------------------------------------------ #

    def _main(self) -> None:
        """
        Periodic loop:

        - If session_active:
            * time_total += dt (always, continuous)
            * If nav_state == LAND:
                  - stop session permanently (session_active = False).
        - Height safety rule active while session_active.
        - Home pads always checked; awards only if session_active.
        """
        now = time()
        dt = now - self._last_time_update
        self._last_time_update = now

        is_land = (self.nav_state == NAV_LAND)

        # LAND → stop session permanently
        if self.session_active and is_land and not self._land_latched:
            self._land_latched = True
            self.session_active = False
            self.get_logger().info("PX4 in LAND → stop timing session permanently")

        # Timing: continuous while session_active
        if self.session_active:
            self.time_total += dt

        # Height rule active only when session_active
        if self.session_active:
            self._check_height()

        # Home pad scoring (edge-based)
        self._check_home_pads()

    def _check_height(self) -> None:
        """
        Height safety rule:
        - While session is active, if height stays below
          (height_check_target - safe_margin) for >= height_alarm_delay,
          increment alarm counter.
        - Once exceeded height_alarm_max, trigger alarm.
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
            self._last_height_check_time = now
            self.alarm_rule_height = 0

    def _check_home_pads(self) -> None:
        """
        Home pads logic (edge-based):
        - TAKEOFF (H1): contact -> no contact  → +2.5 points (once per session).
        - LAND   (H2): no contact -> contact  → +2.5 points (with cooldown).
        Awards require session_active == True.
        """
        now = time()

        if not self._edges_initialized:
            self._prev_h1 = self.home_H1_contact
            self._prev_h2 = self.home_H2_contact
            self._edges_initialized = True
            return

        h1_falling = self._prev_h1 and (not self.home_H1_contact)
        h2_rising = (not self._prev_h2) and self.home_H2_contact

        print(f"h1:{h1_falling} - h2:{h2_rising}")

        if h2_rising:
            self._award_land_h2(now)
            
        if h1_falling:
            self._award_takeoff_h1()

        self._prev_h1 = self.home_H1_contact
        self._prev_h2 = self.home_H2_contact

    def _award_takeoff_h1(self) -> None:
        """Award takeoff (H1). Only once per session; requires session_active."""
        if not self.session_active:
            return
        if self.home_H2_contact:
            return
        if self.h1_takeoff_awarded:
            return
        
        self.scored_H1 = True
        self.h1_takeoff_awarded = True
        self._prev_h2 = False
        self.total_score_scaled += ScoreConfig.TAKEOFF_H1
        logical_score = self.total_score_scaled / ScoreConfig.SCORE_SCALE
        self.get_logger().info(
            f"[SCORE] +{ScoreConfig.TAKEOFF_H1_POINTS} TAKEOFF (H1) → total={logical_score}"
        )
        self._publish_score()

    def _award_land_h2(self, now: float) -> None:
        """Award landing (H2) with cooldown; requires session_active."""
        # if not self.session_active:
        #     return
        if not self.scored_H1:
            return
        if self.scored_H2:
            return
        if now - self._last_h2_award < self.h2_cooldown_s:
            return
        
        self.scored_H2 = True
        self._last_h2_award = now
        self.total_score_scaled += ScoreConfig.LAND_H2
        logical_score = self.total_score_scaled / ScoreConfig.SCORE_SCALE
        self.get_logger().info(
            f"[SCORE] +{ScoreConfig.LAND_H2_POINTS} LAND (H2) → total={logical_score}"
        )
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

        msg.total_score = int(self.total_score_scaled)
        msg.score_point = int(self.score_point_scaled)
        msg.alarm_rule_height = int(self.alarm_rule_height)
        msg.time_total = int(self.time_total)  # seconds

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
