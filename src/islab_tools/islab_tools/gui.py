#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Islab Mode + Status/Score GUI (PyQt5 + ROS 2)

Layout (no tabs)
----------------
[ BIG Overview image (left, auto-scale) ]  |  [ Control panel (right) ]
"""

import sys
import time
import threading
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)

from islab_msgs.msg import IslabChangeMode, IslabScore
from islab_msgs.msg import IslabFlag
from px4_msgs.msg import VehicleStatus, LogMessage, VehicleOdometry
from sensor_msgs.msg import Image as RosImage
from cv_bridge import CvBridge

from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QLabel,
    QPushButton,
    QComboBox,
    QDoubleSpinBox,
    QCheckBox,
    QGroupBox,
    QHBoxLayout,
    QTextEdit,
    QSizePolicy,
)
from PyQt5.QtCore import QTimer, pyqtSignal, Qt, QSize
from PyQt5.QtGui import QFont, QImage, QPixmap

# --- Flight Modes (simplified) ---
MODES = [
    ("POSHOLD", 2),   # manual
    ("OFFBOARD", 4),  # auto
    ("LAND", 6),      # manual/stop
]

# --- Operation handle types ---
HANDLE = [
    ("MODE", 0),      # Normal mode change
    ("TAKEOFF", 1),   # Takeoff action
    ("ARM", 2),       # Arm/disarm action
]

# --- PX4 Navigation state mapping (id -> name) ---
NAV_STATE_NAMES = {
    0:  "MANUAL", 1:  "ALTCTL", 2:  "POSCTL", 3:  "AUTO_MISSION", 4:  "AUTO_LOITER",
    5:  "AUTO_RTL", 6:  "POSITION_SLOW", 7:  "FREE5", 8:  "ALTITUDE_CRUISE",
    9:  "FREE3", 10: "ACRO", 11: "FREE2", 12: "DESCEND", 13: "TERMINATION",
    14: "OFFBOARD", 15: "STAB", 16: "FREE1", 17: "AUTO_TAKEOFF", 18: "AUTO_LAND",
    19: "AUTO_FOLLOW_TARGET", 20: "AUTO_PRECLAND", 21: "ORBIT",
    22: "AUTO_VTOL_TAKEOFF", 23: "EXTERNAL1", 24: "EXTERNAL2", 25: "EXTERNAL3",
    26: "EXTERNAL4", 27: "EXTERNAL5", 28: "EXTERNAL6", 29: "EXTERNAL7",
    30: "EXTERNAL8", 31: "MAX",
}


# ============================= ROS Node ============================= #
class IslabChangeModePublisher(Node):
    """
    Publishes IslabChangeMode/IslabFlag; subscribes VehicleStatus/Log/Score/Odometry/Image.
    """

    def __init__(self):
        super().__init__("islab_change_mode_publisher")

        # ---------- UI parameters (ROS 2) ----------
        self.declare_parameter("always_on_top", True)
        self.declare_parameter("font_scale", 1.4)
        self.declare_parameter("window_pos", "top-right")  # top-right | top-left
        self.declare_parameter("window_width", 960)
        self.declare_parameter("window_height", 720)
        self.declare_parameter("show_alarm_rule_height", False)

        self.ui_cfg = dict(
            always_on_top = bool(self.get_parameter("always_on_top").value),
            font_scale     = float(self.get_parameter("font_scale").value),
            window_pos     = str(self.get_parameter("window_pos").value).lower(),
            window_width   = int(self.get_parameter("window_width").value),
            window_height  = int(self.get_parameter("window_height").value),
            show_alarm_rule_height = bool(self.get_parameter("show_alarm_rule_height").value),
        )

        # --- QoS profiles ---
        self.qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        status_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.qos_profile_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # --- Publishers ---
        self.change_mode_pub = self.create_publisher(
            IslabChangeMode, "islab/change_mode", self.qos_profile_pub
        )
        self.flag_pub = self.create_publisher(
            IslabFlag, "/islab/flag_mode", self.qos_profile_pub
        )

        # --- State and GUI callbacks ---
        self.vehicle_status = None
        self.status_update_callback = None
        self.log_message_callback = None

        self.total_score = None
        self.score_point = None
        self.time_total = None
        self.alarm_rule_height = None
        self.score_update_callback = None

        self.height = None
        self.height_update_callback = None

        # Image callback for Overview
        self.image_update_callback = None
        self.bridge = CvBridge()

        # --- Subscriptions ---
        self.create_subscription(IslabScore, "/islab/score/status", self.score_callback, 10)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.vehicle_status_cb, status_qos)
        self.create_subscription(LogMessage, "/fmu/out/log_message", self.log_message_cb, status_qos)
        self.create_subscription(VehicleOdometry, "/fmu/out/vehicle_odometry", self.status_odom_callback, self.qos_profile_sub)

        # Overview image (best-effort)
        self.create_subscription(
            RosImage, "/islab/overview/image_raw", self.image_cb,
            QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            )
        )

        self.get_logger().info("IslabChangeModePublisher started")

    # -------------------- Publishers -------------------- #
    def send_mode(self, mode: int, altitude: float, arm: int, handle: int):
        msg = IslabChangeMode()
        now_us = int(time.time() * 1e6)
        msg.timestamp = now_us
        msg.timestamp_sample = now_us
        msg.mode = int(mode)
        msg.source = 0
        msg.altitude = float(altitude)
        msg.arm = int(arm)
        msg.handel = int(handle)
        self.change_mode_pub.publish(msg)
        self.get_logger().info(
            f"Published IslabChangeMode: mode={msg.mode}, source={msg.source}, "
            f"alt={msg.altitude}, arm={msg.arm}, handle={msg.handel}"
        )

    def send_flag_auto(
        self, *,
        flag_auto=False,
        flag_start_auto=False,
        flag_stop_auto=False,
        flag_manual=False,
        flag_start_manual=False,
        flag_stop_manual=False,
    ):
        m = IslabFlag()
        now_us = int(time.time() * 1e6)
        m.timestamp = now_us
        m.timestamp_sample = now_us

        m.flag_auto = bool(flag_auto)
        m.flag_start_auto = bool(flag_start_auto)
        m.flag_stop_auto = bool(flag_stop_auto)
        m.flag_manual = bool(flag_manual)
        m.flag_start_manual = bool(flag_start_manual)
        m.flag_stop_manual = bool(flag_stop_manual)

        self.flag_pub.publish(m)
        self.get_logger().info(
            "Published IslabFlag("
            f"auto={m.flag_auto}, start_auto={m.flag_start_auto}, stop_auto={m.flag_stop_auto}, "
            f"manual={m.flag_manual}, start_manual={m.flag_start_manual}, stop_manual={m.flag_stop_manual})"
        )

    # -------------------- Subscribers -------------------- #
    def score_callback(self, msg: IslabScore):
        self.total_score = msg.total_score
        if hasattr(msg, "score_point"):
            self.score_point = msg.score_point
        self.time_total = msg.time_total
        self.alarm_rule_height = msg.alarm_rule_height

        if self.score_update_callback:
            self.score_update_callback(
                {
                    "total_score": self.total_score,
                    "time_total": self.time_total,
                    "alarm_rule_height": self.alarm_rule_height,
                }
            )

    def vehicle_status_cb(self, msg: VehicleStatus):
        self.vehicle_status = msg
        if self.status_update_callback:
            self.status_update_callback(msg)

    def log_message_cb(self, msg: LogMessage):
        text = bytes(msg.text).split(b"\x00", 1)[0].decode(errors="ignore")
        sev = msg.severity
        if sev <= 2:
            level = "critical"
        elif sev == 3:
            level = "error"
        elif sev == 4:
            level = "warning"
        elif sev in (5, 6):
            level = "info"
        else:
            level = "debug"

        if self.log_message_callback:
            self.log_message_callback(text, level)

    def status_odom_callback(self, msg: VehicleOdometry):
        """PX4 NED: height = -position[2]."""
        try:
            self.height = float(-msg.position[2])
            if self.height_update_callback:
                self.height_update_callback(self.height)
        except Exception as e:
            self.get_logger().error(f"[Status Odometry] {e}")

    def image_cb(self, msg: RosImage):
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if self.image_update_callback:
                self.image_update_callback(frame_bgr)
        except Exception as e:
            self.get_logger().warning(f"image_cb error: {e}")


# ============================= GUI ============================= #
class ModeChangerGUI(QWidget):
    """Single-window GUI: left big image, right control panel."""

    # Thread-safe signals from ROS thread -> Qt main thread
    vehicle_status_signal = pyqtSignal(object)
    log_message_signal = pyqtSignal(str, str)
    score_signal = pyqtSignal(object)
    height_signal = pyqtSignal(float)
    image_signal = pyqtSignal(object)  # np.ndarray (BGR)

    def __init__(self, ros_node: IslabChangeModePublisher):
        super().__init__()
        self.ros_node = ros_node
        cfg = ros_node.ui_cfg

        self.setWindowTitle("Islab Mode + Status/Score Monitor")

        # Always-on-top + size
        if cfg.get("always_on_top", True):
            self.setWindowFlag(Qt.WindowStaysOnTopHint, True)
        self.resize(cfg.get("window_width", 960), cfg.get("window_height", 720))

        # ================== Root (no tabs) ================== #
        outer = QHBoxLayout(self)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(10)

        # ---------- LEFT: Big image ----------
        left = QVBoxLayout()
        left.setContentsMargins(0, 0, 0, 0)
        left.setSpacing(6)

        self.image_label = QLabel("Waiting for /islab/overview/image_raw ...")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.image_label.setMinimumSize(QSize(480, 360))
        self.image_label.setStyleSheet(
            "QLabel { background:#0b0b0b; color:#9aa0a6; border:1px solid #2d2d2d; border-radius:8px; }"
        )
        self._last_qimage = None
        left.addWidget(self.image_label, stretch=1)

        # ---------- RIGHT: Control panel (same as trước) ----------
        right = QVBoxLayout()
        right.setContentsMargins(0, 0, 0, 0)
        right.setSpacing(8)

        # ---- TOP BAR: status + big score/time ----
        topbar = QHBoxLayout()
        topbar.setContentsMargins(0, 0, 0, 0)
        topbar.setSpacing(10)

        # Doc-style status label
        self.status_label = QLabel("Vehicle status: ...")
        self.status_label.setTextFormat(Qt.RichText)
        self.status_label.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
        self.status_label.setStyleSheet(
            "QLabel { border:1px solid #e5e7eb; border-radius:8px; padding:8px 10px; background:#fafafa; }"
        )
        self.status_label.setMinimumHeight(100)

        # Big fonts for score/time
        base_pts = 20
        big_pts = int(base_pts * float(cfg.get("font_scale", 1.4)))
        bold_font = QFont()
        bold_font.setPointSize(big_pts)
        bold_font.setBold(True)

        mono_font = QFont()
        mono_font.setPointSize(big_pts)
        mono_font.setBold(True)
        mono_font.setFamily("Consolas")

        self.big_score = QLabel("Score: 0")
        self.big_score.setFont(bold_font)
        self.big_score.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.big_score.setStyleSheet("color:#00aa00;")

        self.big_time = QLabel("Time: 00:00")
        self.big_time.setFont(mono_font)
        self.big_time.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.big_time.setStyleSheet("color:#0077cc;")

        topbar.addWidget(self.status_label, 1)
        rightbox = QVBoxLayout()
        rightbox.setSpacing(0)
        rightbox.setContentsMargins(0, 0, 0, 0)
        rightbox.addWidget(self.big_score, 0, Qt.AlignRight)
        rightbox.addWidget(self.big_time, 0, Qt.AlignRight)
        topbar.addLayout(rightbox, 0)
        right.addLayout(topbar)

        # ---- MODE BANNER (large) ----
        self.mode_banner = QLabel("")
        banner_font = QFont()
        banner_font.setPointSize(big_pts + 2)
        banner_font.setBold(True)
        self.mode_banner.setFont(banner_font)
        self.mode_banner.setAlignment(Qt.AlignCenter)
        self.mode_banner.setStyleSheet("color:#444;")
        right.addWidget(self.mode_banner)

        # ---- Optional: Alarm rule line ----
        if cfg.get("show_alarm_rule_height", False):
            self.alarm_label = QLabel("<b>Alarm Rule Height:</b> ...")
            self.alarm_label.setAlignment(Qt.AlignRight)
            right.addWidget(self.alarm_label)

        # ---- Height Panel ----
        height_group = QGroupBox("Altitude")
        height_layout = QVBoxLayout()
        self.height_label = QLabel("<b>Height:</b> ... m")
        height_layout.addWidget(self.height_label)
        height_group.setLayout(height_layout)
        right.addWidget(height_group)

        # ---- Mode Selection ----
        right.addWidget(QLabel("Select Mode:"))
        self.mode_combo = QComboBox()
        for name, val in MODES:
            self.mode_combo.addItem(f"{name} ({val})", val)
        right.addWidget(self.mode_combo)

        self.last_mode_selected = None

        # ---- Altitude ----
        right.addWidget(QLabel("Altitude (m):"))
        self.altitude_spin = QDoubleSpinBox()
        self.altitude_spin.setRange(-100.0, 500.0)
        self.altitude_spin.setSingleStep(0.5)
        self.altitude_spin.setValue(2.0)
        right.addWidget(self.altitude_spin)

        # ---- Arm State ----
        arm_group = QGroupBox("Arm State")
        arm_layout = QHBoxLayout()
        self.arm_checkbox = QCheckBox("Arm (checked=ARM, unchecked=DISARM)")
        arm_layout.addWidget(self.arm_checkbox)
        arm_group.setLayout(arm_layout)
        right.addWidget(arm_group)

        # ---- Handle ----
        right.addWidget(QLabel("Select Handle:"))
        self.handle_combo = QComboBox()
        for name, val in HANDLE:
            self.handle_combo.addItem(f"{name} ({val})", val)
        right.addWidget(self.handle_combo)

        # ---- Buttons row ----
        btn_row = QHBoxLayout()
        btn_row.setSpacing(8)

        self.apply_btn = QPushButton("Apply (Mode + Flag)")
        self.apply_btn.setToolTip("Send IslabChangeMode then IslabFlag based on selection")
        self.apply_btn.clicked.connect(self.on_apply_clicked)
        btn_row.addWidget(self.apply_btn)

        self.starting = False
        self.start_stop_btn = QPushButton("Start")
        self.start_stop_btn.setToolTip("Start/Stop current selected mode via IslabFlag")
        self.start_stop_btn.clicked.connect(self.on_start_stop_clicked)
        btn_row.addWidget(self.start_stop_btn)

        right.addLayout(btn_row)

        # ---- PX4 Log ----
        right.addWidget(QLabel("PX4 Log:"))
        self.log_area = QTextEdit()
        self.log_area.setReadOnly(True)
        self.log_area.setMinimumHeight(220)
        right.addWidget(self.log_area, stretch=1)

        # ---- Assemble root ----
        outer.addLayout(left, stretch=3)   # big image area
        outer.addLayout(right, stretch=2)  # control panel

        # ---- Signals wiring (ROS -> Qt) ----
        self.vehicle_status_signal.connect(self._update_vehicle_status)
        self.log_message_signal.connect(self._append_log_message)
        self.score_signal.connect(self._update_score)
        self.height_signal.connect(self._update_height)
        self.image_signal.connect(self._update_image)

        self.ros_node.status_update_callback = self.receive_status_update
        self.ros_node.log_message_callback = self.receive_log_message
        self.ros_node.score_update_callback = self.receive_score_update
        self.ros_node.height_update_callback = self.receive_height_update
        self.ros_node.image_update_callback = self.receive_image_update

        # --- Connect AFTER creating buttons
        self.mode_combo.currentIndexChanged.connect(self.on_mode_combo_changed)
        self.on_mode_combo_changed(self.mode_combo.currentIndex())

        # Timer to refresh vehicle status line
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.poll_status)
        self.timer.start(500)

        # Initial placement
        self._place_initial(cfg.get("window_pos", "top-right"))

    # Keep image nice on window resize
    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self._last_qimage is not None:
            self._apply_qimage(self._last_qimage)

    # -------------------- placement helper -------------------- #
    def _place_initial(self, where: str):
        scr = QApplication.primaryScreen().availableGeometry()
        w, h = self.frameGeometry().width(), self.frameGeometry().height()
        if "top-right" in where:
            self.move(scr.right() - w - 10, scr.top() + 10)
        elif "top-left" in where:
            self.move(scr.left() + 10, scr.top() + 10)

    # -------------------- Mode banner logic -------------------- #
    def on_mode_combo_changed(self, idx: int):
        try:
            mode_val = int(self.mode_combo.itemData(idx))
        except Exception:
            mode_val = None

        self._update_mode_banner(mode_val)
        self.starting = False
        self.start_stop_btn.setText("Start")

    def _update_mode_banner(self, mode_val: int | None):
        """
        Update the large mode banner text and color.
        - If mode == 4 (OFFBOARD): show "QUÁ TRÌNH ĐIỀU KHIỂN TỰ ĐỘNG"
        - Otherwise: show "QUÁ TRÌNH ĐIỀU KHIỂN BẰNG TAY"
        """
        is_auto = (mode_val == 4)

        if is_auto:
            # Autonomous control process
            self.mode_banner.setText("QUÁ TRÌNH ĐIỀU KHIỂN TỰ ĐỘNG")
            self.mode_banner.setStyleSheet("color:#cc2020;")
        else:
            # Manual control process
            self.mode_banner.setText("QUÁ TRÌNH ĐIỀU KHIỂN BẰNG TAY")
            self.mode_banner.setStyleSheet("color:#0aa27a;")
    # -------------------- Doc-style status formatting -------------------- #
    def _fmt_status_doc(self, msg: VehicleStatus) -> str:
        nav_name = NAV_STATE_NAMES.get(int(msg.nav_state), "UNKNOWN")
        armed = (msg.arming_state == 2)
        failsafe = bool(msg.failsafe)

        armed_html = f"<span style='color:{'#0a7' if armed else '#c00'};font-weight:700;'>{armed}</span>"
        nav_html = (
            f"<code style='background:#f3f4f6;padding:1px 6px;border-radius:6px'>{nav_name}</code> "
            f"<span style='color:#666;'>({int(msg.nav_state)})</span>"
        )
        sysid_html = f"{int(msg.system_id)}"
        type_html = f"{int(msg.vehicle_type)}"
        failsafe_html = f"<span style='color:{'#c00' if failsafe else '#666'};font-weight:{'700' if failsafe else '600'};'>{failsafe}</span>"

        html = f"""
        <div style="font-family:Segoe UI,Roboto,Helvetica,Arial,sans-serif; font-size:12pt; line-height:1.35;">
          <table style="border-collapse:collapse;">
            <tr><td style="padding:2px 10px; color:#444; font-weight:700; white-space:nowrap;">Armed</td>
                <td style="padding:2px 10px;">{armed_html}</td></tr>
            <tr><td style="padding:2px 10px; color:#444; font-weight:700; white-space:nowrap;">Nav</td>
                <td style="padding:2px 10px;">{nav_html}</td></tr>
            <tr><td style="padding:2px 10px; color:#444; font-weight:700; white-space:nowrap;">SysID</td>
                <td style="padding:2px 10px;">{sysid_html}</td></tr>
            <tr><td style="padding:2px 10px; color:#444; font-weight:700; white-space:nowrap;">Type</td>
                <td style="padding:2px 10px;">{type_html}</td></tr>
            <tr><td style="padding:2px 10px; color:#444; font-weight:700; white-space:nowrap;">Failsafe</td>
                <td style="padding:2px 10px;">{failsafe_html}</td></tr>
          </table>
        </div>
        """
        return html

    # -------------------- Button handlers -------------------- #
    def on_apply_clicked(self):
        mode_val = int(self.mode_combo.currentData())
        altitude = float(self.altitude_spin.value())
        arm = 1 if self.arm_checkbox.isChecked() else 0
        handle_val = int(self.handle_combo.currentData())

        self._update_mode_banner(mode_val)
        self.ros_node.send_mode(mode_val, altitude, arm, handle_val)

        if mode_val == 4:  # OFFBOARD => AUTO
            self.ros_node.send_flag_auto(
                flag_auto=True,
                flag_start_auto=False, flag_stop_auto=False,
                flag_manual=False,
                flag_start_manual=False, flag_stop_manual=False
            )
        else:               # POSHOLD/LAND => MANUAL
            self.ros_node.send_flag_auto(
                flag_auto=False,
                flag_start_auto=False, flag_stop_auto=False,
                flag_manual=True,
                flag_start_manual=False, flag_stop_manual=False
            )

    def on_start_stop_clicked(self):
        mode_val = int(self.mode_combo.currentData())
        is_auto = (mode_val == 4)

        if not self.starting:
            if is_auto:
                self.ros_node.send_flag_auto(
                    flag_auto=True,
                    flag_start_auto=True, flag_stop_auto=False,
                    flag_manual=False,
                    flag_start_manual=False, flag_stop_manual=False
                )
            else:
                self.ros_node.send_flag_auto(
                    flag_auto=False,
                    flag_start_auto=False, flag_stop_auto=False,
                    flag_manual=True,
                    flag_start_manual=True, flag_stop_manual=False
                )
            self.start_stop_btn.setText("Stop")
            self.starting = True
        else:
            if is_auto:
                self.ros_node.send_flag_auto(
                    flag_auto=True,
                    flag_start_auto=False, flag_stop_auto=True,
                    flag_manual=False,
                    flag_start_manual=False, flag_stop_manual=False
                )
            else:
                self.ros_node.send_flag_auto(
                    flag_auto=False,
                    flag_start_auto=False, flag_stop_auto=False,
                    flag_manual=True,
                    flag_start_manual=False, flag_stop_manual=True
                )
            self.start_stop_btn.setText("Start")
            self.starting = False

    # -------------------- ROS→Qt bridge -------------------- #
    def receive_status_update(self, msg: VehicleStatus):
        self.vehicle_status_signal.emit(msg)

    def receive_log_message(self, text: str, level: str):
        self.log_message_signal.emit(text, level)

    def receive_score_update(self, data: dict):
        self.score_signal.emit(data)

    def receive_height_update(self, height_m: float):
        self.height_signal.emit(height_m)

    def receive_image_update(self, frame_bgr: np.ndarray):
        self.image_signal.emit(frame_bgr)

    # -------------------- Qt slot methods -------------------- #
    def _update_vehicle_status(self, msg: VehicleStatus):
        self.status_label.setText(self._fmt_status_doc(msg))

    def _update_score(self, data: dict):
        def v(x):
            return "..." if x is None else str(x)

        time_val = data.get("time_total", None)
        if time_val is None or time_val == 0:
            time_str = "00:00"
        else:
            try:
                total_sec = int(time_val)
                minutes = total_sec // 60
                seconds = total_sec % 60
                time_str = f"{minutes:02d}:{seconds:02d}"
            except Exception:
                time_str = "??:??"

        self.big_score.setText(f"Score: {v(data.get('total_score'))}")
        self.big_time.setText(f"Time: {time_str}")

        if hasattr(self, "alarm_label"):
            self.alarm_label.setText(f"<b>Alarm Rule Height:</b> {v(data.get('alarm_rule_height'))}")

    def _update_height(self, height_m: float):
        try:
            self.height_label.setText(f"<b>Height:</b> {height_m:.2f} m")
        except Exception:
            self.height_label.setText(f"<b>Height:</b> ... m")

    def _apply_qimage(self, qimg: QImage):
        """Scale and paint image to the label, keeping aspect ratio and maximizing size."""
        self._last_qimage = qimg
        if self.image_label.width() < 2 or self.image_label.height() < 2:
            return
        pm = QPixmap.fromImage(qimg)
        scaled = pm.scaled(
            self.image_label.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation
        )
        self.image_label.setPixmap(scaled)

    def _update_image(self, frame_bgr: np.ndarray):
        try:
            if frame_bgr.ndim == 2:
                frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_GRAY2RGB)
            else:
                frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
            h, w, ch = frame_rgb.shape
            qimg = QImage(frame_rgb.data, w, h, ch * w, QImage.Format.Format_RGB888).copy()
            self._apply_qimage(qimg)
        except Exception as e:
            self.image_label.setText(f"Image error: {e}")

    # -------------------- Periodic poll -------------------- #
    def poll_status(self):
        msg = self.ros_node.vehicle_status
        if msg:
            self.vehicle_status_signal.emit(msg)

    def _append_log_message(self, text: str, level: str):
        """Append a PX4 log line to the QTextEdit with color by severity."""
        color = {
            "critical": "#ff0000",
            "error": "#ff3030",
            "warning": "#ff8800",
            "info": "#0080ff",
            "debug": "#888888",
        }.get(level, "#000000")

        import datetime as _dt
        now = _dt.datetime.now().strftime("%H:%M:%S")
        html = (
            f'<span style="color:#888;">[{now}]</span> '
            f'<span style="color:{color};"><b>{level.upper()}</b>: {text}</span>'
        )

        if hasattr(self, "log_area") and self.log_area is not None:
            self.log_area.append(html)
            self.log_area.moveCursor(self.log_area.textCursor().End)



# ============================= Entrypoint ============================= #
def main(args=None):
    rclpy.init(args=args)
    ros_node = IslabChangeModePublisher()

    app = QApplication(sys.argv)
    gui = ModeChangerGUI(ros_node)
    gui.show()

    # Spin ROS 2 in a background thread so Qt stays responsive
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    try:
        sys.exit(app.exec_())
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
