#!/usr/bin/env python3
import sys
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from islab_msgs.msg import IslabChangeMode, IslabControl
from px4_msgs.msg import VehicleStatus, LogMessage
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel, QPushButton, QComboBox,
    QDoubleSpinBox, QCheckBox, QGroupBox, QHBoxLayout, QTextEdit
)
from PyQt5.QtCore import QTimer, pyqtSignal

MODES = [
    ('STABILIZE', 0),
    ('ALT', 1),
    ('POSHOLD', 2),
    ('AUTO', 3),
    ('OFFBOARD', 4),
    ('RTH', 5),
    ('LAND', 6),
]
SOURCES = [
    ('GROUND', 0),
    ('WEB', 1),
    ('USER', 2),
]
HANDLE = [
    ('MODE', 0),      # Normal mode change
    ('TAKEOFF', 1),   # Takeoff action
    ('ARM', 2),       # Arm/disarm action
]

class IslabChangeModePublisher(Node):
    def __init__(self):
        super().__init__('islab_change_mode_publisher')

        self.qos_profile_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher_ = self.create_publisher(IslabChangeMode, 'islab/change_mode', self.qos_profile_pub)
        self.control_pub = self.create_publisher(IslabControl, 'islab/velocity', self.qos_profile_pub)

        # Vehicle status storage
        self.vehicle_status = None
        self.status_update_callback = None

        status_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_cb,
            status_qos
        )

        # --- PX4 Log Message subscription ---
        self.log_message_callback = None
        self.create_subscription(
            LogMessage,
            '/fmu/out/log_message',
            self.log_message_cb,
            status_qos
        )

    def send_mode(self, mode, source, altitude, arm, handle):
        msg = IslabChangeMode()
        now_us = int(time.time() * 1e6)
        msg.timestamp = now_us
        msg.timestamp_sample = now_us
        msg.mode = int(mode)
        msg.source = int(source)
        msg.altitude = float(altitude)
        msg.arm = int(arm)
        msg.handel = int(handle)
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Published: mode={mode}, source={source}, altitude={altitude}, arm={arm}, handle={handle}'
        )

    def send_control(self, vx, vy, vz, vyaw):
        msg = IslabControl()
        now_us = int(time.time() * 1e6)
        msg.timestamp = now_us
        msg.timestamp_sample = now_us
        msg.vx = float(vx)
        msg.vy = float(vy)
        msg.vz = float(vz)
        msg.vyaw = float(vyaw)
        self.control_pub.publish(msg)
        self.get_logger().info(
            f'Published control: vx={vx}, vy={vy}, vz={vz}, vyaw={vyaw}'
        )

    def vehicle_status_cb(self, msg):
        self.vehicle_status = msg
        if self.status_update_callback:
            self.status_update_callback(msg)

    # New: receive px4 log messages and forward to GUI signal
    def log_message_cb(self, msg):
        # Convert C array to string, strip after first null (0x00)
        log_text = bytes(msg.text).split(b'\x00', 1)[0].decode(errors='ignore')
        severity = msg.severity
        # Use QGC/standard PX4 coloring
        if severity <= 2:
            level = "critical"
        elif severity == 3:
            level = "error"
        elif severity == 4:
            level = "warning"
        elif severity == 5 or severity == 6:
            level = "info"
        else:
            level = "debug"
        # Call the GUI thread-safe callback if set
        if self.log_message_callback:
            self.log_message_callback(log_text, level)

class ModeChangerGUI(QWidget):
    vehicle_status_signal = pyqtSignal(object)  # For thread-safe status updates
    log_message_signal = pyqtSignal(str, str)   # For thread-safe log messages

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("Islab Change Mode & Control Publisher")
        self.setGeometry(300, 300, 510, 600)
        layout = QVBoxLayout()

        # ---- Vehicle Status Label ----
        self.status_label = QLabel("Vehicle status: ...", self)
        layout.addWidget(self.status_label)

        # ---- Mode Combo ----
        self.mode_combo = QComboBox(self)
        for name, val in MODES:
            self.mode_combo.addItem(f"{name} ({val})", val)
        layout.addWidget(QLabel("Select Mode:"))
        layout.addWidget(self.mode_combo)

        # ---- Source Combo ----
        self.source_combo = QComboBox(self)
        for name, val in SOURCES:
            self.source_combo.addItem(f"{name} ({val})", val)
        layout.addWidget(QLabel("Select Source:"))
        layout.addWidget(self.source_combo)

        # ---- Altitude Spinbox ----
        self.altitude_spin = QDoubleSpinBox(self)
        self.altitude_spin.setMinimum(-100)
        self.altitude_spin.setMaximum(500)
        self.altitude_spin.setValue(2.0)
        self.altitude_spin.setSingleStep(0.5)
        layout.addWidget(QLabel("Altitude (m):"))
        layout.addWidget(self.altitude_spin)

        # ---- Arm/Disarm Checkbox ----
        arm_group = QGroupBox("Arm State")
        arm_layout = QHBoxLayout()
        self.arm_checkbox = QCheckBox("Arm (checked=ARM, unchecked=DISARM)")
        arm_layout.addWidget(self.arm_checkbox)
        arm_group.setLayout(arm_layout)
        layout.addWidget(arm_group)

        # ---- Handle Combo ----
        self.handle_combo = QComboBox(self)
        for name, val in HANDLE:
            self.handle_combo.addItem(f"{name} ({val})", val)
        layout.addWidget(QLabel("Select Handle:"))
        layout.addWidget(self.handle_combo)

        # ---- Send Mode/Arm/Takeoff Button ----
        self.button = QPushButton("Send Mode/Takeoff/Arm", self)
        self.button.clicked.connect(self.send_mode)
        layout.addWidget(self.button)

        # ---- Velocity Control Panel ----
        control_group = QGroupBox("Velocity Control")
        control_layout = QHBoxLayout()
        self.vx_spin = QDoubleSpinBox(self)
        self.vx_spin.setRange(-5, 5)
        self.vx_spin.setValue(0.0)
        self.vx_spin.setSingleStep(0.1)
        self.vx_spin.setPrefix("Vx: ")
        control_layout.addWidget(self.vx_spin)

        self.vy_spin = QDoubleSpinBox(self)
        self.vy_spin.setRange(-5, 5)
        self.vy_spin.setValue(0.0)
        self.vy_spin.setSingleStep(0.1)
        self.vy_spin.setPrefix("Vy: ")
        control_layout.addWidget(self.vy_spin)

        self.vz_spin = QDoubleSpinBox(self)
        self.vz_spin.setRange(-5, 5)
        self.vz_spin.setValue(0.0)
        self.vz_spin.setSingleStep(0.1)
        self.vz_spin.setPrefix("Vz: ")
        control_layout.addWidget(self.vz_spin)

        self.vyaw_spin = QDoubleSpinBox(self)
        self.vyaw_spin.setRange(-180, 180)
        self.vyaw_spin.setValue(0.0)
        self.vyaw_spin.setSingleStep(1.0)
        self.vyaw_spin.setPrefix("Yaw: ")
        control_layout.addWidget(self.vyaw_spin)

        self.control_button = QPushButton("Send Control", self)
        self.control_button.clicked.connect(self.send_control)
        control_layout.addWidget(self.control_button)
        control_group.setLayout(control_layout)
        layout.addWidget(control_group)

        # ---- PX4 Log Message Area ----
        self.log_area = QTextEdit(self)
        self.log_area.setReadOnly(True)
        self.log_area.setMinimumHeight(200)
        layout.addWidget(QLabel("PX4 Log:"))
        layout.addWidget(self.log_area)

        self.setLayout(layout)

        # Thread-safe: connect Qt signal to GUI update method
        self.vehicle_status_signal.connect(self._update_vehicle_status)
        self.log_message_signal.connect(self._append_log_message)
        self.ros_node.status_update_callback = self.receive_status_update
        self.ros_node.log_message_callback = self.receive_log_message

        # Poll status every 0.5s for UI responsiveness
        self.timer = QTimer()
        self.timer.timeout.connect(self.poll_status)
        self.timer.start(500)

    def send_mode(self):
        mode_val = self.mode_combo.currentData()
        source_val = self.source_combo.currentData()
        altitude_val = self.altitude_spin.value()
        arm_val = 1 if self.arm_checkbox.isChecked() else 0
        handle_val = self.handle_combo.currentData()
        self.ros_node.send_mode(mode_val, source_val, altitude_val, arm_val, handle_val)

    def send_control(self):
        vx = self.vx_spin.value()
        vy = self.vy_spin.value()
        vz = self.vz_spin.value()
        vyaw = self.vyaw_spin.value()
        self.ros_node.send_control(vx, vy, vz, vyaw)

    # Called from ROS 2 thread
    def receive_status_update(self, msg):
        self.vehicle_status_signal.emit(msg)  # Qt will ensure _update_vehicle_status runs in main thread

    def receive_log_message(self, text, level):
        self.log_message_signal.emit(text, level)

    # Called in main GUI thread only
    def _update_vehicle_status(self, msg):
        status_text = (
            f"<b>Armed:</b> {bool(msg.arming_state == 2)}<br>"
            f"<b>Nav State:</b> {msg.nav_state}<br>"
            f"<b>System ID:</b> {msg.system_id}<br>"
            f"<b>Vehicle Type:</b> {msg.vehicle_type}<br>"
            f"<b>Failsafe:</b> {bool(msg.failsafe)}"
        )
        self.status_label.setText(f"Vehicle status:<br>{status_text}")

    def poll_status(self):
        msg = self.ros_node.vehicle_status
        if msg:
            self.vehicle_status_signal.emit(msg)

    # Append PX4 log message (with color) to QTextEdit
    def _append_log_message(self, text, level):
        color = {
            'critical': '#ff0000',
            'error':    '#ff3030',
            'warning':  '#ff8800',
            'info':     '#0080ff',
            'debug':    '#888888'
        }.get(level, '#000000')
        import datetime
        now = datetime.datetime.now().strftime("%H:%M:%S")
        html = f'<span style="color:#888;">[{now}]</span> <span style="color:{color};"><b>{level.upper()}</b>: {text}</span>'
        self.log_area.append(html)
        self.log_area.moveCursor(self.log_area.textCursor().End)

def main(args=None):
    rclpy.init(args=args)
    ros_node = IslabChangeModePublisher()
    app = QApplication(sys.argv)
    gui = ModeChangerGUI(ros_node)
    gui.show()
    # ROS 2 spin in background thread
    thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    thread.start()
    sys.exit(app.exec_())
    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()