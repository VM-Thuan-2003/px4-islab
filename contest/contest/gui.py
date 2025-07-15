import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import sys

from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout,
    QListWidget, QGridLayout, QGroupBox, QSizePolicy
)
from PyQt5.QtGui import QPixmap, QImage
from PyQt5.QtCore import Qt, QTimer, QObject, QEvent

import json

class UAVContestGUI(QWidget):
    def __init__(self):
        super().__init__()

        self.size_view = (320, 240)
        self.max_view = (480, 320)
        self.full_screen = True
        self.mode = "Manual"
        self.mode_publisher = None  # Will be set from ROS node

        self.setWindowTitle("UAV - AUTOPILOT - CONTEST")
        self.setGeometry(100, 100, 1280, 720)
        self.setStyleSheet("font-family: Arial;")

        main_layout = QGridLayout()

        title = QLabel("UAV - AUTOPILOT - CONTEST")
        title.setStyleSheet("font-size: 26px; font-weight: bold;")
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title, 0, 0, 1, 3)

        left_panel = QVBoxLayout()

        self.time_label = QLabel("Time: 00:00\nMode: Manual")
        self.time_label.setAlignment(Qt.AlignCenter)
        self.time_label.setStyleSheet("font-size: 16px;")

        self.auto_btn = QPushButton("Switch to Auto")
        self.auto_btn.setStyleSheet("padding: 6px 12px; font-weight: bold;")
        self.auto_btn.clicked.connect(self.toggle_mode)

        time_mode_group = QVBoxLayout()
        time_mode_group.addWidget(self.time_label)
        time_mode_group.addWidget(self.auto_btn)

        time_mode_box = QGroupBox()
        time_mode_box.setLayout(time_mode_group)
        left_panel.addWidget(time_mode_box)

        self.score_list_manual = QListWidget()
        self.score_list_auto = QListWidget()

        score_layout = QVBoxLayout()
        score_layout.addWidget(QLabel("Manual"))
        score_layout.addWidget(self.score_list_manual)
        score_layout.addWidget(QLabel("Auto"))
        score_layout.addWidget(self.score_list_auto)

        score_box = QGroupBox("Score")
        score_box.setLayout(score_layout)
        score_box.setMaximumWidth(160)
        left_panel.addWidget(score_box)

        main_layout.addLayout(left_panel, 1, 0)

        self.top_view = QLabel()
        self.set_view_size(self.top_view)
        self.top_view.setStyleSheet("border: 2px solid black;")
        self.top_view.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(self.top_view, 1, 1)

        right_panel = QVBoxLayout()

        right_panel.addWidget(QLabel("<b>Front View</b>", alignment=Qt.AlignCenter))
        self.front_view = QLabel()
        self.set_view_size(self.front_view)
        self.front_view.setStyleSheet("border: 2px solid black;")
        self.front_view.setAlignment(Qt.AlignCenter)
        right_panel.addWidget(self.front_view)

        right_panel.addWidget(QLabel("<b>Down View</b>", alignment=Qt.AlignCenter))
        self.down_view = QLabel()
        self.set_view_size(self.down_view)
        self.down_view.setStyleSheet("border: 2px solid black;")
        self.down_view.setAlignment(Qt.AlignCenter)
        right_panel.addWidget(self.down_view)

        main_layout.addLayout(right_panel, 1, 2)

        controls = QLabel("""
        <div style="text-align: center;">
        <table style="margin: 0 auto; font-size: 14px;">
            <tr><td><b>W:</b> + Throttle</td><td><b>I:</b> + Pitch</td></tr>
            <tr><td><b>S:</b> - Throttle</td><td><b>K:</b> - Pitch</td></tr>
            <tr><td><b>A:</b> - Yaw</td><td><b>L:</b> + Roll</td></tr>
            <tr><td><b>D:</b> + Yaw</td><td><b>J:</b> - Roll</td></tr>
            <tr><td><b>R:</b> Land mode</td><td><b>SPACE:</b> Takeoff</td></tr>
        </table></div>""")
        controls.setTextFormat(Qt.RichText)
        controls.setAlignment(Qt.AlignCenter)
        controls.setStyleSheet("font-size: 14px;")
        main_layout.addWidget(controls, 2, 0, 1, 3)

        self.setLayout(main_layout)

    def toggle_mode(self):
        self.mode = "Manual" if self.mode == "Auto" else "Auto"
        self.auto_btn.setText(f"Switch to {'Manual' if self.mode == 'Auto' else 'Auto'}")
        time_str = self.time_label.text().split("\n")[0].replace("Time: ", "")
        self.set_time_mode(time_str, self.mode)
        if self.mode_publisher:
            msg = String()
            msg.data = self.mode
            self.mode_publisher.publish(msg)

    def set_view_size(self, view):
        size = self.max_view if self.full_screen else self.size_view
        view.setFixedSize(*size)

    def update_front_view(self, image):
        self.front_view.setPixmap(self.cv2_to_pixmap(image))

    def update_down_view(self, image):
        self.down_view.setPixmap(self.cv2_to_pixmap(image))

    def update_top_view(self, image):
        self.top_view.setPixmap(self.cv2_to_pixmap(image))

    def cv2_to_pixmap(self, cv_img):
        rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        q_img = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        return QPixmap.fromImage(q_img).scaled(*self.size_view, Qt.KeepAspectRatio)

    def set_time_mode(self, time_str: str, mode_str: str):
        self.time_label.setText(f"Time: {time_str}\nMode: {mode_str}")

    def update_scores(self, manual_scores: list, auto_scores: list):
        self.score_list_manual.clear()
        self.score_list_auto.clear()
        self.score_list_manual.addItems([f"  Point {i+1}: {t}" for i, t in enumerate(manual_scores)])
        self.score_list_auto.addItems([f"  Point {i+1}: {t}" for i, t in enumerate(auto_scores)])

class Control(QObject):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.key_map = {
            Qt.Key_W: "W", Qt.Key_S: "S", Qt.Key_A: "A", Qt.Key_D: "D",
            Qt.Key_Q: "Q", Qt.Key_E: "E", Qt.Key_I: "I", Qt.Key_K: "K",
            Qt.Key_J: "J", Qt.Key_L: "L", Qt.Key_T: "T", Qt.Key_F: "F",
            Qt.Key_Space: "SPACE", Qt.Key_R: "R"
        }
        self.key_state = {name: False for name in self.key_map.values()}
        self.publisher = ros_node.create_publisher(String, '/event/keyboard', 10)

        self.timer = QTimer()
        self.timer.timeout.connect(self.publish_key_states)
        self.timer.start(50)

    def eventFilter(self, obj, event):
        if event.type() == QEvent.KeyPress and not event.isAutoRepeat():
            key = event.key()
            if key in self.key_map:
                if self.key_map[key] == "SPACE" and hasattr(self.ros_node.gui, 'mode') and self.ros_node.gui.mode == 'Auto':
                    self.ros_node.gui.toggle_mode()
                self.key_state[self.key_map[key]] = True
            return True
        elif event.type() == QEvent.KeyRelease and not event.isAutoRepeat():
            key = event.key()
            if key in self.key_map:
                self.key_state[self.key_map[key]] = False
            return True
        return False

    def publish_key_states(self):
        if hasattr(self.ros_node.gui, 'mode') and self.ros_node.gui.mode == 'Manual':
            msg = String()
            msg.data = json.dumps(self.key_state)
            self.publisher.publish(msg)

class GuiContest(Node):
    def __init__(self, gui: UAVContestGUI, control: Control):
        super().__init__('gui_contest')
        self.gui = gui
        self.control = control
        self.bridge = CvBridge()

        self.create_subscription(Image, '/UAV/bottom/image_raw', self.callback_down, 10)
        self.create_subscription(Image, '/UAV/forward/image_raw', self.callback_front, 10)
        self.create_subscription(Image, '/top_camera/image_raw', self.callback_top, 10)

        self.mode_publisher = self.create_publisher(String, '/event/mode', 10)
        self.gui.mode_publisher = self.mode_publisher

        self.elapsed_seconds = 0
        self.create_timer(1.0, self.timer_callback)

    def callback_top(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.gui.update_top_view(cv_image)
        except Exception as e:
            self.get_logger().error(f"Top image error: {e}")

    def callback_down(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.gui.update_down_view(cv_image)
        except Exception as e:
            self.get_logger().error(f"Down image error: {e}")

    def callback_front(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            self.gui.update_front_view(cv_image)
        except Exception as e:
            self.get_logger().error(f"Front image error: {e}")

    def timer_callback(self):
        self.elapsed_seconds += 1
        minutes = self.elapsed_seconds // 60
        seconds = self.elapsed_seconds % 60
        time_str = f"{minutes:02}:{seconds:02}"
        self.gui.set_time_mode(time_str, self.gui.mode)

        manual_scores = [f"01:{i+1:02}" for i in range(6)]
        auto_scores = [f"02:{i+1:02}" for i in range(6)]
        self.gui.update_scores(manual_scores, auto_scores)

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    gui = UAVContestGUI()
    node = GuiContest(gui, None)
    control = Control(node)
    gui.installEventFilter(control)
    gui.setFocus()
    node.control = control

    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    timer.start(10)

    try:
        gui.show()
        app.exec_()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Exception: {e}")

    node.destroy_node()
    rclpy.shutdown()
