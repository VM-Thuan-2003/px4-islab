import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import threading

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Optimized QoS for low-latency real-time streaming
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Lower reliability for reduced CPU load
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1  # Reduce buffering for real-time streaming
        )

        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', self.qos_profile)
        self.bridge = CvBridge()

        self.fps = 60  # Target FPS
        
        # Set camera properties for high FPS and low CPU usage
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))  # MJPEG for better performance
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce buffering

        if not self.cap.isOpened():
            self.get_logger().error('Could not open camera')
            rclpy.shutdown()
            return

        self.frame = None
        self.lock = threading.Lock()

        # FPS tracking variables
        self.frame_count = 0
        self.start_time = time.monotonic()

        # Start image capture thread
        self.capture_thread = threading.Thread(target=self.capture_frames, daemon=True)
        self.capture_thread.start()

        # Timer for publishing frames (at a slightly lower rate for performance balance)
        self.timer = self.create_timer(1 / 50, self.publish_frame)  # 50Hz instead of 60Hz for CPU relief

    def capture_frames(self):
        while rclpy.ok():
            ret, frame = self.cap.read()
            if ret:
                with self.lock:
                    self.frame = frame  # Avoid unnecessary copy
            time.sleep(0.001)  # Allow CPU to process other tasks

    def publish_frame(self):
        with self.lock:
            if self.frame is None:
                return
            frame = self.frame  # Direct reference, no copy()

        # cv2.imshow('Camera', frame)
        # cv2.waitKey(1)
        
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)

        # Update FPS every 100 frames
        self.frame_count += 1
        if self.frame_count % 100 == 0:
            elapsed_time = time.monotonic() - self.start_time
            actual_fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0.0
            self.get_logger().info(f'Actual FPS: {actual_fps:.2f}')
            self.start_time = time.monotonic()
            self.frame_count = 0

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        self.get_logger().info("Camera released")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    
    if node.cap is None:
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
