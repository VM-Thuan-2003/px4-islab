import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class ProcessingCamera(Node):
    def __init__(self):
        super().__init__('processing_camera')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback,
            10)
        self.publisher_ = self.create_publisher(Image, 'camera/image_processed', 10)

    def callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        cv_image = cv2.Canny(cv_image, 50, 150)
        output_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='mono8')
        self.publisher_.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)

    node = ProcessingCamera()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
