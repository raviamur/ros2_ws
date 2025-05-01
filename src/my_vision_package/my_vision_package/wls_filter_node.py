import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import cv2
import numpy as np

class WLSFilterNode(Node):
    def __init__(self):
        super().__init__('wls_filter_node')
        self.bridge = CvBridge()
        self.dispL = None
        self.dispR = None
        self.left_view = None

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.create_subscription(DisparityImage, '/disparity', self.dispL_callback, qos)
        self.create_subscription(DisparityImage, '/disparity_right', self.dispR_callback, qos)
        self.create_subscription(Image, '/left/image_raw', self.left_image_callback, qos)

        self.pub_raw = self.create_publisher(Image, '/disparity_wls_raw', 10)
        self.pub_vis = self.create_publisher(Image, '/disparity_wls_vis', 10)

        self.get_logger().info("WLS Filter Node started")

    def dispL_callback(self, msg):
        try:
            self.dispL = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding='passthrough')
            self.try_filter(msg.header)
        except Exception as e:
            self.get_logger().error(f"Failed to convert dispL: {e}")

    def dispR_callback(self, msg):
        try:
            self.dispR = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Failed to convert dispR: {e}")

    def left_image_callback(self, msg):
        try:
            self.left_view = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert left_view: {e}")

    def try_filter(self, header):
        if self.dispL is None or self.dispR is None or self.left_view is None:
            self.get_logger().warn("WLS inputs not ready")
            return

        try:
            wls = cv2.ximgproc.createDisparityWLSFilterGeneric(False)
            wls.setLambda(8000)
            wls.setSigmaColor(1.5)

            filtered = wls.filter(self.dispL, self.left_view)

            msg_raw = self.bridge.cv2_to_imgmsg(filtered, encoding='32FC1')
            msg_raw.header = header
            self.pub_raw.publish(msg_raw)

            norm = cv2.normalize(filtered, None, 0, 255, cv2.NORM_MINMAX)
            norm_u8 = np.uint8(norm)
            color = cv2.applyColorMap(norm_u8, cv2.COLORMAP_JET)
            msg_vis = self.bridge.cv2_to_imgmsg(color, encoding='bgr8')
            msg_vis.header = header
            self.pub_vis.publish(msg_vis)

            self.get_logger().info("Published WLS-filtered disparity")

        except Exception as e:
            self.get_logger().error(f"WLS filtering failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WLSFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
