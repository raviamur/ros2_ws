import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import cv_bridge

class WLSPublishNode(Node):
    def __init__(self):
        super().__init__('wls_filter_node')

        self.bridge = cv_bridge.CvBridge()

        # Subscribe to raw disparity
        self.subscription = self.create_subscription(
            Image,
            '/disparity',
            self.disparity_callback,
            10)

        # Publisher for WLS filtered output
        self.publisher = self.create_publisher(
            Image,
            '/disparity_wls_vis',
            10)

        # WLS Filter setup
        self.wls_filter = cv2.ximgproc.createDisparityWLSFilterGeneric(False)
        self.wls_filter.setLambda(8000)
        self.wls_filter.setSigmaColor(1.5)

        self.get_logger().info("WLS Filter Node started, subscribing to /disparity and publishing /disparity_wls_vis")

    def disparity_callback(self, msg):
        try:
            # Convert ROS2 image to OpenCV
            disp = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Apply WLS (trivial when single input)
            disp_wls = self.wls_filter.filter(disp, None)

            # Normalize for visualization
            disp_vis = cv2.normalize(disp_wls, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
            disp_vis = disp_vis.astype('uint8')

            # Publish the processed image
            disp_msg = self.bridge.cv2_to_imgmsg(disp_vis, encoding='mono8')
            self.publisher.publish(disp_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing disparity: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = WLSPublishNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
