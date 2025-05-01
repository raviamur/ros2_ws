import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
import numpy as np
import cv2
from cv_bridge import CvBridge

class StereoClusterNode(Node):
    def __init__(self):
        super().__init__('stereo_cluster_wls_node')
        self.get_logger().info("Stereo WLS cluster node started")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.bridge = CvBridge()
        self.left_info = None
        self.right_info = None

        self.create_subscription(CameraInfo, '/left/camera_info', self.left_info_cb, qos_profile)
        self.create_subscription(CameraInfo, '/right/camera_info', self.right_info_cb, qos_profile)
        self.create_subscription(Image, '/disparity', self.disparity_cb, qos_profile)

        self.pub = self.create_publisher(String, '/nav_cmd', 10)

    def left_info_cb(self, msg):
        self.left_info = msg

    def right_info_cb(self, msg):
        self.right_info = msg

    def disparity_cb(self, msg):
        try:
            disp_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            depth = np.array(disp_image, dtype=np.float32)
            if depth.size == 0 or self.left_info is None:
                return

            height, width = depth.shape
            zone_width = width // 3

            zones = {
                'left': depth[:, :zone_width],
                'center': depth[:, zone_width:2*zone_width],
                'right': depth[:, 2*zone_width:]
            }

            zone_min = {key: np.min(zone[np.isfinite(zone)]) if np.any(np.isfinite(zone)) else float('inf')
                        for key, zone in zones.items()}

            threshold = 1.2  # meters
            stable = {key: (val < threshold) for key, val in zone_min.items()}

            cmd = 'NONE'
            if stable['left'] and not stable['right']:
                cmd = 'RIGHT'
            elif stable['right'] and not stable['left']:
                cmd = 'LEFT'

            self.pub.publish(String(data=cmd))
            self.get_logger().info(f"dists: {zone_min}, Stable: {stable}, Command: {cmd}")

        except Exception as e:
            self.get_logger().error(f"Disparity callback failed: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = StereoClusterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
