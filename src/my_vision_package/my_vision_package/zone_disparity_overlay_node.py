import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge
import numpy as np
import cv2

class DisparityBlobOverlayNode(Node):
    def __init__(self):
        super().__init__('disparity_blob_overlay_node')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            DisparityImage,
            '/disparity',
            self.disparity_callback,
            10
        )

        self.get_logger().info("📸 Disparity Blob Overlay Node Started")

    def disparity_callback(self, msg):
        try:
            # Convert disparity image
            disp_img = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding='passthrough')
            disparity = np.array(disp_img, dtype=np.float32)
            h, w = disparity.shape

            # Estimate distance: depth = f * T / d
            valid_mask = (disparity > msg.min_disparity)
            depth_map = np.zeros_like(disparity)
            with np.errstate(divide='ignore'):
                depth_map[valid_mask] = (msg.f * msg.t) / disparity[valid_mask]

            # Threshold and find blobs
            binary_mask = cv2.inRange(depth_map, 0.2, 10.0)  # between 0.2m and 10m
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary_mask)

            # Create overlay image
            overlay = cv2.cvtColor((binary_mask * 0).astype(np.uint8), cv2.COLOR_GRAY2BGR)

            for i in range(1, num_labels):  # skip background
                x, y, bw, bh, area = stats[i]
                if area < 50:
                    continue
                cx, cy = centroids[i]
                d = disparity[int(cy), int(cx)]
                distance = (msg.f * msg.t) / d if d > 0 else 0

                # Draw box and label
                cv2.rectangle(overlay, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
                cv2.putText(
                    overlay, f"{distance:.2f} m", (x, y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1
                )

            cv2.imshow("Disparity Blobs with Distance", overlay)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Disparity callback error: {e}")

def main():
    rclpy.init()
    node = DisparityBlobOverlayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()