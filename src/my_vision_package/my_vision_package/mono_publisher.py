import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ImageConverter(Node):
    def __init__(self):
        super().__init__('image_converter')

        # Create QoS profile for best effort reliability
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create subscriptions for both left and right image topics with Best Effort QoS
        self.left_subscriber = self.create_subscription(
            Image,
            '/left/image_color',
            self.left_image_callback,
            qos_profile
        )
        self.right_subscriber = self.create_subscription(
            Image,
            '/right/image_color',
            self.right_image_callback,
            qos_profile
        )

        # Create publishers for both left and right monochrome image topics with Best Effort QoS
        self.left_publisher = self.create_publisher(
            Image,
            '/left/image_mono',
            qos_profile
        )
        self.right_publisher = self.create_publisher(
            Image,
            '/right/image_mono',
            qos_profile
        )

        self.bridge = CvBridge()

    def left_image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV image (in BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert BGR image to grayscale (Mono)
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Convert OpenCV grayscale image to ROS Image message (mono8 encoding)
            mono_image = self.bridge.cv2_to_imgmsg(gray_image, encoding='mono8')

            # Publish the grayscale (mono8) image for the left camera
            self.left_publisher.publish(mono_image)
        except Exception as e:
            self.get_logger().error(f"Error processing left image: {e}")

    def right_image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV image (in BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert BGR image to grayscale (Mono)
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Convert OpenCV grayscale image to ROS Image message (mono8 encoding)
            mono_image = self.bridge.cv2_to_imgmsg(gray_image, encoding='mono8')

            # Publish the grayscale (mono8) image for the right camera
            self.right_publisher.publish(mono_image)
        except Exception as e:
            self.get_logger().error(f"Error processing right image: {e}")

def main(args=None):
    rclpy.init(args=args)
    image_converter = ImageConverter()
    rclpy.spin(image_converter)
    image_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
