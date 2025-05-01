import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

class RightCameraSyncRepublisher(Node):
    def __init__(self):
        super().__init__('right_camera_sync_republisher')

        self.image_sub = self.create_subscription(
            Image, '/right/image_raw', self.image_callback, 10)
        self.info_sub = self.create_subscription(
            CameraInfo, '/right/stereo/camera_info', self.info_callback, 10)

        self.image_pub = self.create_publisher(Image, '/right/image_raw_sync', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/right/stereo/camera_info_sync', 10)

        self.latest_image = None
        self.latest_info = None

    def image_callback(self, msg):
        self.latest_image = msg
        self.try_publish()

    def info_callback(self, msg):
        self.latest_info = msg
        self.try_publish()

    def try_publish(self):
        if self.latest_image and self.latest_info:
            now = self.get_clock().now().to_msg()
            self.latest_image.header.stamp = now
            self.latest_info.header.stamp = now
            self.image_pub.publish(self.latest_image)
            self.info_pub.publish(self.latest_info)
            self.latest_image = None
            self.latest_info = None

def main(args=None):
    rclpy.init(args=args)
    node = RightCameraSyncRepublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
