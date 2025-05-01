import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

class StereoSyncNode(Node):
    def __init__(self):
        super().__init__('stereo_sync')

        qos = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

        # Subscribe to images and CameraInfo topics
        self.sub_left_img = Subscriber(self, Image, '/left/image_raw', qos_profile=qos)
        self.sub_right_img = Subscriber(self, Image, '/right/image_raw', qos_profile=qos)
        self.sub_left_info = Subscriber(self, CameraInfo, '/left/camera_info', qos_profile=qos)
        self.sub_right_info = Subscriber(self, CameraInfo, '/right/camera_info', qos_profile=qos)

        # ApproximateTimeSynchronizer to match images and CameraInfo together
        self.ts = ApproximateTimeSynchronizer(
            [self.sub_left_img, self.sub_right_img, self.sub_left_info, self.sub_right_info], 
            queue_size=10, slop=0.05
        )
        self.ts.registerCallback(self.sync_callback)

        # Publishers for synchronized messages
        self.pub_left_img = self.create_publisher(Image, '/left/image_rect', 10)
        self.pub_right_img = self.create_publisher(Image, '/right/image_rect', 10)
        self.pub_left_info = self.create_publisher(CameraInfo, '/left/camera_info', 10)
        self.pub_right_info = self.create_publisher(CameraInfo, '/right/camera_info', 10)

    def sync_callback(self, left_img, right_img, left_info, right_info):
        # Assign a common timestamp for all messages
        sync_time = left_img.header.stamp

        left_img.header.stamp = sync_time
        right_img.header.stamp = sync_time
        left_info.header.stamp = sync_time
        right_info.header.stamp = sync_time

        # Publish synchronized data
        self.pub_left_img.publish(left_img)
        self.pub_right_img.publish(right_img)
        self.pub_left_info.publish(left_info)
        self.pub_right_info.publish(right_info)

        #self.get_logger().info(f"Published synchronized images & CameraInfo with timestamp {sync_time.sec}.{sync_time.nanosec}")

def main():
    rclpy.init()
    node = StereoSyncNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
