#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import CameraInfo, Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class RepublishAndSync(Node):
    def __init__(self):
        super().__init__('republish_and_sync')

        # Create QoS profile for best effort reliability
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers (Original Topics)
        self.image_sub = Subscriber(self, Image, '/left/image_rect_color', qos_profile=qos_profile)
        self.info_sub = Subscriber(self, CameraInfo, '/left/camera_info', qos_profile=qos_profile)

        # Synchronizer
        self.ts = ApproximateTimeSynchronizer([self.image_sub, self.info_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)

        # Publishers (Republished & Synced Topics)
        self.image_pub = self.create_publisher(Image, 'left/image_rect_color', qos_profile)
        self.info_pub = self.create_publisher(CameraInfo, '/camera_info', qos_profile)

    def sync_callback(self, image_msg, info_msg):
        """ Synchronization Callback """
        self.get_logger().info("Synchronized /image_rect_color and /camera_info")
        
        # Publish synchronized messages
        self.image_pub.publish(image_msg)
        self.info_pub.publish(info_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RepublishAndSync()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
