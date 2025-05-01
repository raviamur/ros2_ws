#!/usr/bin/env python3
from cv_bridge import CvBridge
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class CameraInfoRepublisher(Node):
    def __init__(self):
        super().__init__('camera_info_republisher')

         # Create QoS profile for best effort reliability
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(
            CameraInfo,
            '/left/camera_info',
            self.camera_info_callback,
            qos_profile
        )
        self.publisher = self.create_publisher(CameraInfo, '/camera_info', 10)

    def camera_info_callback(self, msg):
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
