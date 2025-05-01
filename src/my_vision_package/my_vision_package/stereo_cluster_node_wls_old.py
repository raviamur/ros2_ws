#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge
import message_filters
import numpy as np
import cv2
import sensor_msgs_py.point_cloud2 as pc2
from cv2 import ximgproc

class StereoClusterWLSNode(Node):
    def __init__(self):
        super().__init__('stereo_cluster_wls_node')
        self.bridge = CvBridge()

        self.sub_left = message_filters.Subscriber(self, Image, '/disparity')
        self.sub_right = message_filters.Subscriber(self, Image, '/disparity_right')
        self.sub_info = message_filters.Subscriber(self, CameraInfo, '/left/camera_info')

        ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_left, self.sub_right, self.sub_info],
            queue_size=10,
            slop=0.1
        )
        ts.registerCallback(self.process_disparity)

        self.pub_cloud = self.create_publisher(PointCloud2, '/stereo_clusters_wls', 10)
        self.get_logger().info("Stereo WLS cluster node started")

    def process_disparity(self, dispL_msg, dispR_msg, info_msg):
        try:
            dispL = self.bridge.imgmsg_to_cv2(dispL_msg, desired_encoding='passthrough')
            dispR = self.bridge.imgmsg_to_cv2(dispR_msg, desired_encoding='passthrough')

            if dispL.dtype != np.float32 or dispR.dtype != np.float32:
                self.get_logger().warn("Disparity images not float32")
                return

            # WLS filtering
            wls = ximgproc.createDisparityWLSFilterGeneric(False)
            wls.setLambda(8000)
            wls.setSigmaColor(1.5)

            filtered = wls.filter(disparity_map_left=dispL, disparity_map_right=dispR)

            fx = info_msg.k[0]
            cx = info_msg.k[2]
            cy = info_msg.k[5]
            baseline = 0.1  # meters

            Q = np.float32([[1, 0, 0, -cx],
                            [0, 1, 0, -cy],
                            [0, 0, 0, fx],
                            [0, 0, -1 / baseline, 0]])

            points_3d = cv2.reprojectImageTo3D(filtered, Q)
            mask = (filtered > filtered.min()) & (filtered < 96)
            valid_points = points_3d[mask]

            if valid_points.shape[0] == 0:
                self.get_logger().warn("No valid points from WLS-filtered disparity")
                return

            cloud_msg = pc2.create_cloud_xyz32(dispL_msg.header, valid_points)
            self.pub_cloud.publish(cloud_msg)

        except Exception as e:
            self.get_logger().error(f"WLS clustering failed: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = StereoClusterWLSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
