#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, PointCloud2
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge
import numpy as np
import cv2
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sklearn.cluster import DBSCAN

class StereoClusterNode(Node):
    def __init__(self):
        super().__init__('stereo_cluster_node')
        self.bridge = CvBridge()
        self.camera_info_received = False
        self.fx = self.cx = self.cy = self.baseline = None

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.create_subscription(CameraInfo, '/left/camera_info', self.camera_info_callback, qos)
        self.create_subscription(DisparityImage, '/disparity', self.disparity_callback, qos)

        self.pub = self.create_publisher(PointCloud2, '/stereo_clusters', 10)
        self.get_logger().info("Stereo cluster node started")

    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.fx = msg.k[0]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.baseline = 0.1  # Set your actual baseline
            self.camera_info_received = True
            self.get_logger().info("Camera info received")

    def disparity_callback(self, msg):
        if not self.camera_info_received:
            return  # Wait for intrinsics first

        try:
            disp = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding='passthrough')

            if disp.dtype != np.float32:
                self.get_logger().warn("Disparity image not float32")
                return

            Q = np.float32([[1, 0, 0, -self.cx],
                            [0, 1, 0, -self.cy],
                            [0, 0, 0, self.fx],
                            [0, 0, -1 / self.baseline, 0]])

            points_3d = cv2.reprojectImageTo3D(disp, Q)
            mask = (disp > disp.min()) & (disp < 96)
            valid_points = points_3d[mask]

            if valid_points.shape[0] == 0:
                return

            cloud_msg = pc2.create_cloud_xyz32(msg.header, valid_points)
            self.pub.publish(cloud_msg)
            self.get_logger().info(f"Published {len(valid_points)} points")
        
            # Filter and downsample to reduce noise and speed up clustering
            mask = (disp > disp.min()) & (disp < 96)
            valid_points = points_3d[mask]

            if valid_points.shape[0] == 0:
                return

            # Optional: downsample if too dense
            if valid_points.shape[0] > 20000:
                valid_points = valid_points[::4]

            # Run DBSCAN clustering
            clustering = DBSCAN(eps=0.15, min_samples=30).fit(valid_points)
            labels = clustering.labels_

            # Group points by cluster
            clusters = {}
            for pt, label in zip(valid_points, labels):
                if label == -1:
                    continue  # noise
                clusters.setdefault(label, []).append(pt)

            # Sort clusters by size
            sorted_clusters = sorted(clusters.items(), key=lambda kv: -len(kv[1]))

            # Print centroids of top 10 clusters
            top_n = 10
            for i, (label, pts) in enumerate(sorted_clusters[:top_n]):
                pts_np = np.array(pts)
                centroid = np.mean(pts_np, axis=0)
                self.get_logger().info(f"Cluster {i+1} (size {len(pts)}): centroid = {centroid}")

            # Optionally publish all points (unclustered or clustered)
            cloud_msg = pc2.create_cloud_xyz32(msg.header, valid_points)
            self.pub.publish(cloud_msg)
            self.get_logger().info(f"Published {len(valid_points)} points")


        except Exception as e:
            self.get_logger().error(f"Error in disparity callback: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = StereoClusterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
