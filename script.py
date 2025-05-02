import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import struct
import math

class PointCloudReader(Node):
    def __init__(self):
        super().__init__('pointcloud_reader')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/points2',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info(f"Received PointCloud2 message: {msg.width} x {msg.height}")

        # Convert binary data to NumPy array
        data = np.frombuffer(msg.data, dtype=np.uint8)
        point_step = msg.point_step
        num_points = len(data) // point_step

        points = []
        for i in range(num_points):
            start = i * point_step
            x = struct.unpack('f', data[start:start+4])[0]
            y = struct.unpack('f', data[start+4:start+8])[0]
            z = struct.unpack('f', data[start+8:start+12])[0]

            # Ignore NaN or infinite values
            if math.isnan(x) or math.isnan(y) or math.isnan(z):
                continue

            points.append((x, y, z))

        # Print only first 5 valid points
        for i, point in enumerate(points[:5]):
            self.get_logger().info(f"Point {i}: {point}")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
