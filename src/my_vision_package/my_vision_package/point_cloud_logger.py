import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
import csv
import time

class PointCloudLogger(Node):
    def __init__(self):
        super().__init__('pointcloud_logger')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/points2',  # Change if needed
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning
        
        self.data = []
        self.start_time = time.time()
        self.duration = 20  # Collect data for 20 seconds

    def listener_callback(self, msg):
        """ Callback function to process incoming point cloud messages """
        current_time = time.time()

        if current_time - self.start_time > self.duration:
            self.get_logger().info("Logging complete. Writing to file...")
            self.write_to_file()

            # Shutdown properly
            self.get_logger().info("Shutting down node.")
            self.destroy_node()
            rclpy.shutdown()

        try:
            pc_list = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            self.data.extend(pc_list)
        
        except Exception as e:
            self.get_logger().error(f"Error processing point cloud: {e}")

    def write_to_file(self):
        """ Writes collected point cloud data to a CSV file """
        if not self.data:
            self.get_logger().warn("No data collected. File not created.")
            return
        
        with open('pointcloud_log.csv', 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["X", "Y", "Z"])  # Column headers
            writer.writerows(self.data)

        self.get_logger().info(f"Saved {len(self.data)} points to pointcloud_log.csv")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("User interrupted. Writing collected data...")
        node.write_to_file()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
