import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2  # Correct method for reading PointCloud2 data

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('pointcloud_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/points2',  # Change this if your topic is different
            self.pointcloud_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def pointcloud_callback(self, msg):
        self.get_logger().info(f"Received PointCloud2 message with width={msg.width} and height={msg.height}")

        # Read points
        points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        # Print first few points for verification
        if points:
            print(f"First 5 points: {points[:5]}")
        else:
            print("No valid points in the message")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

