import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class TimestampChecker(Node):
    def __init__(self):
        super().__init__('timestamp_checker')
        self.sub_left = self.create_subscription(Image, '/left/image_color', self.left_callback, 10)
        self.sub_right = self.create_subscription(Image, '/right/image_color', self.right_callback, 10)
        self.left_stamp = None
        self.right_stamp = None

    def left_callback(self, msg):
        self.left_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.compare_timestamps()

    def right_callback(self, msg):
        self.right_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.compare_timestamps()

    def compare_timestamps(self):
        if self.left_stamp is not None and self.right_stamp is not None:
            time_diff = abs(self.left_stamp - self.right_stamp)
            self.get_logger().info(f"Left: {self.left_stamp:.6f}, Right: {self.right_stamp:.6f}, Diff: {time_diff:.6f} sec")

def main(args=None):
    rclpy.init(args=args)
    node = TimestampChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
