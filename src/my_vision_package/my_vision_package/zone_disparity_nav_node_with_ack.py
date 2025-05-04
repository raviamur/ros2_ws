import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from stereo_msgs.msg import DisparityImage
import numpy as np
import time

class ZoneDisparityNavWithAck(Node):
    def __init__(self):
        super().__init__('zone_disparity_nav_node_with_ack')

        self.subscription = self.create_subscription(
            DisparityImage,
            '/disparity',
            self.disparity_callback,
            10
        )
        self.ack_subscription = self.create_subscription(
            Bool,
            '/esp_ack',
            self.ack_callback,
            10
        )
        self.cmd_publisher = self.create_publisher(String, '/nav_cmd', 10)

        self.baseline = 0.1  # meters (adjust from calibration)
        self.focal_length = 320  # pixels (adjust from calibration)

        self.latest_zones = None
        self.last_command_time = time.time()
        self.command_interval = 0.5  # seconds
        self.last_command = "NONE"

        self.get_logger().info("🚀 Zone Disparity Nav Node with ACK gating started")

    def disparity_callback(self, msg):
        try:
            disparity = np.frombuffer(msg.image.data, dtype=np.float32).reshape((240, 320))
            valid = disparity > 1.0
            height, width = disparity.shape

            distance = np.zeros_like(disparity)
            distance[valid] = (self.focal_length * self.baseline) / disparity[valid]

            near_mask = (distance > 0.2) & (distance < 1.5)

            left_zone = near_mask[:, :width // 3].sum()
            center_zone = near_mask[:, width // 3: 2 * width // 3].sum()
            right_zone = near_mask[:, 2 * width // 3:].sum()
            stop_zone = (distance[valid] < 0.2).sum() > 0

            self.latest_zones = (left_zone, center_zone, right_zone, stop_zone)
            self.get_logger().info(f"[ZONES] L:{left_zone} C:{center_zone} R:{right_zone} STOP:{stop_zone}")

        except Exception as e:
            self.get_logger().error(f"❌ Error in disparity_callback: {e}")

    def ack_callback(self, msg):
        if not msg.data or self.latest_zones is None:
            return

        if time.time() - self.last_command_time < self.command_interval:
            return

        left_zone, center_zone, right_zone, stop = self.latest_zones

        cmd = "NONE"
        if stop:
            cmd = "STOP"
        elif center_zone > left_zone and center_zone > right_zone:
            cmd = "LEFT" if left_zone < right_zone else "RIGHT"
        elif left_zone > right_zone:
            cmd = "RIGHT"
        elif right_zone > left_zone:
            cmd = "LEFT"
        else:
            cmd = "REVERSE"

        self.cmd_publisher.publish(String(data=cmd))
        if cmd != self.last_command:
            self.get_logger().info(f"📢 nav_cmd changed to: {cmd}")
        else:
            self.get_logger().info(f"🔁 nav_cmd repeated: {cmd}")

        self.last_command = cmd
        self.last_command_time = time.time()

def main(args=None):
    rclpy.init(args=args)
    node = ZoneDisparityNavWithAck()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
