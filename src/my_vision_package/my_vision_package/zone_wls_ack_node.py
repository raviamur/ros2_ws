import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import time

class ZoneBasedWLSFilterNode(Node):
    def __init__(self):
        super().__init__('zone_based_wls_filter_node')

        self.bridge = CvBridge()

        # Subscribers
        self.subscription = self.create_subscription(
            Image,
            '/disparity_wls_raw',
            self.disparity_callback,
            10)

        self.ack_subscription = self.create_subscription(
            Bool,
            '/esp_ack',
            self.ack_callback,
            10)

        # Publisher
        self.cmd_publisher = self.create_publisher(String, '/nav_cmd', 10)

        # Zone Accumulators
        self.reset_accumulators()

        # Gating Control
        self.waiting_for_ack = False
        self.last_command = "NONE"
        self.frame_accumulate_threshold = 10
        self.frame_counter = 0
        self.last_command_time = time.time()
        self.command_cooldown = 0.5

        self.get_logger().info("Zone-Based WLS Filter Node with Temporal Accumulation + ACK gating started")

    def reset_accumulators(self):
        self.left_zone = 0
        self.center_zone = 0
        self.right_zone = 0
        self.frame_counter = 0

    def disparity_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            disp_image = np.array(cv_image, dtype=np.float32)

            valid_disp = np.where((disp_image > 0.5) & (disp_image < 5.0), disp_image, 0)
            h, w = valid_disp.shape

            left = valid_disp[:, :w//3]
            center = valid_disp[:, w//3:2*w//3]
            right = valid_disp[:, 2*w//3:]

            self.left_zone += np.count_nonzero(left)
            self.center_zone += np.count_nonzero(center)
            self.right_zone += np.count_nonzero(right)
            self.frame_counter += 1

            self.get_logger().info(f"[ZoneWLS Accum] L:{self.left_zone} C:{self.center_zone} R:{self.right_zone}")
        except Exception as e:
            self.get_logger().error(f"Error in disparity_callback: {e}")

    def ack_callback(self, msg):
        if not msg.data:
            return

        if time.time() - self.last_command_time < self.command_cooldown:
            return

        if self.frame_counter >= self.frame_accumulate_threshold:
            cmd = "NONE"
            if self.left_zone > self.center_zone and self.left_zone > self.right_zone:
                cmd = "RIGHT"
            elif self.right_zone > self.center_zone and self.right_zone > self.left_zone:
                cmd = "LEFT"
            else:
                cmd = "REVERSE"

            self.cmd_publisher.publish(String(data=cmd))
            if cmd != self.last_command:
                self.get_logger().info(f"Switching nav_cmd to: {cmd}")
                self.last_command = cmd
            else:
                self.get_logger().info(f"Reaffirming nav_cmd: {cmd}")

            self.last_command_time = time.time()
            self.reset_accumulators()

    def spin(self):
        rclpy.spin(self)

def main():
    rclpy.init()
    node = ZoneBasedWLSFilterNode()
    node.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
