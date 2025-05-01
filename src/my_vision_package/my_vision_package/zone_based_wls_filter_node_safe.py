import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import deque
from rclpy.qos import QoSProfile, QoSReliabilityPolicy


class ZoneBasedWLSFilterNode(Node):
    def __init__(self):
        super().__init__('zone_based_wls_filter_node')
        self.bridge = CvBridge()
        self.dispL = None
        self.dispR = None
        self.left_view = None
        self.last_cmd = None
        self.command_hold_counter = 0
        self.hold_duration = 5

        self.zone_history = {
            'left': deque(maxlen=10),
            'center': deque(maxlen=10),
            'right': deque(maxlen=10)
        }

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.create_subscription(DisparityImage, '/disparity', self.dispL_callback, qos)
        self.create_subscription(DisparityImage, '/disparity_right', self.dispR_callback, qos)
        self.create_subscription(Image, '/left/image_raw', self.left_image_callback, qos)

        self.pub_raw = self.create_publisher(Image, '/disparity_wls_raw', 10)
        self.pub_vis = self.create_publisher(Image, '/disparity_wls_vis', 10)
        self.cmd_pub = self.create_publisher(String, '/nav_cmd', 10)

        self.get_logger().info("✅ Zone-Based WLS Filter Node started")

    def dispL_callback(self, msg):
        try:
            self.dispL = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding='passthrough')
            self.try_filter(msg.header)
        except Exception as e:
            self.get_logger().error(f"Failed to convert dispL: {e}")

    def dispR_callback(self, msg):
        try:
            self.dispR = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Failed to convert dispR: {e}")

    def left_image_callback(self, msg):
        try:
            self.left_view = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().error(f"Failed to convert left_view: {e}")

    def try_filter(self, header):
        if self.dispL is None or self.dispR is None or self.left_view is None:
            self.get_logger().warn("WLS inputs not ready")
            return

        try:
            wls = cv2.ximgproc.createDisparityWLSFilterGeneric(False)
            wls.setLambda(8000)
            wls.setSigmaColor(1.5)

            filtered = wls.filter(self.dispL, self.left_view)

            msg_raw = self.bridge.cv2_to_imgmsg(filtered, encoding='32FC1')
            msg_raw.header = header
            self.pub_raw.publish(msg_raw)

            norm = cv2.normalize(filtered, None, 0, 255, cv2.NORM_MINMAX)
            norm_u8 = np.uint8(norm)
            color = cv2.applyColorMap(norm_u8, cv2.COLORMAP_JET)
            msg_vis = self.bridge.cv2_to_imgmsg(color, encoding='bgr8')
            msg_vis.header = header
            self.pub_vis.publish(msg_vis)

            h, w = filtered.shape
            mask = (filtered > 0.5) & (filtered < 3.0)
            points = np.column_stack(np.where(mask))

            if len(points) == 0:
                self.cmd_pub.publish(String(data='NONE'))
                return

            x_world = (points[:, 1] - w / 2) * 0.002
            z_world = filtered[points[:, 0], points[:, 1]]

            zone_data = {'left': [], 'center': [], 'right': []}
            for x, z in zip(x_world, z_world):
                if 0.3 <= z <= 3.0:
                    if x < -0.1:
                        zone_data['left'].append(z)
                    elif x > 0.1:
                        zone_data['right'].append(z)
                    else:
                        zone_data['center'].append(z)

            zone_min = {k: (min(v) if v else float('inf')) for k, v in zone_data.items()}
            for k in ['left', 'center', 'right']:
                self.zone_history[k].append(1 if zone_min[k] < 1.5 else 0)

            stable = {k: sum(self.zone_history[k]) >= 7 for k in ['left', 'center', 'right']}

            if stable['center']:
                if zone_min['left'] > 1.5 and zone_min['right'] > 1.5:
                    cmd = self.last_cmd or 'RIGHT'
                elif zone_min['left'] > 1.5:
                    cmd = 'LEFT'
                elif zone_min['right'] > 1.5:
                    cmd = 'RIGHT'
                elif all(z > 0.6 and z < float('inf') for z in zone_min.values()):
                    cmd = 'REVERSE'  # default motion forward for this cart
                else:
                    cmd = 'NONE'
            elif stable['left'] and not stable['right'] and zone_min['left'] < float('inf'):
                cmd = 'RIGHT'
            elif stable['right'] and not stable['left'] and zone_min['right'] < float('inf'):
                cmd = 'LEFT'
            else:
                cmd = 'NONE'

            print(f"[ZoneWLS] Zmin: L:{zone_min['left']:.2f} C:{zone_min['center']:.2f} R:{zone_min['right']:.2f} → {cmd}")

            if cmd != self.last_cmd:
                if self.command_hold_counter >= self.hold_duration:
                    self.get_logger().info(f"📢 Publishing to /nav_cmd: {cmd}")
                    self.last_cmd = cmd
                    self.command_hold_counter = 0
                else:
                    self.command_hold_counter += 1
                    cmd = self.last_cmd
            else:
                self.command_hold_counter = 0

            self.cmd_pub.publish(String(data=cmd or 'NONE'))
        except Exception as e:
            self.get_logger().error(f"WLS filtering failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ZoneBasedWLSFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
