import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import serial
import threading
import time

MAX_SPEED = 250
STEP_SIZE = 50
STEP_DELAY = 0.06  # 60 ms between ramp steps
HEARTBEAT_INTERVAL = 3.0

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('ros2_serial_bridge_node')

        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        time.sleep(2)
        self.ser.write(b'0,0\n')
        self.get_logger().info("🚀 Sent startup NOOP command to ESP32")

        self.subscription = self.create_subscription(
            String,
            '/nav_cmd',
            self.cmd_callback,
            10)
        self.ack_publisher = self.create_publisher(Bool, '/esp_ack', 10)

        self.current_left = 0
        self.current_right = 0
        self.target_left = 0
        self.target_right = 0
        self.pending_command = False

        threading.Thread(target=self.read_loop, daemon=True).start()
        threading.Thread(target=self.ramp_loop, daemon=True).start()

        self.get_logger().info("✅ Serial Bridge Node started and listening to /nav_cmd")

    def cmd_callback(self, msg):
        cmd = msg.data.strip()
        if cmd in ["LEFT", "RIGHT", "REVERSE", "NONE"]:
            if cmd == "LEFT":
                self.target_left = -160
                self.target_right = -120
            elif cmd == "RIGHT":
                self.target_left = -120
                self.target_right = -160
            elif cmd == "REVERSE":
                self.target_left = -120
                self.target_right = -120
            else:
                self.target_left = -120
                self.target_right = -120

            self.pending_command = True
        else:
            self.get_logger().warn(f"⚠️ Unknown nav_cmd: {cmd}")

    def ramp_loop(self):
        last_ack_time = time.time()
        while rclpy.ok():
            time.sleep(STEP_DELAY)
            if self.pending_command:
                changed = False
                if self.current_left < self.target_left:
                    self.current_left = min(self.current_left + STEP_SIZE, self.target_left)
                    changed = True
                elif self.current_left > self.target_left:
                    self.current_left = max(self.current_left - STEP_SIZE, self.target_left)
                    changed = True

                if self.current_right < self.target_right:
                    self.current_right = min(self.current_right + STEP_SIZE, self.target_right)
                    changed = True
                elif self.current_right > self.target_right:
                    self.current_right = max(self.current_right - STEP_SIZE, self.target_right)
                    changed = True

                if changed:
                    self.send_command(self.current_left, self.current_right)
                else:
                    self.pending_command = False
                last_ack_time = time.time()

            elif time.time() - last_ack_time > HEARTBEAT_INTERVAL:
                self.send_command(self.current_left, self.current_right)
                last_ack_time = time.time()

    def send_command(self, left, right):
        left = int(max(-MAX_SPEED, min(MAX_SPEED, left)))
        right = int(max(-MAX_SPEED, min(MAX_SPEED, right)))
        cmd = f"{left},{right}\n"
        self.ser.write(cmd.encode('utf-8'))
        self.get_logger().info(f"💬 Sent command to ESP32: {left},{right}")

    def read_loop(self):
        while rclpy.ok():
            try:
                ack = self.ser.readline().decode().strip()
                if ack == "ACK":
                    msg = Bool()
                    msg.data = True  # <-- Corrected
                    self.ack_publisher.publish(msg)
                    self.get_logger().info("❤️ Real ACK received and published to /esp_ack")
            except Exception as e:
                self.get_logger().error(f"❌ Error reading serial: {e}")

def main():
    rclpy.init()
    node = SerialBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
