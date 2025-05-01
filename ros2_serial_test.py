import serial
import time
import sys
import struct

# Define the protobuf format manually (int32 left_motor_speed = 1, int32 right_motor_speed = 2)
def encode_drive_motor_request(left, right):
    def encode_varint(value):
        out = []
        while True:
            byte = value & 0x7F
            value >>= 7
            if value:
                out.append(byte | 0x80)
            else:
                out.append(byte)
                break
        return bytes(out)

    # Field numbers: 1 and 2 (varint format)
    packet = b''
    packet += bytes([0x08]) + encode_varint(left)   # tag 1, varint
    packet += bytes([0x10]) + encode_varint(right)  # tag 2, varint
    return packet

# Open the serial port
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # allow time for ESP32 reset

# Send test command: move forward
packet = encode_drive_motor_request(120, 120)
ser.write(packet)
print(f"Sent DriveMotorRequest: L120 R120")

# Wait a few seconds
time.sleep(2)

# Send stop command
packet = encode_drive_motor_request(0, 0)
ser.write(packet)
print("Sent DriveMotorRequest: STOP")

ser.close()
