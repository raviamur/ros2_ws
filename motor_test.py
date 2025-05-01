import serial
import time

def send_command(ser, left, right):
    cmd = f"{left},{right}\n"
    ser.write(cmd.encode('utf-8'))
    print(f"Sent: {cmd.strip()}")

def ramp_speed(ser, start_left, start_right, target_left, target_right, step=30, delay=0.2):
    left = start_left
    right = start_right

    # Ramp up or down to target
    while (left != target_left) or (right != target_right):
        if left < target_left:
            left = min(left + step, target_left)
        elif left > target_left:
            left = max(left - step, target_left)

        if right < target_right:
            right = min(right + step, target_right)
        elif right > target_right:
            right = max(right - step, target_right)

        send_command(ser, left, right)
        time.sleep(delay)

# --- Main program ---

def main():
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    time.sleep(2)  # Allow time for serial to settle

    try:
        while True:
            # Forward ramp
            ramp_speed(ser, 0, 0, 200, 200)
            time.sleep(1)
            ramp_speed(ser, 200, 200, 0, 0)
            time.sleep(1)

            # Reverse ramp
            ramp_speed(ser, 0, 0, -200, -200)
            time.sleep(1)
            ramp_speed(ser, -200, -200, 0, 0)
            time.sleep(1)

            # Turn Left (left slower)
            ramp_speed(ser, 0, 0, 150, 200)
            time.sleep(1)
            ramp_speed(ser, 150, 200, 0, 0)
            time.sleep(1)

            # Turn Right (right slower)
            ramp_speed(ser, 0, 0, 200, 150)
            time.sleep(1)
            ramp_speed(ser, 200, 150, 0, 0)
            time.sleep(1)

    except KeyboardInterrupt:
        send_command(ser, 0, 0)
        ser.close()
        print("Stopped safely.")

if __name__ == "__main__":
    main()