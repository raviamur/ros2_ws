import serial
import time

# --- SETTINGS ---
SERIAL_PORT = '/dev/ttyUSB0'  # Change if needed150
BAUD_RATE = 115200

# --- MAIN CODE ---
def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Give time to settle
    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")

    try:
        while True:
            cmd = input("Enter command (e.g., 120,-120): ").strip()
            if not cmd:
                continue
            ser.write((cmd + '\n').encode())
            print(f"Sent: {cmd}")

            # Read response from ESP32
            resp = ser.readline().decode().strip()
            if resp:
                print(f"Received: {resp}")

    except KeyboardInterrupt:
        print("\nExiting.")
    finally:
        ser.close()

if __name__ == '__main__':
    main()
