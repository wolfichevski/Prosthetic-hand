import sys
import time
from scscl_package.scscl import SCSCL

def main():
    print("Auto-detecting serial port...")
    port = SCSCL.find_port()
    if port is None:
        print("No serial port found!")
        print("\nAvailable ports:")
        for p in SCSCL.list_ports():
            print(f"  {p}")
            print("\nUsage: python 01_basic.py <serial_port>")
            sys.exit(1)
    print(f"Found: {port}")

    servo_id = 2 # Servo ID number

    print(f"Connecting to {port}...")

    # Servo safe limits
    with SCSCL(port, baudrate=1000000) as servo:
        servo.with_limits({
            1: (50, 420),              # Servo 1: limited to 50-420
            2: (50, 900, True),         # Servo 2: 50-900, INVERTED
            3: (50, 900, True),         # Servo 3: 50-900, INVERTED
            4: (50, 900, True),         # Servo 4: 50-900, INVERTED
            5: (50, 900, True),         # Servo 5: 50-900, INVERTED
        })

        print(f"Servo {servo_id} connected!")
        
        # Enable torque
        print("Enabling torque...")
        servo.enable_torque(servo_id)

        # Read current position
        current_pos = servo.read_pos(servo_id)
        print(f"Current position: {current_pos}")

        # Move to position
        print("Moving to position...")
        servo.write_pos_safe(servo_id, 50, 1000)
        servo.wait_for_move(servo_id, timeout=5.0, threshold=10, stable_time=0.1)
        current_pos = servo.read_pos(servo_id)
        print(f"Current position: {current_pos}")

        # Disable torque when done
        print("\nDisabling torque...")
        servo.disable_torque(servo_id)

        print("Done!")

if __name__ == "__main__":
    main()
