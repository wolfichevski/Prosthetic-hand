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


    print(f"Connecting to {port}...")

    # Servo safe limits
    with SCSCL(port, baudrate=1000000) as servo:
        servo.with_groups({
            "hand": [1, 2, 3, 4, 5],  # All fingers
        })
        servo.with_limits({
            1: (50, 420),              # Servo 1: limited to 50-420
            2: (50, 900, True),         # Servo 2: 50-900, INVERTED
            3: (50, 900, True),         # Servo 3: 50-900, INVERTED
            4: (50, 900, True),         # Servo 4: 50-900, INVERTED
            5: (50, 900, True),         # Servo 5: 50-900, INVERTED
        })


        
        # Enable torque for the hand group
        print("\nEnabling torque for hand group...")
        servo.group_enable_torque("hand", True)

        # Read current position
        print("\nReading all positions:")
        positions = servo.group_read_pos("hand")
        for sid, pos in positions.items():
            print(f"  Servo {sid}: {pos}")

        # Close hand
        print("Closing hand...")
        servo.group_write_pos_safe("hand", [420, 900, 900, 900, 900], time_ms=1000)
        servo.group_wait_for_move("hand", timeout=10.0, threshold=10, stable_time=0.1)

        time.sleep(1)

        # Move all fingers to open position
        print("\nOpening hand...")
        servo.group_write_pos_safe("hand", [50, 50, 50, 50, 50], time_ms=1000)
        servo.group_wait_for_move("hand", timeout=10.0, threshold=10, stable_time=0.1)

        # Disable torque
        print("Disabling torque...")
        servo.group_enable_torque("hand", False)

        print("Done!")

if __name__ == "__main__":
    main()