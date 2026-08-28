#!/usr/bin/env python3
"""
Simple Servo Test Script

Tests basic servo functionality:
1. Auto-detect port
2. Ping servo
3. Move servo back and forth
4. Read position in real time

Usage:
    python test_servo.py           # Auto-detect, servo ID 1
    python test_servo.py 2         # Auto-detect, servo ID 2
    python test_servo.py 1 /dev/ttyUSB0  # Manual port
"""

import sys
import time

# Add parent directory to path if running from examples folder
sys.path.insert(0, '..')

from scscl_package.scscl import SCSCL


def main():
    # Parse arguments
    servo_id = 2
    port = None

    if len(sys.argv) >= 2:
        servo_id = int(sys.argv[1])
    if len(sys.argv) >= 3:
        port = sys.argv[2]

    # Find port
    if port is None:
        print("Auto-detecting serial port...")
        port = SCSCL.find_port()
        if port is None:
            print("ERROR: No serial port found!")
            print("\nAvailable ports:")
            for p in SCSCL.list_ports_detailed():
                print(f"  {p['device']}: {p['chip'] or 'Unknown'}")
            return

    print(f"Port: {port}")
    print(f"Servo ID: {servo_id}")
    print()

    # Connect
    servo = SCSCL(port, baudrate=1000000)
    servo.open()

    try:
        # Test 1: Ping
        print("=== Test 1: Ping ===")
        result = servo.ping(servo_id)
        if result == -1:
            print(f"ERROR: Servo {servo_id} not responding!")
            print("\nScanning for servos...")
            found = servo.scan(1, 20)
            if found:
                print(f"Found servos: {found}")
                print(f"Try: python test_servo.py {found[0]}")
            else:
                print("No servos found. Check:")
                print("  - Power supply")
                print("  - Wiring (TX/RX/GND)")
                print("  - Baud rate")
            return
        print(f"OK - Servo {servo_id} responding\n")

        # Test 2: Read position
        print("=== Test 2: Read Position ===")
        pos = servo.read_pos(servo_id)
        print(f"Current position: {pos}\n")

        # Test 3: Enable torque
        print("=== Test 3: Enable Torque ===")
        servo.enable_torque(servo_id)
        print("Torque enabled\n")

        # Test 4: Move and read position in real time
        print("=== Test 4: Move + Real-time Position ===")
        print("Moving to position 200...")

        servo.write_pos(servo_id, 200, 1500)  # Move to 200 in 1.5s

        # Read position while moving
        print("\nReal-time position:")
        start = time.time()
        while time.time() - start < 2.0:
            pos = servo.read_pos(servo_id)
            elapsed = time.time() - start
            bar = '=' * (pos // 20)  # Simple bar visualization
            print(f"  {elapsed:.1f}s: pos={pos:4d} |{bar}")
            time.sleep(0.03)

        print("\nMoving to position 800...")
        servo.write_pos(servo_id, 800, 1500)  # Move to 800 in 1.5s

        start = time.time()
        while time.time() - start < 2.0:
            pos = servo.read_pos(servo_id)
            elapsed = time.time() - start
            bar = '=' * (pos // 20)
            print(f"  {elapsed:.1f}s: pos={pos:4d} |{bar}")
            time.sleep(0.03)

        # Test 5: Read full feedback
        print("\n=== Test 5: Full Feedback ===")
        fb = servo.get_feedback(servo_id)
        if fb:
            print(f"  Position:    {fb.position}")
            print(f"  Speed:       {fb.speed}")
            print(f"  Load:        {fb.load} ({fb.load_percent:.1f}%)")
            print(f"  Voltage:     {fb.voltage_volts:.1f}V")
            print(f"  Temperature: {fb.temperature}°C")
            print(f"  Moving:      {fb.moving}")

        # Return to center
        print("\n=== Returning to Center ===")
        servo.write_pos(servo_id, 512, 1000)
        time.sleep(1.2)

        # Disable torque
        print("\n=== Disabling Torque ===")
        servo.disable_torque(servo_id)
        print("Torque disabled")

        print("\n=== All Tests Passed! ===")

    except KeyboardInterrupt:
        print("\n\nInterrupted! Disabling torque...")
        servo.disable_torque(servo_id)

    finally:
        servo.close()


if __name__ == "__main__":
    main()