import sys
import time
from scscl_package.scscl import SCSCL


def do_gesture_then_open(servo, pose, group="hand", move_ms=1200, hold_s=0.8):
    # Move to gesture pose
    servo.group_write_pos_safe(group, pose, time_ms=move_ms)
    servo.group_wait_for_move(group, timeout=10.0, threshold=10, stable_time=0.1)

    # Hold gesture
    time.sleep(hold_s)

    # Open hand
    servo.group_write_pos_safe(group, [50, 50, 50, 50, 50], time_ms=1200)
    servo.group_wait_for_move(group, timeout=10.0, threshold=10, stable_time=0.1)

    # Disable torque
    servo.group_enable_torque(group, False)


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

    with SCSCL(port, baudrate=1000000) as servo:
        servo.with_groups({
            "hand": [1, 2, 3, 4, 5],  # thumb, index, middle, ring, pinky
        })
        servo.with_limits({
            1: (50, 420),
            2: (50, 900, True),
            3: (50, 900, True),
            4: (50, 900, True),
            5: (50, 900, True),
        })

        while True:
            cmd = input(
                '\nType a command ("peace", "rock", "open", "close", "quit"): '
            ).strip().lower()

            # Re‑enable torque at the start of each gesture cycle
            if cmd in ("peace", "rock", "middle", "open", "close"):
                print("\nEnabling torque for hand group...")
                servo.group_enable_torque("hand", True)

            if cmd == "peace":
                print("Peace sign...")
                peace_pose = [
                    420,  # thumb closed
                    50,   # index open
                    50,   # middle open
                    900,  # ring closed
                    900,  # pinky closed
                ]
                do_gesture_then_open(servo, peace_pose)

            elif cmd == "rock":
                print("Rock gesture...")
                rock_pose = [
                    420,  # thumb closed
                    50,   # index open
                    900,  # middle closed
                    900,  # ring closed
                    50,   # pinky open
                ]
                do_gesture_then_open(servo, rock_pose)

            elif cmd == "open":
                print("Open gesture...")
                open_pose = [50, 50, 50, 50, 50]
                do_gesture_then_open(servo, open_pose)

            elif cmd == "close":
                print("Close gesture...")
                close_pose = [420, 900, 900, 900, 900]
                do_gesture_then_open(servo, close_pose)

            elif cmd in ("quit", "exit"):
                print("Exiting...")
                break

            else:
                print("Unknown command.")


if __name__ == "__main__":
    main()
