"""
SCSCL Servo Control Library for Python

A Python library for controlling Feetech SCSCL series serial bus servo motors.

Basic Usage:
    from scscl import SCSCL

    with SCSCL('/dev/ttyUSB0') as servo:
        servo.ping(1)
        servo.write_pos(1, 512, 1000)  # ID=1, position=512, time=1000ms

Safe Position Control (with limits):
    from scscl import SCSCL

    with SCSCL('/dev/ttyUSB0') as servo:
        servo.set_limits(1, min_pos=50, max_pos=900)
        servo.write_pos_safe(1, 500, 1000)

Servo Groups:
    from scscl import SCSCL

    with SCSCL('/dev/ttyUSB0') as servo:
        servo.create_group("hand", [1, 2, 3, 4, 5])
        servo.group_write_pos("hand", [512, 512, 512, 512, 512], time=1000)

Auto-discovery (no port needed!):
    from scscl import SCSCL

    # Quick connect - auto-detects port
    servo = SCSCL.quick_connect()
    servo.ping(1)

    # Or find port first
    port = SCSCL.find_port()
    print(f"Found port: {port}")

    # Full auto-connect with servo scanning
    servo = SCSCL.auto_connect()
    print(f"Found servos: {servo.found_servos}")
"""

from .servo import (
    SCSCL,
    ServoFeedback,
    ServoLimits,
    ServoGroup,
    Register,
    Instruction,
    BaudRate,
    BROADCAST_ID,
    # Exceptions
    SCSCLError,
    CommunicationError,
    ServoTimeoutError,
    ChecksumError,
    ValidationError,
)

__version__ = "2.0.0"
__author__ = "wolf3x"
__all__ = [
    "SCSCL",
    "ServoFeedback",
    "ServoLimits",
    "ServoGroup",
    "Register",
    "Instruction",
    "BaudRate",
    "BROADCAST_ID",
    "SCSCLError",
    "CommunicationError",
    "ServoTimeoutError",
    "ChecksumError",
    "ValidationError",
]
