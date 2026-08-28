import serial
import serial.tools.list_ports
import time
import glob
import sys
import threading
import logging
from typing import List, Optional, Tuple, Dict, Callable, Union
from dataclasses import dataclass, field
from enum import IntEnum

# Setup logging
logger = logging.getLogger(__name__)


# ============================================================================
# Protocol Constants
# ============================================================================

class Instruction(IntEnum):
    """Protocol instruction codes."""
    PING = 0x01
    READ = 0x02
    WRITE = 0x03
    REG_WRITE = 0x04
    REG_ACTION = 0x05
    SYNC_READ = 0x82
    SYNC_WRITE = 0x83


class BaudRate(IntEnum):
    """Baud rate register values."""
    BAUD_1M = 0
    BAUD_500K = 1
    BAUD_250K = 2
    BAUD_128K = 3
    BAUD_115200 = 4
    BAUD_76800 = 5
    BAUD_57600 = 6
    BAUD_38400 = 7


class Register:
    """Memory table addresses (SCSCL Series)."""
    # EPROM (Read Only)
    VERSION_L = 3
    VERSION_H = 4

    # EPROM (Read/Write)
    ID = 5
    BAUD_RATE = 6
    MIN_ANGLE_LIMIT_L = 9
    MIN_ANGLE_LIMIT_H = 10
    MAX_ANGLE_LIMIT_L = 11
    MAX_ANGLE_LIMIT_H = 12
    CW_DEAD = 26
    CCW_DEAD = 27

    # SRAM (Read/Write)
    TORQUE_ENABLE = 40
    GOAL_POSITION_L = 42
    GOAL_POSITION_H = 43
    GOAL_TIME_L = 44
    GOAL_TIME_H = 45
    GOAL_SPEED_L = 46
    GOAL_SPEED_H = 47
    LOCK = 48

    # SRAM (Read Only)
    PRESENT_POSITION_L = 56
    PRESENT_POSITION_H = 57
    PRESENT_SPEED_L = 58
    PRESENT_SPEED_H = 59
    PRESENT_LOAD_L = 60
    PRESENT_LOAD_H = 61
    PRESENT_VOLTAGE = 62
    PRESENT_TEMPERATURE = 63
    MOVING = 66
    PRESENT_CURRENT_L = 69
    PRESENT_CURRENT_H = 70


# Broadcast ID (all servos)
BROADCAST_ID = 0xFE

# Valid ranges
POSITION_MIN = 0
POSITION_MAX = 1023
PWM_MIN = -1000
PWM_MAX = 1000
SERVO_ID_MIN = 1
SERVO_ID_MAX = 253


# ============================================================================
# Exceptions
# ============================================================================

class SCSCLError(Exception):
    """Base exception for SCSCL errors."""
    pass


class CommunicationError(SCSCLError):
    """Communication error with servo."""
    pass


class ServoTimeoutError(SCSCLError):
    """Timeout waiting for servo response."""
    pass


class ChecksumError(SCSCLError):
    """Checksum verification failed."""
    pass


class ValidationError(SCSCLError):
    """Input validation error."""
    pass


# ============================================================================
# Data Classes
# ============================================================================

@dataclass
class ServoFeedback:
    """Servo feedback data container with unit conversions."""
    position: int
    speed: int
    load: int
    voltage: int  # Raw value in 0.1V units
    temperature: int  # Celsius
    moving: bool
    current: int

    @property
    def voltage_volts(self) -> float:
        """Voltage in volts."""
        return self.voltage / 10.0

    @property
    def load_percent(self) -> float:
        """Load as percentage (-100 to 100)."""
        return self.load / 10.0

    @property
    def position_degrees(self) -> float:
        """Position in degrees (assuming 300 degree range)."""
        return (self.position / 1023.0) * 300.0

    @property
    def position_normalized(self) -> float:
        """Position normalized to 0.0-1.0 range."""
        return self.position / 1023.0


@dataclass
class ServoLimits:
    """Servo position limits and mapping configuration."""
    min_pos: int = 0
    max_pos: int = 1023
    invert: bool = False

    def __post_init__(self):
        if not (POSITION_MIN <= self.min_pos <= POSITION_MAX):
            raise ValidationError(f"min_pos must be {POSITION_MIN}-{POSITION_MAX}")
        if not (POSITION_MIN <= self.max_pos <= POSITION_MAX):
            raise ValidationError(f"max_pos must be {POSITION_MIN}-{POSITION_MAX}")
        if self.min_pos > self.max_pos:
            raise ValidationError("min_pos cannot be greater than max_pos")


@dataclass
class ServoGroup:
    """A named group of servos for batch operations."""
    name: str
    servo_ids: List[int]
    limits: Optional[Dict[int, ServoLimits]] = field(default_factory=dict)

    def __len__(self) -> int:
        return len(self.servo_ids)

    def __iter__(self):
        return iter(self.servo_ids)


# ============================================================================
# Main SCSCL Class
# ============================================================================

class SCSCL:
    """
    SCSCL Servo Controller

    Thread-safe controller for Feetech SCSCL series serial bus servo motors.

    Args:
        port: Serial port path (e.g., '/dev/ttyUSB0' on Linux, '/dev/cu.usbserial-xxx' on macOS)
        baudrate: Communication baud rate (default: 1000000)
        timeout: Read timeout in seconds (default: 0.1)
        end: Endianness flag (default: 1 for big-endian, SCSCL default)
        level: Response level (default: 1, return response for non-broadcast)
        auto_reconnect: If True, automatically reconnect on connection loss
        retry_count: Number of retries for failed operations (default: 3)

    Example:
        >>> with SCSCL('/dev/ttyUSB0') as servo:
        ...     servo.write_pos(1, 512, 1000)

    Fluent API:
        >>> servo = (SCSCL('/dev/ttyUSB0')
        ...     .connect()
        ...     .with_limits({1: (50, 900), 2: (0, 1023, True)}))

    Auto-discovery:
        >>> servo = SCSCL.auto_connect()
        >>> print(f"Found servos: {servo.found_servos}")
    """

    def __init__(
        self,
        port: Optional[str] = None,
        baudrate: int = 1000000,
        timeout: float = 0.1,
        end: int = 1,
        level: int = 1,
        auto_reconnect: bool = False,
        retry_count: int = 3
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.end = end  # Endianness: 1 = big-endian (SCSCL default)
        self.level = level  # Response level
        self.auto_reconnect = auto_reconnect
        self.retry_count = retry_count
        self.error = 0

        self._serial: Optional[serial.Serial] = None
        self._lock = threading.RLock()  # Thread safety
        self._feedback_mem = bytearray(Register.PRESENT_CURRENT_H - Register.PRESENT_POSITION_L + 1)

        # Configuration
        self._limits: Dict[int, ServoLimits] = {}
        self._max_speeds: Dict[int, int] = {}
        self._groups: Dict[str, ServoGroup] = {}
        self._found_servos: List[int] = []

        logger.debug(f"SCSCL initialized: port={port}, baudrate={baudrate}")

    # ========================================================================
    # Context Manager and Connection
    # ========================================================================

    def __enter__(self) -> 'SCSCL':
        self.open()
        return self

    def __exit__(self, exc_type: object, exc_val: object, exc_tb: object) -> None:
        self.close()

    def open(self) -> 'SCSCL':
        """
        Open serial connection.

        Returns:
            self for method chaining.

        Raises:
            CommunicationError: If connection fails.
        """
        if self.port is None:
            raise CommunicationError("No port specified. Use auto_connect() or provide a port.")

        with self._lock:
            try:
                self._serial = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=self.timeout
                )
                logger.info(f"Connected to {self.port} at {self.baudrate} baud")
                return self
            except serial.SerialException as e:
                logger.error(f"Failed to open port {self.port}: {e}")
                raise CommunicationError(f"Failed to open port {self.port}: {e}")

    def connect(self) -> 'SCSCL':
        """Alias for open(). Returns self for method chaining."""
        return self.open()

    def reconnect(self) -> 'SCSCL':
        """Close and reopen the connection."""
        self.close()
        return self.open()

    def close(self) -> None:
        """Close serial connection."""
        with self._lock:
            if self._serial and self._serial.is_open:
                logger.info(f"Closing connection to {self.port}")
                self._serial.close()
                self._serial = None

    @property
    def is_open(self) -> bool:
        """Check if serial port is open."""
        return self._serial is not None and self._serial.is_open

    def _ensure_open(self) -> None:
        """Ensure serial port is open."""
        if not self.is_open:
            raise CommunicationError("Serial port is not open")

    # ========================================================================
    # Validation Helpers
    # ========================================================================

    @staticmethod
    def _validate_servo_id(servo_id: int, allow_broadcast: bool = True) -> None:
        """Validate servo ID."""
        if servo_id == BROADCAST_ID and allow_broadcast:
            return
        if not (SERVO_ID_MIN <= servo_id <= SERVO_ID_MAX):
            raise ValidationError(
                f"Servo ID must be {SERVO_ID_MIN}-{SERVO_ID_MAX} "
                f"(or {BROADCAST_ID} for broadcast), got {servo_id}"
            )

    @staticmethod
    def _validate_position(position: int) -> int:
        """Validate and clamp position to valid range."""
        if not isinstance(position, (int, float)):
            raise ValidationError(f"Position must be a number, got {type(position)}")
        return max(POSITION_MIN, min(POSITION_MAX, int(position)))

    @staticmethod
    def _validate_pwm(pwm: int) -> int:
        """Validate and clamp PWM to valid range."""
        if not isinstance(pwm, (int, float)):
            raise ValidationError(f"PWM must be a number, got {type(pwm)}")
        return max(PWM_MIN, min(PWM_MAX, int(pwm)))

    @staticmethod
    def _validate_time(time_ms: int) -> int:
        """Validate movement time."""
        if not isinstance(time_ms, (int, float)):
            raise ValidationError(f"Time must be a number, got {type(time_ms)}")
        if time_ms < 0:
            raise ValidationError(f"Time cannot be negative, got {time_ms}")
        return int(time_ms)

    # ========================================================================
    # Protocol Layer
    # ========================================================================

    def _host2scs(self, data: int) -> Tuple[int, int]:
        """Convert 16-bit value to two 8-bit values based on endianness."""
        if self.end:
            return (data >> 8) & 0xFF, data & 0xFF
        else:
            return data & 0xFF, (data >> 8) & 0xFF

    def _scs2host(self, low: int, high: int) -> int:
        """Convert two 8-bit values to 16-bit value based on endianness."""
        if self.end:
            return (low << 8) | high
        else:
            return (high << 8) | low

    def _calculate_checksum(self, data: bytes) -> int:
        """Calculate checksum (inverted sum of bytes)."""
        return (~sum(data)) & 0xFF

    def _write_packet(self, servo_id: int, instruction: int, params: bytes = b'') -> None:
        """Write a packet to the serial port (thread-safe)."""
        self._ensure_open()
        assert self._serial is not None

        length = len(params) + 2  # instruction + checksum

        # Build packet
        packet = bytearray([0xFF, 0xFF, servo_id, length, instruction])
        packet.extend(params)

        # Calculate checksum
        checksum = self._calculate_checksum(bytes([servo_id, length, instruction]) + params)
        packet.append(checksum)

        # Write with lock
        with self._lock:
            self._serial.reset_input_buffer()
            self._serial.write(packet)
            self._serial.flush()

        logger.debug(f"TX: {packet.hex()}")

    def _read_response(self, expected_len: int) -> Optional[bytes]:
        """Read response packet from servo (thread-safe)."""
        self._ensure_open()
        assert self._serial is not None

        with self._lock:
            response = self._serial.read(expected_len)

        if len(response) != expected_len:
            logger.debug(f"RX: incomplete ({len(response)}/{expected_len} bytes)")
            return None

        logger.debug(f"RX: {response.hex()}")

        # Validate header
        if response[0] != 0xFF or response[1] != 0xFF:
            logger.warning("Invalid response header")
            return None

        # Validate checksum
        calc_checksum = self._calculate_checksum(response[2:-1])
        if calc_checksum != response[-1]:
            logger.warning(f"Checksum mismatch: expected {calc_checksum:02X}, got {response[-1]:02X}")
            return None

        # Store error status
        self.error = response[4]

        return response

    def _read_ack(self, servo_id: int) -> bool:
        """Read acknowledgment packet."""
        self.error = 0

        # Broadcast commands don't get ACK
        if servo_id == BROADCAST_ID or self.level == 0:
            return True

        response = self._read_response(6)
        if response is None:
            return False

        if response[2] != servo_id:
            return False

        if response[3] != 2:
            return False

        self.error = response[4]
        return True

    # ========================================================================
    # Low-Level Write Operations
    # ========================================================================

    def _gen_write(self, servo_id: int, address: int, data: bytes) -> bool:
        """General write command."""
        params = bytes([address]) + data
        self._write_packet(servo_id, Instruction.WRITE, params)
        return self._read_ack(servo_id)

    def _reg_write(self, servo_id: int, address: int, data: bytes) -> bool:
        """Asynchronous (registered) write command."""
        params = bytes([address]) + data
        self._write_packet(servo_id, Instruction.REG_WRITE, params)
        return self._read_ack(servo_id)

    def write_byte(self, servo_id: int, address: int, value: int) -> bool:
        """
        Write single byte to servo memory.

        Args:
            servo_id: Servo ID (1-253, or 254 for broadcast)
            address: Memory address
            value: Byte value (0-255)

        Returns:
            True if successful.
        """
        self._validate_servo_id(servo_id)
        return self._gen_write(servo_id, address, bytes([value & 0xFF]))

    def write_word(self, servo_id: int, address: int, value: int) -> bool:
        """
        Write 16-bit word to servo memory.

        Args:
            servo_id: Servo ID (1-253, or 254 for broadcast)
            address: Memory address
            value: Word value (0-65535)

        Returns:
            True if successful.
        """
        self._validate_servo_id(servo_id)
        low, high = self._host2scs(value)
        return self._gen_write(servo_id, address, bytes([low, high]))

    # ========================================================================
    # Low-Level Read Operations
    # ========================================================================

    def _read(self, servo_id: int, address: int, length: int) -> Optional[bytes]:
        """Read data from servo memory."""
        params = bytes([address, length])
        self._write_packet(servo_id, Instruction.READ, params)

        response = self._read_response(length + 6)
        if response is None:
            return None

        return response[5:5+length]

    def read_byte(self, servo_id: int, address: int) -> int:
        """
        Read single byte from servo memory.

        Args:
            servo_id: Servo ID (1-253)
            address: Memory address

        Returns:
            Byte value or -1 on error.
        """
        self._validate_servo_id(servo_id, allow_broadcast=False)
        data = self._read(servo_id, address, 1)
        if data is None:
            return -1
        return data[0]

    def read_word(self, servo_id: int, address: int) -> int:
        """
        Read 16-bit word from servo memory.

        Args:
            servo_id: Servo ID (1-253)
            address: Memory address

        Returns:
            Word value or -1 on error.
        """
        self._validate_servo_id(servo_id, allow_broadcast=False)
        data = self._read(servo_id, address, 2)
        if data is None:
            return -1
        return self._scs2host(data[0], data[1])

    # ========================================================================
    # Sync Read (NEW - Multi-servo read in one command)
    # ========================================================================

    def sync_read_pos(self, servo_ids: List[int]) -> Dict[int, int]:
        """
        Read positions from multiple servos in one command.

        Much faster than reading each servo individually.

        Args:
            servo_ids: List of servo IDs to read

        Returns:
            Dict mapping servo_id to position, or -1 for failed reads.

        Example:
            >>> positions = servo.sync_read_pos([1, 2, 3, 4, 5])
            >>> print(positions)  # {1: 512, 2: 300, 3: 800, 4: 512, 5: 100}
        """
        for sid in servo_ids:
            self._validate_servo_id(sid, allow_broadcast=False)

        self._ensure_open()
        assert self._serial is not None

        # Build sync read packet
        # Format: [0xFF][0xFF][0xFE][len][SYNC_READ][addr][data_len][ID1][ID2]...[checksum]
        data_len = 2  # Reading position (2 bytes)
        msg_len = len(servo_ids) + 4

        packet = bytearray([
            0xFF, 0xFF, BROADCAST_ID, msg_len, Instruction.SYNC_READ,
            Register.PRESENT_POSITION_L, data_len
        ])
        packet.extend(servo_ids)

        checksum = BROADCAST_ID + msg_len + Instruction.SYNC_READ + Register.PRESENT_POSITION_L + data_len
        checksum += sum(servo_ids)
        packet.append((~checksum) & 0xFF)

        results = {}

        with self._lock:
            self._serial.reset_input_buffer()
            self._serial.write(packet)
            self._serial.flush()

            # Read responses from each servo
            for servo_id in servo_ids:
                response = self._serial.read(8)  # 6 header + 2 data bytes

                if len(response) == 8 and response[0] == 0xFF and response[1] == 0xFF:
                    if response[2] == servo_id:
                        pos = self._scs2host(response[5], response[6])
                        results[servo_id] = pos
                    else:
                        results[servo_id] = -1
                else:
                    results[servo_id] = -1

        return results

    def sync_read_feedback(self, servo_ids: List[int]) -> Dict[int, Optional[ServoFeedback]]:
        """
        Read full feedback from multiple servos.

        Args:
            servo_ids: List of servo IDs

        Returns:
            Dict mapping servo_id to ServoFeedback or None on error.
        """
        results = {}
        for sid in servo_ids:
            results[sid] = self.get_feedback(sid)
        return results

    # ========================================================================
    # Servo Control Methods
    # ========================================================================

    def ping(self, servo_id: int) -> int:
        """
        Ping servo to check if it's responding.

        Args:
            servo_id: Servo ID to ping

        Returns:
            Servo ID if responding, -1 if not found.
        """
        self._write_packet(servo_id, Instruction.PING)

        response = self._read_response(6)
        if response is None:
            return -1

        if response[2] != servo_id and servo_id != BROADCAST_ID:
            return -1

        if response[3] != 2:
            return -1

        self.error = response[4]
        return response[2]

    def write_pos(
        self,
        servo_id: int,
        position: int,
        time_ms: int,
        speed: int = 0
    ) -> bool:
        """
        Write position to servo.

        Args:
            servo_id: Servo ID
            position: Target position (0-1023, auto-clamped)
            time_ms: Movement time in milliseconds
            speed: Maximum speed (0 = no limit)

        Returns:
            True if successful.
        """
        self._validate_servo_id(servo_id)
        position = self._validate_position(position)
        time_ms = self._validate_time(time_ms)

        # Apply max speed limit if configured
        speed = self._apply_max_speed(servo_id, speed)

        pos_l, pos_h = self._host2scs(position)
        time_l, time_h = self._host2scs(time_ms)
        speed_l, speed_h = self._host2scs(speed)

        data = bytes([pos_l, pos_h, time_l, time_h, speed_l, speed_h])

        logger.debug(f"write_pos: id={servo_id}, pos={position}, time={time_ms}, speed={speed}")
        return self._gen_write(servo_id, Register.GOAL_POSITION_L, data)

    def reg_write_pos(
        self,
        servo_id: int,
        position: int,
        time_ms: int,
        speed: int = 0
    ) -> bool:
        """
        Register (async) write position to servo.

        Position is stored but not executed until reg_write_action() is called.
        """
        self._validate_servo_id(servo_id)
        position = self._validate_position(position)
        time_ms = self._validate_time(time_ms)
        speed = self._apply_max_speed(servo_id, speed)

        pos_l, pos_h = self._host2scs(position)
        time_l, time_h = self._host2scs(time_ms)
        speed_l, speed_h = self._host2scs(speed)

        data = bytes([pos_l, pos_h, time_l, time_h, speed_l, speed_h])
        return self._reg_write(servo_id, Register.GOAL_POSITION_L, data)

    def reg_write_action(self, servo_id: int = BROADCAST_ID) -> bool:
        """Execute all pending registered writes."""
        self._write_packet(servo_id, Instruction.REG_ACTION)
        return self._read_ack(servo_id)

    def sync_write_pos(
        self,
        servo_ids: List[int],
        positions: List[int],
        times: List[int],
        speeds: Optional[List[int]] = None
    ) -> None:
        """
        Synchronously write positions to multiple servos.

        All servos move simultaneously.

        Args:
            servo_ids: List of servo IDs
            positions: List of target positions
            times: List of movement times in milliseconds
            speeds: List of max speeds (optional, default 0)
        """
        self._ensure_open()

        n = len(servo_ids)
        if speeds is None:
            speeds = [0] * n

        if len(positions) != n or len(times) != n or len(speeds) != n:
            raise ValidationError("All lists must have the same length")

        for sid in servo_ids:
            self._validate_servo_id(sid)

        # Validate and apply max speeds
        positions = [self._validate_position(p) for p in positions]
        times = [self._validate_time(t) for t in times]
        speeds = [self._apply_max_speed(sid, s) for sid, s in zip(servo_ids, speeds)]

        # Build packet
        data_len = 6
        msg_len = (data_len + 1) * n + 4

        packet = bytearray([
            0xFF, 0xFF, BROADCAST_ID, msg_len, Instruction.SYNC_WRITE,
            Register.GOAL_POSITION_L, data_len
        ])

        checksum = BROADCAST_ID + msg_len + Instruction.SYNC_WRITE + Register.GOAL_POSITION_L + data_len

        for i in range(n):
            pos_l, pos_h = self._host2scs(positions[i])
            time_l, time_h = self._host2scs(times[i])
            speed_l, speed_h = self._host2scs(speeds[i])

            packet.append(servo_ids[i])
            packet.extend([pos_l, pos_h, time_l, time_h, speed_l, speed_h])

            checksum += servo_ids[i] + pos_l + pos_h + time_l + time_h + speed_l + speed_h

        packet.append((~checksum) & 0xFF)

        with self._lock:
            assert self._serial is not None
            self._serial.reset_input_buffer()
            self._serial.write(packet)
            self._serial.flush()

        logger.debug(f"sync_write_pos: ids={servo_ids}, positions={positions}")

    def pwm_mode(self, servo_id: int) -> bool:
        """
        Set servo to PWM output mode.

        This sets min/max angle limits to 0, enabling PWM mode.
        """
        self._validate_servo_id(servo_id)
        data = bytes([0, 0, 0, 0])
        return self._gen_write(servo_id, Register.MIN_ANGLE_LIMIT_L, data)

    def write_pwm(self, servo_id: int, pwm: int) -> bool:
        """
        Write PWM value to servo (requires PWM mode).

        Args:
            servo_id: Servo ID
            pwm: PWM value (-1000 to 1000, auto-clamped)

        Returns:
            True if successful.
        """
        self._validate_servo_id(servo_id)
        pwm = self._validate_pwm(pwm)

        if pwm < 0:
            pwm = (-pwm) | (1 << 10)

        pwm_l, pwm_h = self._host2scs(pwm)
        data = bytes([pwm_l, pwm_h])

        logger.debug(f"write_pwm: id={servo_id}, pwm={pwm}")
        return self._gen_write(servo_id, Register.GOAL_TIME_L, data)

    def enable_torque(self, servo_id: int, enable: bool = True) -> bool:
        """
        Enable or disable servo torque.

        Args:
            servo_id: Servo ID (use BROADCAST_ID for all)
            enable: True to enable, False to disable

        Returns:
            True if successful.
        """
        logger.debug(f"enable_torque: id={servo_id}, enable={enable}")
        return self.write_byte(servo_id, Register.TORQUE_ENABLE, 1 if enable else 0)

    def disable_torque(self, servo_id: int) -> bool:
        """Disable servo torque (shortcut for enable_torque(id, False))."""
        return self.enable_torque(servo_id, False)

    def unlock_eprom(self, servo_id: int) -> bool:
        """Unlock EPROM for writing."""
        return self.write_byte(servo_id, Register.LOCK, 0)

    def lock_eprom(self, servo_id: int) -> bool:
        """Lock EPROM (prevent writes)."""
        return self.write_byte(servo_id, Register.LOCK, 1)

    # ========================================================================
    # Feedback Methods
    # ========================================================================

    def feedback(self, servo_id: int) -> bool:
        """
        Read all feedback data from servo into internal buffer.

        Use read_pos(-1), read_speed(-1), etc. to get cached values.
        """
        self._validate_servo_id(servo_id, allow_broadcast=False)
        data = self._read(servo_id, Register.PRESENT_POSITION_L, len(self._feedback_mem))
        if data is None:
            return False

        self._feedback_mem = bytearray(data)
        return True

    def get_feedback(self, servo_id: int) -> Optional[ServoFeedback]:
        """
        Read all feedback data and return as ServoFeedback object.

        Args:
            servo_id: Servo ID

        Returns:
            ServoFeedback object or None on error.
        """
        if not self.feedback(servo_id):
            return None

        return ServoFeedback(
            position=self.read_pos(-1),
            speed=self.read_speed(-1),
            load=self.read_load(-1),
            voltage=self.read_voltage(-1),
            temperature=self.read_temperature(-1),
            moving=self.read_moving(-1) == 1,
            current=self.read_current(-1)
        )

    def read_pos(self, servo_id: int) -> int:
        """
        Read current position.

        Args:
            servo_id: Servo ID, or -1 to read from cached feedback

        Returns:
            Position (0-1023) or -1 on error.
        """
        if servo_id == -1:
            offset = 0
            return self._scs2host(self._feedback_mem[offset], self._feedback_mem[offset + 1])
        else:
            self._validate_servo_id(servo_id, allow_broadcast=False)
            return self.read_word(servo_id, Register.PRESENT_POSITION_L)

    def read_speed(self, servo_id: int) -> int:
        """Read current speed (signed)."""
        if servo_id == -1:
            offset = Register.PRESENT_SPEED_L - Register.PRESENT_POSITION_L
            speed = self._scs2host(self._feedback_mem[offset], self._feedback_mem[offset + 1])
        else:
            self._validate_servo_id(servo_id, allow_broadcast=False)
            speed = self.read_word(servo_id, Register.PRESENT_SPEED_L)
            if speed == -1:
                return -1

        if speed & (1 << 15):
            speed = -(speed & ~(1 << 15))

        return speed

    def read_load(self, servo_id: int) -> int:
        """Read current load (-1000 to 1000)."""
        if servo_id == -1:
            offset = Register.PRESENT_LOAD_L - Register.PRESENT_POSITION_L
            load = self._scs2host(self._feedback_mem[offset], self._feedback_mem[offset + 1])
        else:
            self._validate_servo_id(servo_id, allow_broadcast=False)
            load = self.read_word(servo_id, Register.PRESENT_LOAD_L)
            if load == -1:
                return -1

        if load & (1 << 10):
            load = -(load & ~(1 << 10))

        return load

    def read_voltage(self, servo_id: int) -> int:
        """Read current voltage (0.1V units)."""
        if servo_id == -1:
            offset = Register.PRESENT_VOLTAGE - Register.PRESENT_POSITION_L
            return self._feedback_mem[offset]
        else:
            self._validate_servo_id(servo_id, allow_broadcast=False)
            return self.read_byte(servo_id, Register.PRESENT_VOLTAGE)

    def read_voltage_volts(self, servo_id: int) -> float:
        """Read current voltage in volts."""
        raw = self.read_voltage(servo_id)
        return raw / 10.0 if raw != -1 else -1.0

    def read_temperature(self, servo_id: int) -> int:
        """Read current temperature (Celsius)."""
        if servo_id == -1:
            offset = Register.PRESENT_TEMPERATURE - Register.PRESENT_POSITION_L
            return self._feedback_mem[offset]
        else:
            self._validate_servo_id(servo_id, allow_broadcast=False)
            return self.read_byte(servo_id, Register.PRESENT_TEMPERATURE)

    def read_moving(self, servo_id: int) -> int:
        """Read moving status (1=moving, 0=stopped)."""
        if servo_id == -1:
            offset = Register.MOVING - Register.PRESENT_POSITION_L
            return self._feedback_mem[offset]
        else:
            self._validate_servo_id(servo_id, allow_broadcast=False)
            return self.read_byte(servo_id, Register.MOVING)

    def is_moving(self, servo_id: int) -> bool:
        """Check if servo is currently moving."""
        return self.read_moving(servo_id) == 1

    def read_current(self, servo_id: int) -> int:
        """Read current (signed)."""
        if servo_id == -1:
            offset = Register.PRESENT_CURRENT_L - Register.PRESENT_POSITION_L
            current = self._scs2host(self._feedback_mem[offset], self._feedback_mem[offset + 1])
        else:
            self._validate_servo_id(servo_id, allow_broadcast=False)
            current = self.read_word(servo_id, Register.PRESENT_CURRENT_L)
            if current == -1:
                return -1

        if current & (1 << 15):
            current = -(current & ~(1 << 15))

        return current

    # ========================================================================
    # Configuration Methods
    # ========================================================================

    def set_id(self, servo_id: int, new_id: int) -> bool:
        """
        Change servo ID. Requires EPROM unlock first.

        Args:
            servo_id: Current servo ID
            new_id: New servo ID (1-253)
        """
        self._validate_servo_id(servo_id, allow_broadcast=False)
        self._validate_servo_id(new_id, allow_broadcast=False)
        return self.write_byte(servo_id, Register.ID, new_id)

    def set_baud_rate(self, servo_id: int, baud_rate: int) -> bool:
        """Set servo baud rate. Requires EPROM unlock first."""
        return self.write_byte(servo_id, Register.BAUD_RATE, baud_rate)

    def set_angle_limits(self, servo_id: int, min_angle: int, max_angle: int) -> bool:
        """Set servo angle limits. Requires EPROM unlock first."""
        min_angle = self._validate_position(min_angle)
        max_angle = self._validate_position(max_angle)

        min_l, min_h = self._host2scs(min_angle)
        max_l, max_h = self._host2scs(max_angle)
        data = bytes([min_l, min_h, max_l, max_h])
        return self._gen_write(servo_id, Register.MIN_ANGLE_LIMIT_L, data)

    def read_model(self, servo_id: int) -> int:
        """Read servo model/version."""
        return self.read_word(servo_id, Register.VERSION_L)

    # ========================================================================
    # Utility Methods
    # ========================================================================

    def scan(self, start_id: int = 1, end_id: int = 253) -> List[int]:
        """
        Scan for connected servos.

        Args:
            start_id: Starting ID to scan
            end_id: Ending ID to scan

        Returns:
            List of found servo IDs.
        """
        self._ensure_open()
        assert self._serial is not None

        found = []
        original_timeout = self._serial.timeout
        self._serial.timeout = 0.02  # Short timeout for scanning

        logger.info(f"Scanning for servos {start_id}-{end_id}...")

        try:
            for servo_id in range(start_id, end_id + 1):
                if self.ping(servo_id) != -1:
                    found.append(servo_id)
                    logger.debug(f"Found servo {servo_id}")
        finally:
            self._serial.timeout = original_timeout

        self._found_servos = found
        logger.info(f"Scan complete. Found {len(found)} servos: {found}")
        return found

    def wait_for_move(
        self,
        servo_id: int,
        timeout: float = 10.0,
        threshold: int = 2,
        stable_time: float = 0.05
    ) -> bool:
        """
        Wait for servo to finish moving.

        Uses position stability detection (not the MOVING register, which is
        unreliable on many SCSCL servos). Waits until position stops changing.

        Args:
            servo_id: Servo ID
            timeout: Maximum wait time in seconds
            threshold: Position change threshold to consider "stopped" (default: 2)
            stable_time: Time position must be stable to consider movement done (default: 0.05s)

        Returns:
            True if servo stopped moving, False if timeout.

        Example:
            >>> servo.write_pos(1, 512, 1000)
            >>> servo.wait_for_move(1)  # Waits until position stabilizes
        """
        start = time.time()
        last_pos = self.read_pos(servo_id)
        stable_start = None

        while time.time() - start < timeout:
            time.sleep(0.02)  # 50Hz polling

            current_pos = self.read_pos(servo_id)
            if current_pos == -1:
                continue  # Read error, try again

            # Check if position is stable
            if abs(current_pos - last_pos) <= threshold:
                if stable_start is None:
                    stable_start = time.time()
                elif time.time() - stable_start >= stable_time:
                    # Position has been stable long enough
                    logger.debug(f"Servo {servo_id} movement complete at position {current_pos}")
                    return True
            else:
                # Position changed, reset stability timer
                stable_start = None

            last_pos = current_pos

        logger.warning(f"Servo {servo_id} wait_for_move timed out")
        return False

    def wait_for_position(
        self,
        servo_id: int,
        target: int,
        tolerance: int = 5,
        timeout: float = 10.0
    ) -> bool:
        """
        Wait for servo to reach a specific position.

        NOTE: This waits for the RAW position. If using inverted limits,
        use wait_for_position_safe() instead.

        Args:
            servo_id: Servo ID
            target: Target position to wait for (raw, not inverted)
            tolerance: Acceptable deviation from target (default: 5)
            timeout: Maximum wait time in seconds

        Returns:
            True if position reached, False if timeout.

        Example:
            >>> servo.write_pos(1, 512, 1000)
            >>> servo.wait_for_position(1, 512, tolerance=10)
        """
        start = time.time()
        last_pos = -1

        while time.time() - start < timeout:
            current_pos = self.read_pos(servo_id)
            if current_pos != -1:
                last_pos = current_pos
                if abs(current_pos - target) <= tolerance:
                    logger.debug(f"Servo {servo_id} reached position {current_pos} (target: {target})")
                    return True
            time.sleep(0.02)

        diff = abs(last_pos - target) if last_pos != -1 else "unknown"
        logger.warning(
            f"Servo {servo_id} did not reach position {target} (timeout). "
            f"Last position: {last_pos}, diff: {diff}, tolerance: {tolerance}"
        )
        return False

    def wait_for_position_safe(
        self,
        servo_id: int,
        target: int,
        tolerance: int = 5,
        timeout: float = 10.0
    ) -> bool:
        """
        Wait for servo to reach a position (accounts for limits and inversion).

        Use this with write_pos_safe() when you have inverted servos.

        Args:
            servo_id: Servo ID
            target: Target position (same value you passed to write_pos_safe)
            tolerance: Acceptable deviation from target (default: 5)
            timeout: Maximum wait time in seconds

        Returns:
            True if position reached, False if timeout.

        Example:
            >>> servo.set_limits(2, 50, 900, invert=True)
            >>> servo.write_pos_safe(2, 800, 1000)
            >>> servo.wait_for_position_safe(2, 800)  # Correctly waits for inverted pos
        """
        # Apply the same transformation as write_pos_safe
        actual_target = self._apply_limits(servo_id, target)
        return self.wait_for_position(servo_id, actual_target, tolerance, timeout)

    # ========================================================================
    # Safe Position Control (with limits)
    # ========================================================================

    def set_limits(
        self,
        servo_id: int,
        min_pos: int,
        max_pos: int,
        invert: bool = False
    ) -> 'SCSCL':
        """
        Set position limits for a servo.

        Args:
            servo_id: Servo ID
            min_pos: Minimum allowed position (0-1023)
            max_pos: Maximum allowed position (0-1023)
            invert: If True, flips the position

        Returns:
            self for method chaining.
        """
        self._limits[servo_id] = ServoLimits(
            min_pos=min_pos,
            max_pos=max_pos,
            invert=invert
        )
        logger.debug(f"Set limits for servo {servo_id}: {min_pos}-{max_pos}, invert={invert}")
        return self

    def get_limits(self, servo_id: int) -> Optional[ServoLimits]:
        """Get position limits for a servo."""
        return self._limits.get(servo_id)

    def has_limits(self, servo_id: int) -> bool:
        """Check if limits are configured for a servo."""
        return servo_id in self._limits

    def clear_limits(self, servo_id: int) -> None:
        """Remove position limits for a servo."""
        self._limits.pop(servo_id, None)

    def clear_all_limits(self) -> None:
        """Remove position limits for all servos."""
        self._limits.clear()

    def _apply_limits(self, servo_id: int, position: int) -> int:
        """Apply limits to a position value."""
        limits = self._limits.get(servo_id)
        if limits is None:
            raise ValidationError(
                f"Limits not configured for servo {servo_id}. "
                f"Call set_limits({servo_id}, min_pos, max_pos) first."
            )

        position = max(limits.min_pos, min(limits.max_pos, position))

        if limits.invert:
            position = limits.max_pos - position + limits.min_pos

        return position

    def write_pos_safe(
        self,
        servo_id: int,
        position: int,
        time_ms: int,
        speed: int = 0
    ) -> bool:
        """
        Write position with limit checking and optional invert.

        REQUIRES: Call set_limits() first for this servo.
        """
        safe_position = self._apply_limits(servo_id, position)
        return self.write_pos(servo_id, safe_position, time_ms, speed)

    def sync_write_pos_safe(
        self,
        servo_ids: List[int],
        positions: List[int],
        times: List[int],
        speeds: Optional[List[int]] = None
    ) -> None:
        """
        Synchronously write positions with limit checking.

        REQUIRES: Call set_limits() first for ALL servos.
        """
        safe_positions = [
            self._apply_limits(sid, pos)
            for sid, pos in zip(servo_ids, positions)
        ]
        self.sync_write_pos(servo_ids, safe_positions, times, speeds)

    # ========================================================================
    # Servo s
    # ========================================================================

    def create_group(self, name: str, servo_ids: List[int]) -> 'SCSCL':
        """
        Create a named group of servos.

        Args:
            name: Group name (e.g., "hand", "arm", "fingers")
            servo_ids: List of servo IDs in the group

        Returns:
            self for method chaining.

        Example:
            >>> servo.create_group("hand", [1, 2, 3, 4, 5])
            >>> servo.group_write_pos("hand", [512]*5, time=1000)
        """
        for sid in servo_ids:
            self._validate_servo_id(sid, allow_broadcast=False)

        self._groups[name] = ServoGroup(name=name, servo_ids=servo_ids)
        logger.debug(f"Created group '{name}': {servo_ids}")
        return self

    def get_group(self, name: str) -> Optional[ServoGroup]:
        """Get a servo group by name."""
        return self._groups.get(name)

    def delete_group(self, name: str) -> None:
        """Delete a servo group."""
        self._groups.pop(name, None)

    def group_write_pos(
        self,
        group_name: str,
        positions: List[int],
        time_ms: int,
        speeds: Optional[List[int]] = None
    ) -> None:
        """
        Write positions to all servos in a group.

        Args:
            group_name: Name of the servo group
            positions: List of positions (one per servo in group)
            time_ms: Movement time in milliseconds
            speeds: Optional list of speeds

        Example:
            >>> servo.create_group("hand", [1, 2, 3, 4, 5])
            >>> servo.group_write_pos("hand", [512, 512, 512, 512, 512], time=1000)
        """
        group = self._groups.get(group_name)
        if group is None:
            raise ValidationError(f"Group '{group_name}' not found")

        if len(positions) != len(group):
            raise ValidationError(
                f"Expected {len(group)} positions for group '{group_name}', got {len(positions)}"
            )

        times = [time_ms] * len(group)
        self.sync_write_pos(group.servo_ids, positions, times, speeds)

    def group_write_pos_safe(
        self,
        group_name: str,
        positions: List[int],
        time_ms: int,
        speeds: Optional[List[int]] = None
    ) -> None:
        """
        Write positions to group with limit checking.

        REQUIRES: set_limits() for all servos in group.
        """
        group = self._groups.get(group_name)
        if group is None:
            raise ValidationError(f"Group '{group_name}' not found")

        if len(positions) != len(group):
            raise ValidationError(
                f"Expected {len(group)} positions for group '{group_name}', got {len(positions)}"
            )

        times = [time_ms] * len(group)
        self.sync_write_pos_safe(group.servo_ids, positions, times, speeds)

    def group_enable_torque(self, group_name: str, enable: bool = True) -> None:
        """Enable/disable torque for all servos in a group."""
        group = self._groups.get(group_name)
        if group is None:
            raise ValidationError(f"Group '{group_name}' not found")

        for servo_id in group.servo_ids:
            self.enable_torque(servo_id, enable)

    def group_read_pos(self, group_name: str) -> Dict[int, int]:
        """Read positions from all servos in a group."""
        group = self._groups.get(group_name)
        if group is None:
            raise ValidationError(f"Group '{group_name}' not found")

        return self.sync_read_pos(group.servo_ids)

    def group_wait_for_move(
        self,
        group_name: str,
        timeout: float = 10.0,
        threshold: int = 2,
        stable_time: float = 0.05
    ) -> bool:
        """
        Wait for all servos in a group to finish moving.

        Uses position stability detection. Waits until ALL servos stop moving.

        Args:
            group_name: Name of the servo group
            timeout: Maximum wait time in seconds
            threshold: Position change threshold to consider "stopped" (default: 2)
            stable_time: Time positions must be stable (default: 0.05s)

        Returns:
            True if all servos stopped, False if timeout.

        Example:
            >>> servo.group_write_pos_safe("hand", [500]*5, time_ms=1000)
            >>> servo.group_wait_for_move("hand")
        """
        group = self._groups.get(group_name)
        if group is None:
            raise ValidationError(f"Group '{group_name}' not found")

        start = time.time()
        last_positions = {sid: self.read_pos(sid) for sid in group.servo_ids}
        stable_start = None

        while time.time() - start < timeout:
            time.sleep(0.02)  # 50Hz polling

            # Read all positions
            current_positions = {}
            all_stable = True

            for sid in group.servo_ids:
                pos = self.read_pos(sid)
                if pos == -1:
                    all_stable = False
                    continue

                current_positions[sid] = pos
                last_pos = last_positions.get(sid, pos)

                # Check if this servo is stable
                if abs(pos - last_pos) > threshold:
                    all_stable = False

            # Update last positions
            last_positions.update(current_positions)

            # Check stability
            if all_stable:
                if stable_start is None:
                    stable_start = time.time()
                elif time.time() - stable_start >= stable_time:
                    logger.debug(f"Group '{group_name}' movement complete")
                    return True
            else:
                stable_start = None

        logger.warning(f"Group '{group_name}' wait_for_move timed out. Last positions: {last_positions}")
        return False

    # ========================================================================
    # Smooth Movement / Interpolation
    # ========================================================================

    def move_smooth(
        self,
        servo_id: int,
        target: int,
        duration_ms: int = 1000,
        steps: int = 20
    ) -> None:
        """
        Smoothly interpolate to target position.

        Creates smoother motion by sending multiple intermediate positions.

        Args:
            servo_id: Servo ID
            target: Target position (0-1023)
            duration_ms: Total movement duration in milliseconds
            steps: Number of intermediate steps

        Example:
            >>> servo.move_smooth(1, 800, duration_ms=2000, steps=50)
        """
        target = self._validate_position(target)

        # Get current position
        current = self.read_pos(servo_id)
        if current == -1:
            raise CommunicationError(f"Failed to read position from servo {servo_id}")

        step_time = duration_ms / steps
        step_delay = step_time / 1000.0  # Convert to seconds

        for i in range(1, steps + 1):
            # Linear interpolation
            t = i / steps
            pos = int(current + (target - current) * t)

            self.write_pos(servo_id, pos, int(step_time))
            time.sleep(step_delay)

    def group_move_smooth(
        self,
        group_name: str,
        targets: List[int],
        duration_ms: int = 1000,
        steps: int = 20
    ) -> None:
        """
        Smoothly move all servos in a group to target positions.

        Args:
            group_name: Group name
            targets: List of target positions
            duration_ms: Total movement duration
            steps: Number of intermediate steps
        """
        group = self._groups.get(group_name)
        if group is None:
            raise ValidationError(f"Group '{group_name}' not found")

        if len(targets) != len(group):
            raise ValidationError(
                f"Expected {len(group)} targets for group '{group_name}', got {len(targets)}"
            )

        targets = [self._validate_position(t) for t in targets]

        # Get current positions
        current_positions = self.sync_read_pos(group.servo_ids)
        currents = [current_positions.get(sid, 512) for sid in group.servo_ids]

        step_time = duration_ms / steps
        step_delay = step_time / 1000.0

        for i in range(1, steps + 1):
            t = i / steps
            positions = [
                int(curr + (tgt - curr) * t)
                for curr, tgt in zip(currents, targets)
            ]

            times = [int(step_time)] * len(group)
            self.sync_write_pos(group.servo_ids, positions, times)
            time.sleep(step_delay)

    # ========================================================================
    # Auto-Discovery
    # ========================================================================

    # Known USB-to-Serial chip vendors (VID:PID patterns)
    _KNOWN_SERIAL_CHIPS = {
        # CH340/CH341
        (0x1A86, 0x7523): "CH340",
        (0x1A86, 0x5523): "CH341",
        # CP210x
        (0x10C4, 0xEA60): "CP2102",
        (0x10C4, 0xEA70): "CP2105",
        (0x10C4, 0xEA71): "CP2108",
        # FTDI
        (0x0403, 0x6001): "FT232R",
        (0x0403, 0x6010): "FT2232",
        (0x0403, 0x6011): "FT4232",
        (0x0403, 0x6014): "FT232H",
        (0x0403, 0x6015): "FT-X",
        # Prolific
        (0x067B, 0x2303): "PL2303",
        # Arduino
        (0x2341, None): "Arduino",
        (0x1A86, 0x55D4): "CH9102",
    }

    @staticmethod
    def list_ports() -> List[str]:
        """
        List available serial ports.

        Returns:
            List of port paths sorted by likelihood of being a servo adapter.
        """
        ports = set()

        for port in serial.tools.list_ports.comports():
            ports.add(port.device)

        # Platform-specific patterns
        if sys.platform.startswith('linux'):
            ports.update(glob.glob('/dev/ttyUSB*'))
            ports.update(glob.glob('/dev/ttyACM*'))
        elif sys.platform == 'darwin':  # macOS
            ports.update(glob.glob('/dev/cu.usbserial*'))
            ports.update(glob.glob('/dev/cu.usbmodem*'))
            ports.update(glob.glob('/dev/cu.wchusbserial*'))  # CH340 on macOS
            ports.update(glob.glob('/dev/cu.SLAB_USBtoUART*'))  # CP210x on macOS

        return sorted(ports)

    @classmethod
    def list_ports_detailed(cls) -> List[Dict[str, any]]:
        """
        List available serial ports with detailed information.

        Returns:
            List of dicts with port details: device, description, vendor, chip.

        Example:
            >>> for port in SCSCL.list_ports_detailed():
            ...     print(f"{port['device']}: {port['chip']} - {port['description']}")
        """
        result = []

        for port in serial.tools.list_ports.comports():
            info = {
                'device': port.device,
                'description': port.description or 'Unknown',
                'vendor': port.manufacturer or 'Unknown',
                'vid': port.vid,
                'pid': port.pid,
                'serial': port.serial_number,
                'chip': None,
                'score': 0,  # Higher = more likely to be servo adapter
            }

            # Identify chip type
            if port.vid and port.pid:
                chip = cls._KNOWN_SERIAL_CHIPS.get((port.vid, port.pid))
                if chip:
                    info['chip'] = chip
                    info['score'] += 10
                else:
                    # Check vendor-only matches
                    for (vid, pid), name in cls._KNOWN_SERIAL_CHIPS.items():
                        if vid == port.vid and pid is None:
                            info['chip'] = name
                            info['score'] += 5
                            break

            # Score based on description
            desc_lower = (port.description or '').lower()
            if 'usb' in desc_lower and 'serial' in desc_lower:
                info['score'] += 5
            if 'ch340' in desc_lower or 'ch341' in desc_lower:
                info['chip'] = info['chip'] or 'CH340'
                info['score'] += 8
            if 'cp210' in desc_lower:
                info['chip'] = info['chip'] or 'CP210x'
                info['score'] += 8
            if 'ftdi' in desc_lower or 'ft232' in desc_lower:
                info['chip'] = info['chip'] or 'FTDI'
                info['score'] += 8
            if 'bluetooth' in desc_lower or 'wireless' in desc_lower:
                info['score'] -= 10  # Less likely to be servo

            result.append(info)

        # Sort by score (highest first)
        result.sort(key=lambda x: (-x['score'], x['device']))

        return result

    @classmethod
    def find_port(cls) -> Optional[str]:
        """
        Auto-detect the most likely serial port for servo communication.

        This method finds USB serial adapters without scanning for servos,
        making it faster than auto_connect().

        Returns:
            Port path (e.g., '/dev/ttyUSB0' or '/dev/cu.usbserial-110') or None.

        Example:
            >>> port = SCSCL.find_port()
            >>> if port:
            ...     servo = SCSCL(port).connect()
        """
        detailed = cls.list_ports_detailed()

        # Return highest scored port if it has a positive score
        if detailed and detailed[0]['score'] > 0:
            logger.info(f"Auto-detected port: {detailed[0]['device']} ({detailed[0]['chip'] or 'USB Serial'})")
            return detailed[0]['device']

        # Fallback: return first USB serial port by pattern
        ports = cls.list_ports()
        for port in ports:
            # Prioritize known patterns
            if any(pattern in port for pattern in ['ttyUSB', 'ttyACM', 'usbserial', 'usbmodem', 'wchusbserial', 'SLAB']):
                logger.info(f"Auto-detected port (by pattern): {port}")
                return port

        # Last resort: return first available port
        if ports:
            logger.info(f"Auto-detected port (first available): {ports[0]}")
            return ports[0]

        logger.warning("No serial ports found")
        return None

    @classmethod
    def find_port_or_raise(cls) -> str:
        """
        Auto-detect serial port or raise an error.

        Returns:
            Port path.

        Raises:
            CommunicationError: If no serial port found.

        Example:
            >>> port = SCSCL.find_port_or_raise()
            >>> servo = SCSCL(port).connect()
        """
        port = cls.find_port()
        if port is None:
            raise CommunicationError(
                "No serial port found. Check USB connection.\n"
                f"Platform: {sys.platform}\n"
                "Expected ports:\n"
                "  Linux: /dev/ttyUSB0, /dev/ttyACM0\n"
                "  macOS: /dev/cu.usbserial-xxx, /dev/cu.usbmodem-xxx"
            )
        return port

    @classmethod
    def quick_connect(cls, baudrate: int = 1000000, **kwargs) -> 'SCSCL':
        """
        Quickly connect to auto-detected port without scanning for servos.

        Faster than auto_connect() - just finds the port and connects.
        Use this when you know servos are connected and just need the port.

        Args:
            baudrate: Baud rate (default: 1000000)
            **kwargs: Additional arguments for SCSCL constructor

        Returns:
            Connected SCSCL instance.

        Raises:
            CommunicationError: If no port found or connection fails.

        Example:
            >>> servo = SCSCL.quick_connect()
            >>> servo.ping(1)
        """
        port = cls.find_port_or_raise()
        instance = cls(port=port, baudrate=baudrate, **kwargs)
        instance.open()
        logger.info(f"Quick-connected to {port}")
        return instance

    @classmethod
    def auto_connect(
        cls,
        baudrate: int = 1000000,
        scan_ids: Tuple[int, int] = (1, 10),
        **kwargs
    ) -> 'SCSCL':
        """
        Automatically find port and scan for servos.

        This is the most thorough auto-connect method - it finds ports,
        tries each one, and scans for responding servos.

        Args:
            baudrate: Baud rate to use (default: 1000000)
            scan_ids: Range of servo IDs to scan (start, end)
            **kwargs: Additional arguments passed to constructor

        Returns:
            Connected SCSCL instance with found_servos populated.

        Example:
            >>> servo = SCSCL.auto_connect()
            >>> print(f"Found servos: {servo.found_servos}")
        """
        # Get ports sorted by likelihood
        detailed = cls.list_ports_detailed()
        ports = [p['device'] for p in detailed] if detailed else cls.list_ports()

        if not ports:
            raise CommunicationError(
                "No serial ports found. Check USB connection.\n"
                f"Platform: {sys.platform}"
            )

        logger.info(f"Auto-connecting, trying ports: {ports}")

        for port in ports:
            try:
                instance = cls(port=port, baudrate=baudrate, **kwargs)
                instance.open()

                found = instance.scan(scan_ids[0], scan_ids[1])

                if found:
                    instance._found_servos = found
                    logger.info(f"Auto-connected to {port}, found servos: {found}")
                    return instance

                instance.close()
            except (CommunicationError, serial.SerialException) as e:
                logger.debug(f"Failed to connect to {port}: {e}")
                continue

        raise CommunicationError(f"No servos found on any port. Tried: {ports}")

    @property
    def found_servos(self) -> List[int]:
        """List of servo IDs found during auto_connect() or scan()."""
        return self._found_servos.copy()

    # ========================================================================
    # Fluent API
    # ========================================================================

    def with_limits(self, limits: Dict[int, Tuple]) -> 'SCSCL':
        """
        Set limits for multiple servos at once.

        Args:
            limits: Dict mapping servo_id to (min_pos, max_pos) or (min_pos, max_pos, invert)

        Example:
            >>> servo = (SCSCL('/dev/ttyUSB0')
            ...     .connect()
            ...     .with_limits({
            ...         1: (50, 900),
            ...         2: (0, 1023, True),
            ...     }))
        """
        for servo_id, values in limits.items():
            if len(values) == 2:
                min_pos, max_pos = values
                invert = False
            else:
                min_pos, max_pos, invert = values
            self.set_limits(servo_id, min_pos, max_pos, invert)
        return self

    def with_max_speed(self, speeds: Dict[int, int]) -> 'SCSCL':
        """
        Set max speed limits for multiple servos.

        Args:
            speeds: Dict mapping servo_id to max_speed

        Example:
            >>> servo.with_max_speed({1: 500, 2: 300})
        """
        for servo_id, max_speed in speeds.items():
            self.set_max_speed(servo_id, max_speed)
        return self

    def with_groups(self, groups: Dict[str, List[int]]) -> 'SCSCL':
        """
        Create multiple servo groups at once.

        Args:
            groups: Dict mapping group_name to list of servo_ids

        Example:
            >>> servo.with_groups({
            ...     "hand": [1, 2, 3, 4, 5],
            ...     "wrist": [6],
            ... })
        """
        for name, servo_ids in groups.items():
            self.create_group(name, servo_ids)
        return self

    # ========================================================================
    # Safety Features
    # ========================================================================

    def emergency_stop(self) -> None:
        """
        EMERGENCY STOP - Immediately disable torque on ALL servos.

        Use in emergency situations to prevent damage.
        """
        logger.warning("EMERGENCY STOP - Disabling all torque")
        self.enable_torque(BROADCAST_ID, False)

    def set_max_speed(self, servo_id: int, max_speed: int) -> 'SCSCL':
        """
        Set maximum speed limit for a servo.

        When set, all position commands will have speed clamped to this value.
        """
        if max_speed > 0:
            self._max_speeds[servo_id] = max_speed
        else:
            self._max_speeds.pop(servo_id, None)
        return self

    def get_max_speed(self, servo_id: int) -> Optional[int]:
        """Get max speed limit for a servo."""
        return self._max_speeds.get(servo_id)

    def clear_max_speed(self, servo_id: int) -> None:
        """Remove max speed limit for a servo."""
        self._max_speeds.pop(servo_id, None)

    def _apply_max_speed(self, servo_id: int, speed: int) -> int:
        """Apply max speed limit if configured."""
        max_speed = self._max_speeds.get(servo_id)
        if max_speed is not None and speed > max_speed:
            return max_speed
        return speed

    # ========================================================================
    # Retry Logic
    # ========================================================================

    def _with_retry(self, operation: Callable, *args, **kwargs):
        """Execute operation with retry logic."""
        last_error = None

        for attempt in range(self.retry_count):
            try:
                return operation(*args, **kwargs)
            except (serial.SerialException, CommunicationError) as e:
                last_error = e
                logger.warning(f"Operation failed (attempt {attempt + 1}/{self.retry_count}): {e}")

                if not self.auto_reconnect:
                    raise

                if attempt < self.retry_count - 1:
                    try:
                        self.reconnect()
                    except CommunicationError:
                        pass

        raise CommunicationError(
            f"Operation failed after {self.retry_count} attempts: {last_error}"
        )

    def write_pos_retry(
        self,
        servo_id: int,
        position: int,
        time_ms: int,
        speed: int = 0
    ) -> bool:
        """Write position with auto-retry on failure."""
        return self._with_retry(self.write_pos, servo_id, position, time_ms, speed)

    def ping_retry(self, servo_id: int) -> int:
        """Ping with auto-retry on failure."""
        return self._with_retry(self.ping, servo_id)

    # ========================================================================
    # Factory Methods
    # ========================================================================

    @classmethod
    def for_robotic_hand(
        cls,
        port: str,
        baudrate: int = 1000000,
        limits: Optional[Dict[int, Tuple]] = None
    ) -> 'SCSCL':
        """
        Pre-configured SCSCL for a 5-finger robotic hand.

        Args:
            port: Serial port
            baudrate: Baud rate
            limits: Optional custom limits, or use defaults

        Returns:
            Configured SCSCL instance.

        Example:
            >>> servo = SCSCL.for_robotic_hand('/dev/ttyUSB0')
            >>> servo.group_write_pos("hand", [512]*5, time=1000)
        """
        if limits is None:
            limits = {
                1: (0, 900),   # Thumb
                2: (0, 900),   # Index
                3: (0, 900),   # Middle
                4: (0, 900),   # Ring
                5: (0, 900),   # Pinky
            }

        return (cls(port, baudrate=baudrate)
            .connect()
            .with_limits(limits)
            .with_groups({"hand": [1, 2, 3, 4, 5]}))

    @classmethod
    def for_single_servo(cls, port: str, servo_id: int = 1, baudrate: int = 1000000) -> 'SCSCL':
        """
        Pre-configured SCSCL for a single servo.

        Example:
            >>> servo = SCSCL.for_single_servo('/dev/ttyUSB0', servo_id=1)
        """
        return cls(port, baudrate=baudrate).connect()
