import time
from dataclasses import dataclass
from typing import List, Optional, Sequence

import serial


CRSF_ADDRESS = 0xC8
CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16
CRSF_RC_PAYLOAD_LENGTH = 22
CRSF_CHANNEL_COUNT = 16
CRSF_CRC_POLY = 0xD5

CRSF_CHANNEL_MIN = 172
CRSF_CHANNEL_MID = 992
CRSF_CHANNEL_MAX = 1811


@dataclass(frozen=True)
class ELRSFrame:
    channels: tuple[int, ...]
    timestamp: float
    packet: bytes


class ELRSDriver:
    """
    ELRS / CRSF serial receiver driver.

    The driver keeps the latest valid RC frame and supports both:
    - `poll()`: non-blocking update, suitable for a robot control loop
    - `read()`: blocking read until a fresh frame is received or timeout
    """

    def __init__(
        self,
        serial_port: str,
        baudrate: int = 420000,
        timeout: float = 0.02,
        max_buffer_size: int = 4096,
    ) -> None:
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.timeout = timeout
        self.max_buffer_size = max_buffer_size

        self.port_handler: Optional[serial.Serial] = None
        self.buffer = bytearray()

        self.last_frame: Optional[ELRSFrame] = None
        self.last_channels: Optional[List[int]] = None
        self.last_frame_time: Optional[float] = None
        self.frame_count = 0

    def hardware_init(self) -> bool:
        """Open the serial port."""
        try:
            self.port_handler = serial.Serial(
                self.serial_port,
                baudrate=self.baudrate,
                timeout=self.timeout,
            )
            return True
        except Exception as exc:
            print(f"[ERROR] Failed to open ELRS serial port {self.serial_port}: {exc}")
            self.port_handler = None
            return False

    def close(self) -> None:
        """Close the serial port if it is open."""
        if self.port_handler and self.port_handler.is_open:
            self.port_handler.close()

    def online_check(self, timeout: float = 0.5, max_age: float = 0.5) -> bool:
        """Return True when a fresh RC frame is available."""
        frame = self.read(timeout=timeout)
        if frame is None:
            return False
        return self.is_data_fresh(max_age=max_age)

    def is_data_fresh(self, max_age: float = 0.5) -> bool:
        """Check whether the latest decoded frame is recent enough."""
        if self.last_frame_time is None:
            return False
        return (time.time() - self.last_frame_time) <= max_age

    def poll(self) -> Optional[ELRSFrame]:
        """
        Non-blocking serial poll.

        Reads currently available bytes, parses all complete packets, and
        returns the latest valid RC frame received in this poll cycle.
        """
        if self.port_handler is None or not self.port_handler.is_open:
            raise RuntimeError("ELRS serial port is not initialized. Call hardware_init() first.")

        frames: List[ELRSFrame] = []
        bytes_waiting = self.port_handler.in_waiting
        if bytes_waiting:
            data = self.port_handler.read(bytes_waiting)
            if data:
                frames = self.feed(data)

        if frames:
            return frames[-1]
        return None

    def read(self, timeout: float = 0.2, sleep_interval: float = 0.002) -> Optional[ELRSFrame]:
        """Block until a valid RC frame is received or timeout expires."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            frame = self.poll()
            if frame is not None:
                return frame
            time.sleep(sleep_interval)
        return None

    def feed(self, data: bytes) -> List[ELRSFrame]:
        """Feed raw bytes into the parser and return all decoded valid RC frames."""
        self.buffer.extend(data)
        if len(self.buffer) > self.max_buffer_size:
            self.buffer = self.buffer[-self.max_buffer_size :]

        frames: List[ELRSFrame] = []

        while len(self.buffer) >= 3:
            if self.buffer[0] != CRSF_ADDRESS:
                self.buffer.pop(0)
                continue

            frame_len = self.buffer[1]
            total_len = frame_len + 2
            if total_len < 5:
                self.buffer.pop(0)
                continue

            if len(self.buffer) < total_len:
                break

            packet = bytes(self.buffer[:total_len])
            del self.buffer[:total_len]

            frame = self.parse_crsf_packet(packet)
            if frame is not None:
                self.last_frame = frame
                self.last_channels = list(frame.channels)
                self.last_frame_time = frame.timestamp
                self.frame_count += 1
                frames.append(frame)

        return frames

    def parse_crsf_packet(self, packet: bytes) -> Optional[ELRSFrame]:
        """Parse a single CRSF packet and return an RC frame when valid."""
        if len(packet) < 5:
            return None

        addr = packet[0]
        length = packet[1]
        if length + 2 != len(packet):
            return None

        frame_type = packet[2]
        payload = packet[3:-1]
        crc = packet[-1]

        if self.compute_crc(packet[2:-1]) != crc:
            return None

        if addr != CRSF_ADDRESS or frame_type != CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            return None

        if len(payload) != CRSF_RC_PAYLOAD_LENGTH:
            return None

        bits = int.from_bytes(payload, byteorder="little")
        channels = []
        for channel_index in range(CRSF_CHANNEL_COUNT):
            value = (bits >> (channel_index * 11)) & 0x7FF
            channels.append(value)

        return ELRSFrame(
            channels=tuple(channels),
            timestamp=time.time(),
            packet=packet,
        )

    @staticmethod
    def compute_crc(data: bytes) -> int:
        """Compute CRSF CRC-8 using the DVB-S2 polynomial."""
        crc = 0
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ CRSF_CRC_POLY) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc

    def get_channels(self, max_age: Optional[float] = None) -> Optional[List[int]]:
        """Return the latest channel list, optionally requiring fresh data."""
        if self.last_channels is None:
            return None
        if max_age is not None and not self.is_data_fresh(max_age=max_age):
            return None
        return list(self.last_channels)

    def get_channel(self, index: int, default: Optional[int] = None, max_age: Optional[float] = None) -> Optional[int]:
        """Return one channel by 0-based index."""
        channels = self.get_channels(max_age=max_age)
        if channels is None:
            return default
        if not (0 <= index < len(channels)):
            raise IndexError(f"Channel index out of range: {index}")
        return channels[index]

    def get_axis(self, index: int, deadband: float = 0.05, max_age: Optional[float] = None) -> Optional[float]:
        """
        Return a normalized axis value in [-1.0, 1.0].

        ELRS / CRSF channel defaults are mapped with asymmetric ranges around
        the center value because the protocol nominal range is not perfectly
        symmetric.
        """
        raw = self.get_channel(index, max_age=max_age)
        if raw is None:
            return None

        if raw >= CRSF_CHANNEL_MID:
            value = (raw - CRSF_CHANNEL_MID) / (CRSF_CHANNEL_MAX - CRSF_CHANNEL_MID)
        else:
            value = (raw - CRSF_CHANNEL_MID) / (CRSF_CHANNEL_MID - CRSF_CHANNEL_MIN)

        if abs(value) < deadband:
            return 0.0
        return max(-1.0, min(1.0, value))

    def get_switch_state(
        self,
        index: int,
        low_threshold: int = 680,
        high_threshold: int = 1300,
        max_age: Optional[float] = None,
    ) -> Optional[int]:
        """
        Return a coarse switch state from one channel.

        Returns:
        - `0` for low
        - `1` for middle
        - `2` for high
        """
        raw = self.get_channel(index, max_age=max_age)
        if raw is None:
            return None
        if raw < low_threshold:
            return 0
        if raw > high_threshold:
            return 2
        return 1

    def get_frame_age(self) -> Optional[float]:
        """Return the age of the latest decoded frame in seconds."""
        if self.last_frame_time is None:
            return None
        return time.time() - self.last_frame_time

    @staticmethod
    def channels_to_dict(channels: Sequence[int]) -> dict[str, int]:
        """Convert a channel sequence into a CH1..CH16 mapping."""
        return {f"ch{index + 1}": value for index, value in enumerate(channels)}

