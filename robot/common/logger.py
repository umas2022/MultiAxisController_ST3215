"""Shared logging helpers for robot controllers and applications."""

from __future__ import annotations

import logging
import socket
import sys
from dataclasses import dataclass
from typing import Optional, Protocol, TextIO


DEFAULT_FORMAT = "%(asctime)s | %(levelname)-8s | %(name)s | %(message)s"
DEFAULT_DATE_FORMAT = "%Y-%m-%d %H:%M:%S"


class SerialWriter(Protocol):
    """Minimal interface implemented by ``serial.Serial`` and test doubles."""

    def write(self, data: bytes) -> object:
        ...

    def flush(self) -> object:
        ...


@dataclass(frozen=True)
class LoggerConfig:
    """Configuration shared by all robot loggers."""

    level: int | str = logging.INFO
    console: bool = True
    udp_host: Optional[str] = None
    udp_port: int = 4211
    encoding: str = "utf-8"
    format: str = DEFAULT_FORMAT
    date_format: str = DEFAULT_DATE_FORMAT


class SerialLogHandler(logging.Handler):
    """Write one UTF-8 encoded log record per line to an open serial link."""

    def __init__(self, serial_writer: SerialWriter, encoding: str = "utf-8") -> None:
        super().__init__()
        self.serial_writer = serial_writer
        self.encoding = encoding

    def emit(self, record: logging.LogRecord) -> None:
        try:
            message = self.format(record) + "\n"
            self.serial_writer.write(message.encode(self.encoding, errors="replace"))
            flush = getattr(self.serial_writer, "flush", None)
            if callable(flush):
                flush()
        except Exception:
            self.handleError(record)


class UdpLogHandler(logging.Handler):
    """Send each formatted log record as a UDP datagram."""

    def __init__(self, host: str, port: int, encoding: str = "utf-8") -> None:
        super().__init__()
        if not host:
            raise ValueError("UDP log host must not be empty")
        if not 1 <= port <= 65535:
            raise ValueError(f"Invalid UDP log port: {port}")

        self.address = (host, port)
        self.encoding = encoding
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def emit(self, record: logging.LogRecord) -> None:
        try:
            payload = self.format(record).encode(self.encoding, errors="replace")
            self.socket.sendto(payload, self.address)
        except Exception:
            self.handleError(record)

    def close(self) -> None:
        self.socket.close()
        super().close()


def _parse_level(level: int | str) -> int:
    if isinstance(level, int):
        return level

    value = logging.getLevelName(level.upper())
    if not isinstance(value, int):
        raise ValueError(f"Unknown log level: {level}")
    return value


def get_logger(
    name: str,
    config: Optional[LoggerConfig] = None,
    *,
    serial_writer: Optional[SerialWriter] = None,
    console_stream: Optional[TextIO] = None,
) -> logging.Logger:
    """Create or reconfigure a named robot logger.

    Calling this function repeatedly with the same name replaces handlers
    created by this module, so logs are never duplicated accidentally.
    The caller retains ownership of ``serial_writer``; closing the logger does
    not close the serial connection.
    """

    if not name:
        raise ValueError("Logger name must not be empty")

    config = config or LoggerConfig()
    level = _parse_level(config.level)
    formatter = logging.Formatter(config.format, config.date_format)
    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.propagate = False

    for handler in list(logger.handlers):
        if getattr(handler, "_robot_common_handler", False):
            logger.removeHandler(handler)
            handler.close()

    handlers: list[logging.Handler] = []

    if config.console:
        handlers.append(logging.StreamHandler(console_stream or sys.stderr))

    if serial_writer is not None:
        handlers.append(SerialLogHandler(serial_writer, config.encoding))

    if config.udp_host is not None:
        handlers.append(UdpLogHandler(config.udp_host, config.udp_port, config.encoding))

    if not handlers:
        handlers.append(logging.NullHandler())

    for handler in handlers:
        handler.setLevel(level)
        handler.setFormatter(formatter)
        handler._robot_common_handler = True  # type: ignore[attr-defined]
        logger.addHandler(handler)

    return logger


def close_logger(logger: logging.Logger) -> None:
    """Flush, close, and remove handlers created by :func:`get_logger`."""

    for handler in list(logger.handlers):
        if getattr(handler, "_robot_common_handler", False):
            logger.removeHandler(handler)
            handler.close()


__all__ = [
    "LoggerConfig",
    "SerialLogHandler",
    "UdpLogHandler",
    "close_logger",
    "get_logger",
]
