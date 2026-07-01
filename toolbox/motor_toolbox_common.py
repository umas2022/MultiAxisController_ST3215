#!/usr/bin/env python3
"""
Shared helpers for motor toolbox scripts.
"""

import argparse
import os
import sys

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DRIVERS_SRC = os.path.join(PROJECT_ROOT, "drivers", "src")

for extra_path in (PROJECT_ROOT, DRIVERS_SRC):
    if extra_path not in sys.path:
        sys.path.append(extra_path)

from multiaxis_driver.motor import MultiAxisSerial, MultiAxisUdp, MultiAxisUSB


def add_connection_args(parser: argparse.ArgumentParser, default_port: str = "COM5") -> None:
    parser.add_argument(
        "--mode",
        choices=("usb", "serial", "udp"),
        default="usb",
        help="Connection method. usb=host direct to motor bus, serial=ESP32 serial/Bluetooth COM port, udp=ESP32 Wi-Fi UDP.",
    )
    parser.add_argument(
        "--port",
        default=default_port,
        help='Serial port for usb/serial mode, for example "COM6" or "COM7".',
    )
    parser.add_argument("--serial-baudrate", type=int, default=115200, help="ESP32 serial/Bluetooth baudrate. Default: 115200")
    parser.add_argument("--udp-ip", default="192.168.4.1", help="ESP32 UDP target IP. Default: 192.168.4.1")
    parser.add_argument("--udp-port", type=int, default=4210, help="ESP32 UDP target port. Default: 4210")
    parser.add_argument("--local-ip", default="0.0.0.0", help='Local bind IP for UDP mode. Default: "0.0.0.0"')
    parser.add_argument("--local-port", type=int, default=4210, help="Local UDP bind port. Default: 4210")


def create_motor_driver(args):
    if args.mode == "usb":
        driver = MultiAxisUSB(args.port)
    elif args.mode == "serial":
        driver = MultiAxisSerial(args.port, baudrate=args.serial_baudrate)
    elif args.mode == "udp":
        driver = MultiAxisUdp(
            esp32_ip=args.udp_ip,
            esp32_port=args.udp_port,
            local_ip=args.local_ip,
            local_port=args.local_port,
        )
    else:
        raise ValueError(f"Unsupported mode: {args.mode}")

    if not driver.hardware_init():
        raise RuntimeError(f"Failed to initialize motor driver in {args.mode} mode")
    return driver
