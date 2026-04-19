#!/usr/bin/env python3
"""
Scan ST3215 / STS servo IDs on the bus.
"""

import argparse
import os
import sys
from typing import List

# Add project root to import path.
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(project_root)

from robot.src.drivers.motor_driver.STservo_sdk import COMM_SUCCESS, MAX_ID, PortHandler, sts


def scan_motors(serial_port: str, baudrate: int = 1_000_000, start_id: int = 1, end_id: int = 20) -> List[int]:
    """Scan the bus and return all online motor IDs."""
    if not (0 <= start_id <= MAX_ID and 0 <= end_id <= MAX_ID):
        print(f"[ERROR] Invalid scan range {start_id}-{end_id}. Valid range is 0-{MAX_ID}.")
        return []
    if start_id > end_id:
        print(f"[ERROR] Invalid scan range {start_id}-{end_id}. start_id must be <= end_id.")
        return []

    online_motors: List[int] = []

    try:
        port_handler = PortHandler(serial_port)
    except Exception as exc:
        print(f"[ERROR] Failed to init PortHandler: {exc}")
        return online_motors

    if not port_handler.openPort():
        print(f"[ERROR] Could not open port: {serial_port}")
        return online_motors

    if not port_handler.setBaudRate(baudrate):
        print(f"[ERROR] Could not set baudrate: {baudrate}")
        port_handler.closePort()
        return online_motors

    packet_handler = sts(port_handler)

    try:
        print(f"[INFO] Scanning motor IDs {start_id}-{end_id}")
        print(f"[INFO] Port={serial_port}, baudrate={baudrate}")
        print("-" * 50)

        for motor_id in range(start_id, end_id + 1):
            model_number, comm_result, sts_error = packet_handler.ping(motor_id)
            if comm_result == COMM_SUCCESS:
                online_motors.append(motor_id)
                print(f"[OK] ID {motor_id:3d} online, model={model_number}")
                if sts_error != 0:
                    print(f"[WARN] ID {motor_id:3d} servo error: {packet_handler.getRxPacketError(sts_error)}")
            else:
                print(f"[--] ID {motor_id:3d} offline")

        print("-" * 50)
        if online_motors:
            print(f"[OK] Found {len(online_motors)} online motor(s): {online_motors}")
        else:
            print("[WARN] No online motors found.")

        return online_motors
    finally:
        port_handler.closePort()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Scan online ST3215 / STS motor IDs on the serial bus.")
    parser.add_argument("--port", default="COM5", help='Serial port, for example "COM5" or "/dev/ttyUSB0".')
    parser.add_argument("--baudrate", type=int, default=1_000_000, help="Bus baudrate. Default: 1000000")
    parser.add_argument("--start-id", type=int, default=1, help="Scan start ID. Default: 1")
    parser.add_argument("--end-id", type=int, default=20, help="Scan end ID. Default: 20")
    return parser.parse_args()


def main():
    args = parse_args()
    online_motors = scan_motors(
        serial_port=args.port,
        baudrate=args.baudrate,
        start_id=args.start_id,
        end_id=args.end_id,
    )
    raise SystemExit(0 if online_motors else 1)


if __name__ == "__main__":
    main()
