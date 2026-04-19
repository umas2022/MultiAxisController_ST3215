#!/usr/bin/env python3
"""
Utility script to read the current position of an ST3215 / STS servo.

This version is defensive:
- it can scan the bus to discover online IDs
- it validates the target ID with `ping()` before reading
- it retries the read a few times and prints readable diagnostics
"""

import argparse
import os
import sys
import time
from typing import List, Optional

# Add project root to import path (same pattern as other toolbox scripts).
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(project_root)

from robot.src.drivers.motor_driver.STservo_sdk import COMM_SUCCESS, MAX_ID, PortHandler, sts


def scan_online_motors(packet_handler: sts, start_id: int = 1, end_id: int = 20) -> List[int]:
    """Return all motor IDs that respond to ping in the given range."""
    online_motors: List[int] = []
    for current_id in range(start_id, end_id + 1):
        _, comm_result, _ = packet_handler.ping(current_id)
        if comm_result == COMM_SUCCESS:
            online_motors.append(current_id)
    return online_motors


def resolve_target_id(
    packet_handler: sts,
    requested_id: Optional[int],
    scan_start: int,
    scan_end: int,
) -> Optional[int]:
    """Resolve which motor ID should be used for the position read."""
    if requested_id is not None:
        if not (0 <= requested_id <= MAX_ID):
            print(f"[ERROR] Invalid motor ID: {requested_id}. Valid range is 0-{MAX_ID}.")
            return None

        _, comm_result, sts_error = packet_handler.ping(requested_id)
        if comm_result == COMM_SUCCESS:
            if sts_error != 0:
                print(f"[WARN] Ping on ID {requested_id} returned servo error: {packet_handler.getRxPacketError(sts_error)}")
            return requested_id

        print(
            f"[WARN] Motor ID {requested_id} did not respond: "
            f"{packet_handler.getTxRxResult(comm_result)}"
        )

    online_motors = scan_online_motors(packet_handler, scan_start, scan_end)
    if not online_motors:
        print(
            f"[ERROR] No online motors found in ID range {scan_start}-{scan_end}. "
            "Check power, wiring, COM port, baudrate, and whether another program is occupying the port."
        )
        return None

    if requested_id is None and len(online_motors) == 1:
        detected_id = online_motors[0]
        print(f"[INFO] Auto-detected motor ID: {detected_id}")
        return detected_id

    print(f"[INFO] Online motor IDs: {online_motors}")

    if requested_id is not None:
        print("[ERROR] The requested motor ID is not online.")
    else:
        print("[ERROR] Multiple motors are online. Specify one with --id.")

    return None


def read_motor_position(
    serial_port: str,
    motor_id: Optional[int],
    baudrate: int = 1_000_000,
    retries: int = 3,
    scan_start: int = 1,
    scan_end: int = 20,
) -> Optional[int]:
    """Open the port, resolve a target motor ID, query its position, and close the port."""
    try:
        port_handler = PortHandler(serial_port)
    except Exception as exc:
        print(f"[ERROR] Failed to init PortHandler: {exc}")
        return None

    if not port_handler.openPort():
        print(f"[ERROR] Could not open port: {serial_port}")
        return None

    if not port_handler.setBaudRate(baudrate):
        print(f"[ERROR] Could not set baudrate: {baudrate}")
        port_handler.closePort()
        return None

    packet_handler = sts(port_handler)

    try:
        target_id = resolve_target_id(packet_handler, motor_id, scan_start, scan_end)
        if target_id is None:
            return None

        last_comm_result = None
        last_sts_error = 0

        for attempt in range(1, retries + 1):
            position, comm_result, sts_error = packet_handler.ReadPos(target_id)
            if comm_result == COMM_SUCCESS:
                if sts_error != 0:
                    print(f"[WARN] Position read finished with servo error: {packet_handler.getRxPacketError(sts_error)}")
                print(f"Motor ID {target_id} current raw position: {position}")
                return position

            last_comm_result = comm_result
            last_sts_error = sts_error
            print(
                f"[WARN] Read attempt {attempt}/{retries} failed: "
                f"{packet_handler.getTxRxResult(comm_result)}"
            )
            time.sleep(0.05)

        print(
            f"[ERROR] Failed to read position from motor ID {target_id}: "
            f"{packet_handler.getTxRxResult(last_comm_result)}"
        )
        if last_sts_error:
            print(f"[ERROR] Last servo error: {packet_handler.getRxPacketError(last_sts_error)}")
        return None
    finally:
        port_handler.closePort()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read the current raw position of an ST3215 / STS servo.")
    parser.add_argument("--port", default="COM5", help='Serial port, for example "COM5" or "/dev/ttyUSB0".')
    parser.add_argument("--baudrate", type=int, default=1_000_000, help="Bus baudrate. Default: 1000000")
    parser.add_argument(
        "--id",
        dest="motor_id",
        type=int,
        default=None,
        help="Motor ID to read. If omitted, the script auto-detects a single online motor.",
    )
    parser.add_argument("--retries", type=int, default=3, help="Number of read retries. Default: 3")
    parser.add_argument("--scan-start", type=int, default=1, help="Start ID used for online scan. Default: 1")
    parser.add_argument("--scan-end", type=int, default=20, help="End ID used for online scan. Default: 20")
    return parser.parse_args()


def main():
    args = parse_args()
    position = read_motor_position(
        serial_port=args.port,
        motor_id=args.motor_id,
        baudrate=args.baudrate,
        retries=args.retries,
        scan_start=args.scan_start,
        scan_end=args.scan_end,
    )
    raise SystemExit(0 if position is not None else 1)


if __name__ == "__main__":
    main()
