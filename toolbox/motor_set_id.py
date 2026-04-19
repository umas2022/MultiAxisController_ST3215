#!/usr/bin/env python3
"""
Utility script to change the ID of an ST3215 / STS servo.
"""

import argparse
import os
import sys
from typing import List, Optional

# Add project root to import path.
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(project_root)

from robot.src.drivers.motor_driver.STservo_sdk import COMM_SUCCESS, MAX_ID, PortHandler, sts
from robot.src.drivers.motor_driver.STservo_sdk.sts import STS_ID


def scan_online_motors(packet_handler: sts, start_id: int, end_id: int) -> List[int]:
    """Return all motor IDs that respond to ping in the given range."""
    online_motors: List[int] = []
    for current_id in range(start_id, end_id + 1):
        _, comm_result, _ = packet_handler.ping(current_id)
        if comm_result == COMM_SUCCESS:
            online_motors.append(current_id)
    return online_motors


def resolve_old_id(
    packet_handler: sts,
    old_id: Optional[int],
    scan_start: int,
    scan_end: int,
) -> Optional[int]:
    """Resolve which current motor ID should be changed."""
    if old_id is not None:
        if not (0 <= old_id <= MAX_ID):
            print(f"[ERROR] Invalid old ID: {old_id}. Valid range is 0-{MAX_ID}.")
            return None

        _, comm_result, sts_error = packet_handler.ping(old_id)
        if comm_result == COMM_SUCCESS:
            if sts_error != 0:
                print(f"[WARN] Ping on old ID {old_id} returned servo error: {packet_handler.getRxPacketError(sts_error)}")
            return old_id

        print(
            f"[WARN] Motor with old ID {old_id} did not respond: "
            f"{packet_handler.getTxRxResult(comm_result)}"
        )

    online_motors = scan_online_motors(packet_handler, scan_start, scan_end)
    if not online_motors:
        print(
            f"[ERROR] No online motors found in ID range {scan_start}-{scan_end}. "
            "Check power, wiring, COM port, baudrate, and whether another program is occupying the port."
        )
        return None

    if old_id is None and len(online_motors) == 1:
        detected_id = online_motors[0]
        print(f"[INFO] Auto-detected current motor ID: {detected_id}")
        return detected_id

    print(f"[INFO] Online motor IDs: {online_motors}")
    if old_id is not None:
        print("[ERROR] The requested old ID is not online.")
    else:
        print("[ERROR] Multiple motors are online. Specify the current ID with --old-id.")
    return None


def change_motor_id(
    serial_port: str,
    old_id: Optional[int],
    new_id: int,
    baudrate: int = 1_000_000,
    scan_start: int = 1,
    scan_end: int = 20,
) -> bool:
    """Change a servo ID and verify the result."""
    if not (0 <= new_id <= MAX_ID):
        print(f"[ERROR] Invalid new ID: {new_id}. Valid range is 0-{MAX_ID}.")
        return False

    try:
        port_handler = PortHandler(serial_port)
    except Exception as exc:
        print(f"[ERROR] Failed to init PortHandler: {exc}")
        return False

    if not port_handler.openPort():
        print(f"[ERROR] Could not open port: {serial_port}")
        return False

    if not port_handler.setBaudRate(baudrate):
        print(f"[ERROR] Could not set baudrate: {baudrate}")
        port_handler.closePort()
        return False

    packet_handler = sts(port_handler)

    try:
        current_id = resolve_old_id(packet_handler, old_id, scan_start, scan_end)
        if current_id is None:
            return False

        if current_id == new_id:
            print("[ERROR] Old ID and new ID are identical. Nothing to change.")
            return False

        comm_result, sts_error = packet_handler.unLockEprom(current_id)
        if comm_result != COMM_SUCCESS:
            print(f"[ERROR] Failed to unlock EEPROM: {packet_handler.getTxRxResult(comm_result)}")
            return False
        if sts_error != 0:
            print(f"[ERROR] Unlock EEPROM returned servo error: {packet_handler.getRxPacketError(sts_error)}")
            return False

        comm_result, sts_error = packet_handler.write1ByteTxRx(current_id, STS_ID, new_id)
        if comm_result != COMM_SUCCESS:
            print(f"[ERROR] Failed to write new ID: {packet_handler.getTxRxResult(comm_result)}")
            return False
        if sts_error != 0:
            print(f"[ERROR] Write new ID returned servo error: {packet_handler.getRxPacketError(sts_error)}")
            return False

        comm_result, sts_error = packet_handler.LockEprom(new_id)
        if comm_result != COMM_SUCCESS:
            print(f"[WARN] ID changed, but failed to lock EEPROM with new ID: {packet_handler.getTxRxResult(comm_result)}")
        elif sts_error != 0:
            print(f"[WARN] ID changed, but lock EEPROM returned servo error: {packet_handler.getRxPacketError(sts_error)}")

        _, comm_result, sts_error = packet_handler.ping(new_id)
        if comm_result != COMM_SUCCESS:
            print(
                f"[ERROR] Write completed, but new ID {new_id} did not respond: "
                f"{packet_handler.getTxRxResult(comm_result)}"
            )
            return False
        if sts_error != 0:
            print(f"[WARN] Ping on new ID returned servo error: {packet_handler.getRxPacketError(sts_error)}")

        _, comm_result, _ = packet_handler.ping(current_id)
        if comm_result == COMM_SUCCESS:
            print(f"[WARN] Old ID {current_id} still responds. Check whether multiple motors are on the bus.")

        print(f"[OK] Motor ID changed successfully: {current_id} -> {new_id}")
        return True
    finally:
        port_handler.closePort()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Change the ID of an ST3215 / STS servo.")
    parser.add_argument("--port", default="COM5", help='Serial port, for example "COM5" or "/dev/ttyUSB0".')
    parser.add_argument("--baudrate", type=int, default=1_000_000, help="Bus baudrate. Default: 1000000")
    parser.add_argument(
        "--old-id",
        type=int,
        default=None,
        help="Current motor ID. If omitted, the script auto-detects a single online motor.",
    )
    parser.add_argument("--new-id", type=int, required=True, help="Target motor ID.")
    parser.add_argument("--scan-start", type=int, default=1, help="Start ID used for online scan. Default: 1")
    parser.add_argument("--scan-end", type=int, default=20, help="End ID used for online scan. Default: 20")
    return parser.parse_args()


def main():
    args = parse_args()
    success = change_motor_id(
        serial_port=args.port,
        old_id=args.old_id,
        new_id=args.new_id,
        baudrate=args.baudrate,
        scan_start=args.scan_start,
        scan_end=args.scan_end,
    )
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
