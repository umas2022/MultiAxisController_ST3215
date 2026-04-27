#!/usr/bin/env python3
"""
Scan ST3215 / STS servo IDs over usb, ESP32 serial/Bluetooth serial, or UDP.
"""

import argparse
from typing import List

from motor_toolbox_common import add_connection_args, create_motor_driver
from robot.src.drivers.motor_driver.STservo_sdk import MAX_ID


def scan_motors(driver, start_id: int = 1, end_id: int = 20) -> List[int]:
    if not (0 <= start_id <= MAX_ID and 0 <= end_id <= MAX_ID):
        print(f"[ERROR] Invalid scan range {start_id}-{end_id}. Valid range is 0-{MAX_ID}.")
        return []
    if start_id > end_id:
        print(f"[ERROR] Invalid scan range {start_id}-{end_id}. start_id must be <= end_id.")
        return []

    online_motors: List[int] = []
    print(f"[INFO] Scanning motor IDs {start_id}-{end_id}")
    print("-" * 50)

    for motor_id in range(start_id, end_id + 1):
        if driver.ping_motor(motor_id):
            online_motors.append(motor_id)
            print(f"[OK] ID {motor_id:3d} online")
        else:
            print(f"[--] ID {motor_id:3d} offline")

    print("-" * 50)
    if online_motors:
        print(f"[OK] Found {len(online_motors)} online motor(s): {online_motors}")
    else:
        print("[WARN] No online motors found.")
    return online_motors


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Scan online ST3215 / STS motor IDs.")
    add_connection_args(parser)
    parser.add_argument("--start-id", type=int, default=1, help="Scan start ID. Default: 1")
    parser.add_argument("--end-id", type=int, default=20, help="Scan end ID. Default: 20")
    return parser.parse_args()


def main():
    args = parse_args()
    driver = create_motor_driver(args)
    online_motors = scan_motors(driver, start_id=args.start_id, end_id=args.end_id)
    raise SystemExit(0 if online_motors else 1)


if __name__ == "__main__":
    main()
