#!/usr/bin/env python3
"""
Change the ID of an ST3215 / STS servo over usb, ESP32 serial, or UDP.
"""

import argparse
from typing import List, Optional

from motor_toolbox_common import add_connection_args, create_motor_driver
from robot.src.drivers.motor_driver.STservo_sdk import MAX_ID


def scan_online_motors(driver, start_id: int, end_id: int) -> List[int]:
    return [current_id for current_id in range(start_id, end_id + 1) if driver.ping_motor(current_id)]


def resolve_old_id(driver, old_id: Optional[int], scan_start: int, scan_end: int) -> Optional[int]:
    if old_id is not None:
        if not (0 <= old_id <= MAX_ID):
            print(f"[ERROR] Invalid old ID: {old_id}. Valid range is 0-{MAX_ID}.")
            return None
        if driver.ping_motor(old_id):
            return old_id
        print(f"[WARN] Motor with old ID {old_id} did not respond.")

    online_motors = scan_online_motors(driver, scan_start, scan_end)
    if not online_motors:
        print(
            f"[ERROR] No online motors found in ID range {scan_start}-{scan_end}. "
            "Check power, wiring, connection mode, and whether another program is occupying the link."
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


def change_motor_id(driver, old_id: Optional[int], new_id: int, scan_start: int = 1, scan_end: int = 20) -> bool:
    if not (0 <= new_id <= MAX_ID):
        print(f"[ERROR] Invalid new ID: {new_id}. Valid range is 0-{MAX_ID}.")
        return False

    current_id = resolve_old_id(driver, old_id, scan_start, scan_end)
    if current_id is None:
        return False

    if current_id == new_id:
        print("[ERROR] Old ID and new ID are identical. Nothing to change.")
        return False

    if not driver.set_motor_id(current_id, new_id):
        print(f"[ERROR] Failed to change motor ID: {current_id} -> {new_id}")
        return False

    if driver.ping_motor(current_id):
        print(f"[WARN] Old ID {current_id} still responds. Check whether multiple motors are on the bus.")

    print(f"[OK] Motor ID changed successfully: {current_id} -> {new_id}")
    return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Change the ID of an ST3215 / STS servo.")
    add_connection_args(parser)
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
    driver = create_motor_driver(args)
    success = change_motor_id(driver, old_id=args.old_id, new_id=args.new_id, scan_start=args.scan_start, scan_end=args.scan_end)
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
