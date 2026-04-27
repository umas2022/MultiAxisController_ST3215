#!/usr/bin/env python3
"""
Set the current ST3215 / STS servo position as the center position (2048).
"""

import argparse
from typing import List, Optional

from motor_toolbox_common import add_connection_args, create_motor_driver
from robot.src.drivers.motor_driver.STservo_sdk import MAX_ID


def scan_online_motors(driver, start_id: int, end_id: int) -> List[int]:
    return [current_id for current_id in range(start_id, end_id + 1) if driver.ping_motor(current_id)]


def resolve_target_id(driver, motor_id: Optional[int], scan_start: int, scan_end: int) -> Optional[int]:
    if motor_id is not None:
        if not (0 <= motor_id <= MAX_ID):
            print(f"[ERROR] Invalid motor ID: {motor_id}. Valid range is 0-{MAX_ID}.")
            return None
        if driver.ping_motor(motor_id):
            return motor_id
        print(f"[WARN] Motor ID {motor_id} did not respond.")

    online_motors = scan_online_motors(driver, scan_start, scan_end)
    if not online_motors:
        print(
            f"[ERROR] No online motors found in ID range {scan_start}-{scan_end}. "
            "Check power, wiring, connection mode, and whether another program is occupying the link."
        )
        return None

    if motor_id is None and len(online_motors) == 1:
        detected_id = online_motors[0]
        print(f"[INFO] Auto-detected motor ID: {detected_id}")
        return detected_id

    print(f"[INFO] Online motor IDs: {online_motors}")
    if motor_id is not None:
        print("[ERROR] The requested motor ID is not online.")
    else:
        print("[ERROR] Multiple motors are online. Specify one with --id.")
    return None


def set_current_position_to_2048(driver, motor_id: Optional[int], scan_start: int = 1, scan_end: int = 20) -> bool:
    target_id = resolve_target_id(driver, motor_id, scan_start, scan_end)
    if target_id is None:
        return False

    position_before = driver.get_motor_position(target_id)
    if position_before is None:
        print("[ERROR] Failed to read current position.")
        return False

    if not driver.set_motor_zero(target_id):
        print("[ERROR] Failed to trigger center calibration.")
        return False

    position_after = driver.get_motor_position(target_id)
    if position_after is None:
        print("[WARN] Calibration command sent, but verification read failed.")
        print(f"[INFO] Position before calibration: {position_before}")
        return True

    print(
        f"[OK] Center calibration command sent successfully to ID {target_id}. "
        f"Position: {position_before} -> {position_after}"
    )
    return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Set the current ST3215 / STS servo position as center (2048).")
    add_connection_args(parser)
    parser.add_argument(
        "--id",
        dest="motor_id",
        type=int,
        default=None,
        help="Motor ID to calibrate. If omitted, the script auto-detects a single online motor.",
    )
    parser.add_argument("--scan-start", type=int, default=1, help="Start ID used for online scan. Default: 1")
    parser.add_argument("--scan-end", type=int, default=20, help="End ID used for online scan. Default: 20")
    return parser.parse_args()


def main():
    args = parse_args()
    driver = create_motor_driver(args)
    success = set_current_position_to_2048(driver, motor_id=args.motor_id, scan_start=args.scan_start, scan_end=args.scan_end)
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
