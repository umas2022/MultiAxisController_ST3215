#!/usr/bin/env python3
"""
Set the current ST3215 / STS servo position as the center position (2048).

This mirrors the behavior used by the ESP32 lower controller in
MultiAxisHandler_ST3215:
    CalibrationOfs(motor_id) -> writeByte(ID, SMS_STS_TORQUE_ENABLE, 128)
"""

import argparse
import os
import sys
from typing import List, Optional

# Add project root to import path.
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(project_root)

from robot.src.drivers.motor_driver.STservo_sdk import COMM_SUCCESS, MAX_ID, PortHandler, sts
from robot.src.drivers.motor_driver.STservo_sdk.sts import STS_TORQUE_ENABLE

CALIBRATION_TRIGGER = 128


def scan_online_motors(packet_handler: sts, start_id: int, end_id: int) -> List[int]:
    """Return all motor IDs that respond to ping in the given range."""
    online_motors: List[int] = []
    for current_id in range(start_id, end_id + 1):
        _, comm_result, _ = packet_handler.ping(current_id)
        if comm_result == COMM_SUCCESS:
            online_motors.append(current_id)
    return online_motors


def resolve_target_id(
    packet_handler: sts,
    motor_id: Optional[int],
    scan_start: int,
    scan_end: int,
) -> Optional[int]:
    """Resolve which motor should receive the calibration command."""
    if motor_id is not None:
        if not (0 <= motor_id <= MAX_ID):
            print(f"[ERROR] Invalid motor ID: {motor_id}. Valid range is 0-{MAX_ID}.")
            return None

        _, comm_result, sts_error = packet_handler.ping(motor_id)
        if comm_result == COMM_SUCCESS:
            if sts_error != 0:
                print(f"[WARN] Ping on ID {motor_id} returned servo error: {packet_handler.getRxPacketError(sts_error)}")
            return motor_id

        print(
            f"[WARN] Motor ID {motor_id} did not respond: "
            f"{packet_handler.getTxRxResult(comm_result)}"
        )

    online_motors = scan_online_motors(packet_handler, scan_start, scan_end)
    if not online_motors:
        print(
            f"[ERROR] No online motors found in ID range {scan_start}-{scan_end}. "
            "Check power, wiring, COM port, baudrate, and whether another program is occupying the port."
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


def set_current_position_to_2048(
    serial_port: str,
    motor_id: Optional[int],
    baudrate: int = 1_000_000,
    scan_start: int = 1,
    scan_end: int = 20,
) -> bool:
    """Trigger the servo's built-in center calibration command."""
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
        target_id = resolve_target_id(packet_handler, motor_id, scan_start, scan_end)
        if target_id is None:
            return False

        position_before, comm_result, sts_error = packet_handler.ReadPos(target_id)
        if comm_result != COMM_SUCCESS:
            print(f"[ERROR] Failed to read current position: {packet_handler.getTxRxResult(comm_result)}")
            return False
        if sts_error != 0:
            print(f"[ERROR] Read current position returned servo error: {packet_handler.getRxPacketError(sts_error)}")
            return False

        comm_result, sts_error = packet_handler.write1ByteTxRx(
            target_id,
            STS_TORQUE_ENABLE,
            CALIBRATION_TRIGGER,
        )
        if comm_result != COMM_SUCCESS:
            print(f"[ERROR] Failed to trigger center calibration: {packet_handler.getTxRxResult(comm_result)}")
            return False
        if sts_error != 0:
            print(f"[ERROR] Center calibration returned servo error: {packet_handler.getRxPacketError(sts_error)}")
            return False

        position_after, comm_result, sts_error = packet_handler.ReadPos(target_id)
        if comm_result != COMM_SUCCESS:
            print(f"[WARN] Calibration command sent, but verification read failed: {packet_handler.getTxRxResult(comm_result)}")
            print(f"[INFO] Position before calibration: {position_before}")
            return True
        if sts_error != 0:
            print(f"[WARN] Verification read returned servo error: {packet_handler.getRxPacketError(sts_error)}")

        print(
            f"[OK] Center calibration command sent successfully to ID {target_id}. "
            f"Position: {position_before} -> {position_after}"
        )
        return True
    finally:
        port_handler.closePort()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Set the current ST3215 / STS servo position as center (2048).")
    parser.add_argument("--port", default="COM5", help='Serial port, for example "COM5" or "/dev/ttyUSB0".')
    parser.add_argument("--baudrate", type=int, default=1_000_000, help="Bus baudrate. Default: 1000000")
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
    success = set_current_position_to_2048(
        serial_port=args.port,
        motor_id=args.motor_id,
        baudrate=args.baudrate,
        scan_start=args.scan_start,
        scan_end=args.scan_end,
    )
    raise SystemExit(0 if success else 1)


if __name__ == "__main__":
    main()
