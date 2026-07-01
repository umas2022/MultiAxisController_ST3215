#!/usr/bin/env python3
"""
ELRS serial resolver based on the reusable ELRSDriver.

This toolbox script is intended for manual bench testing:
- open an ELRS receiver serial port
- decode CRSF RC frames through `ELRSDriver`
- periodically print channel values and selected normalized axes
"""

import argparse
import os
import sys
import time

# Add the standalone driver package to the import path for repository-local use.
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
drivers_src = os.path.join(project_root, "drivers", "src")
sys.path.append(drivers_src)

from multiaxis_driver.elrs import ELRSDriver


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Resolve and print ELRS / CRSF channel data from a serial port.")
    parser.add_argument("--port", default="COM3", help='Serial port, for example "COM3" or "/dev/ttyUSB0".')
    parser.add_argument("--baudrate", type=int, default=420000, help="ELRS receiver baudrate. Default: 420000")
    parser.add_argument("--timeout", type=float, default=0.02, help="Serial timeout in seconds. Default: 0.02")
    parser.add_argument("--print-interval", type=float, default=1.0, help="Console print interval in seconds. Default: 1.0")
    parser.add_argument("--max-age", type=float, default=0.5, help="Maximum accepted age for the latest frame. Default: 0.5")
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    driver = ELRSDriver(
        serial_port=args.port,
        baudrate=args.baudrate,
        timeout=args.timeout,
    )

    if not driver.hardware_init():
        raise SystemExit(1)

    print(f"[INFO] Listening for ELRS data on {args.port} @ {args.baudrate}")
    print("[INFO] Press Ctrl+C to stop.")

    last_print_time = 0.0

    try:
        while True:
            driver.poll()

            now = time.time()
            if now - last_print_time < args.print_interval:
                time.sleep(0.002)
                continue

            channels = driver.get_channels(max_age=args.max_age)
            if channels is None:
                age = driver.get_frame_age()
                if age is None:
                    print("[WAIT] No valid ELRS frame received yet.")
                else:
                    print(f"[WAIT] Latest ELRS frame is stale: age={age:.3f}s")
                last_print_time = now
                continue

            axis_roll = driver.get_axis(0, max_age=args.max_age)
            axis_pitch = driver.get_axis(1, max_age=args.max_age)
            axis_throttle = driver.get_axis(2, deadband=0.0, max_age=args.max_age)
            axis_yaw = driver.get_axis(3, max_age=args.max_age)

            print(f"Channels: {channels}")
            print(
                "[INFO] axes "
                f"roll={axis_roll:.3f} "
                f"pitch={axis_pitch:.3f} "
                f"throttle={axis_throttle:.3f} "
                f"yaw={axis_yaw:.3f}"
            )
            print(
                "[INFO] switches "
                f"ch5={driver.get_switch_state(4, max_age=args.max_age)} "
                f"ch6={driver.get_switch_state(5, max_age=args.max_age)} "
                f"age={driver.get_frame_age():.3f}s "
                f"frames={driver.frame_count}"
            )
            last_print_time = now

    except KeyboardInterrupt:
        print("[INFO] User interrupted, exiting.")
    finally:
        driver.close()


if __name__ == "__main__":
    main()
