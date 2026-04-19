#!/usr/bin/env python3
"""
Legacy setup GUI entrypoint.

This tool is intentionally retired. Motor setup is now done directly from the
host toolbox against the servo bus instead of being proxied through the ESP32
lower controller.
"""

from __future__ import annotations

import sys


def main() -> int:
    print("setup_interface.py has been retired.")
    print("Use the direct host-side tools instead:")
    print("  - toolbox/motor_id_scann.py")
    print("  - toolbox/motor_set_id.py")
    print("  - toolbox/motor_set_2048.py")
    print("  - toolbox/motor_read_position.py")
    print("")
    print("The ESP32 firmware no longer exposes the legacy setup commands 0x02, 0x03, 0x04.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
