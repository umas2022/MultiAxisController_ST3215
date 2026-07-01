"""Initialize H3106 and continuously print its two raw motor positions."""

import sys
import time
from pathlib import Path


# Support both:
#   python -m robot.systems.teleop_2axis_h3106.apps.read_state
#   python read_state.py
PROJECT_ROOT = Path(__file__).resolve().parents[4]
DRIVERS_SRC = PROJECT_ROOT / "drivers" / "src"
for import_path in (PROJECT_ROOT, DRIVERS_SRC):
    if str(import_path) not in sys.path:
        sys.path.insert(0, str(import_path))

from robot.common import get_logger
from robot.systems.teleop_2axis_h3106 import Teleop2AxisController


READ_INTERVAL = 0.05


def main() -> int:
    logger = get_logger("teleop_2axis_h3106.read_state")
    controller = Teleop2AxisController()

    if not controller.init():
        logger.error("Failed to initialize H3106")
        return 1

    logger.info("H3106 initialized and released; press Ctrl+C to stop")
    try:
        while True:
            position_1, position_2 = controller.read_positions()
            print(f"positions=({position_1:4d}, {position_2:4d})")
            time.sleep(READ_INTERVAL)
    except KeyboardInterrupt:
        logger.info("Stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
