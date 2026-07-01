"""Controller for the H3106 two-axis teleoperation joystick."""

from __future__ import annotations

import time

from multiaxis_driver.motor import MotorConfig, MultiAxisUSB


SERIAL_PORT = "COM16"
MOTOR_IDS = (1, 2)
RAW_CENTER = 2048
COUNTS_PER_TURN = 4096
MIN_ANGLE_DEG = -45
MIN_POSITION = RAW_CENTER + int(COUNTS_PER_TURN * MIN_ANGLE_DEG / 360)  # 1536
MOVE_SPEED = 500
MOVE_ACCELERATION = 50
POSITION_TOLERANCE = 20
INIT_TIMEOUT = 10.0
POLL_INTERVAL = 0.05


class Teleop2AxisController:
    """Initialize and read the two ST3215 axes in the H3106 controller."""

    def __init__(self) -> None:
        self._driver = MultiAxisUSB(SERIAL_PORT)
        self._driver.motors_list = [
            MotorConfig(id=motor_id, min=0, max=4095, init=MIN_POSITION, reverse=False)
            for motor_id in MOTOR_IDS
        ]
        self._initialized = False

    def init(self) -> bool:
        """Move both axes to -45 degrees, then release them for manual use."""

        self._initialized = False
        if not self._driver.hardware_init():
            return False
        if not self._driver.online_check():
            return False

        torque_enabled = []
        try:
            for motor_id in MOTOR_IDS:
                if not self._driver.set_motor_torque(motor_id, True):
                    return False
                torque_enabled.append(motor_id)

            self._driver.move_all_init(MOVE_SPEED, MOVE_ACCELERATION)
            if not self._wait_until_min_position():
                return False
        finally:
            released = True
            for motor_id in torque_enabled:
                if not self._driver.set_motor_torque(motor_id, False):
                    released = False

        self._initialized = released
        return self._initialized

    def read_positions(self) -> tuple[int, int]:
        """Return the raw encoder positions of axis 1 and axis 2."""

        if not self._initialized:
            raise RuntimeError("Controller is not initialized; call init() first")

        positions = self._driver.get_all_position()
        if len(positions) != 2:
            raise RuntimeError("Failed to read both H3106 motor positions")
        return int(positions[0]), int(positions[1])

    def _wait_until_min_position(self) -> bool:
        deadline = time.monotonic() + INIT_TIMEOUT
        while time.monotonic() < deadline:
            positions = self._driver.get_all_position()
            if len(positions) != 2:
                return False
            if all(abs(position - MIN_POSITION) <= POSITION_TOLERANCE for position in positions):
                return True
            time.sleep(POLL_INTERVAL)
        return False
