import time
from dataclasses import dataclass

from multiaxis_driver.imu import IMUDriverSerial, IMUDriverUSB
from multiaxis_driver.motor import MotorConfig, MultiAxisSerial, MultiAxisUdp, MultiAxisUSB


@dataclass
class BalanceConfig:
    target_pitch_deg: float = 0.0
    auto_zero_target: bool = True
    pitch_axis: int = 0
    pitch_sign: float = -1.0
    kp: float = 38.0
    kd: float = 0.85
    ki: float = 0.0
    max_speed: int = 1500
    speed_deadband: int = 20
    control_dt: float = 0.02
    pitch_fall_limit_deg: float = 35.0
    left_trim: int = 0
    right_trim: int = 0
    motor_acc: int = 0


class ControllerBalance2W:
    def __init__(
        self,
        motor_mode="serial",
        motor_port="COM8",
        imu_mode="serial",
        imu_port="COM8",
        config: BalanceConfig | None = None,
    ):
        if motor_mode == "usb":
            self.ctrl = MultiAxisUSB(motor_port)
        elif motor_mode == "serial":
            self.ctrl = MultiAxisSerial(motor_port)
        elif motor_mode == "udp":
            self.ctrl = MultiAxisUdp()
        else:
            raise ValueError("Unknown motor mode")

        if imu_mode == "usb":
            self.imu = IMUDriverUSB(port=imu_port)
        elif imu_mode == "serial":
            self.imu = IMUDriverSerial(port=imu_port)
        else:
            raise ValueError("Unknown imu mode")

        self.ctrl.motors_list = [
            MotorConfig(id=1, min=0, max=4095, init=2048, reverse=False),
            MotorConfig(id=2, min=0, max=4095, init=2048, reverse=True),
        ]

        self.config = config or BalanceConfig()
        self._pitch_integral = 0.0
        self._last_update = None

    def _read_imu_state(self):
        if hasattr(self.imu, "refresh"):
            self.imu.refresh()
        acc, gyro, angle = self.imu.get_data()
        axis = max(0, min(2, int(self.config.pitch_axis)))
        sign = 1.0 if self.config.pitch_sign >= 0 else -1.0
        pitch_deg = angle[axis] * sign
        pitch_rate_dps = gyro[axis] * sign
        return acc, gyro, angle, pitch_deg, pitch_rate_dps

    def hardware_init(self) -> bool:
        if not self.ctrl.hardware_init():
            return False
        if not self.imu.hardware_init():
            return False
        return True

    def online_check(self) -> bool:
        return self.ctrl.online_check()

    def wait_for_imu(self, timeout_s: float = 2.0) -> bool:
        return self.imu.wait_for_data(timeout_s)

    def enable_balance_mode(self) -> bool:
        success = True
        for motor in self.ctrl.motors_list:
            success = self.ctrl.set_wheel_mode(motor.id) and success
            success = self.ctrl.set_motor_speed(motor.id, 0, self.config.motor_acc) and success
        if self.config.auto_zero_target:
            _, _, _, pitch_deg, _ = self._read_imu_state()
            self.config.target_pitch_deg = pitch_deg
        self._pitch_integral = 0.0
        self._last_update = time.time()
        return success

    def stop(self) -> None:
        for motor in self.ctrl.motors_list:
            self.ctrl.set_motor_speed(motor.id, 0, self.config.motor_acc)

    def close(self) -> None:
        self.stop()
        self.imu.close()

    def get_state(self) -> dict:
        acc, gyro, angle, pitch_deg, pitch_rate_dps = self._read_imu_state()
        speeds = self.ctrl.get_all_speed()
        positions = self.ctrl.get_all_position()
        return {
            "acc": acc,
            "gyro": gyro,
            "angle": angle,
            "pitch_deg": pitch_deg,
            "pitch_rate_dps": pitch_rate_dps,
            "speeds": speeds,
            "positions": positions,
        }

    def _compute_speed_cmd(self, pitch_deg: float, pitch_rate_dps: float, dt: float) -> int:
        error = self.config.target_pitch_deg - pitch_deg
        self._pitch_integral += error * dt

        cmd = (
            self.config.kp * error
            - self.config.kd * pitch_rate_dps
            + self.config.ki * self._pitch_integral
        )
        cmd = int(max(-self.config.max_speed, min(self.config.max_speed, cmd)))
        if abs(cmd) < self.config.speed_deadband:
            cmd = 0
        return cmd

    def update_balance(self, turn_cmd: int = 0) -> dict:
        now = time.time()
        if self._last_update is None:
            self._last_update = now
        dt = max(1e-3, now - self._last_update)
        self._last_update = now

        acc, gyro, angle, pitch_deg, pitch_rate_dps = self._read_imu_state()

        if abs(pitch_deg) > self.config.pitch_fall_limit_deg:
            self.stop()
            return {
                "ok": False,
                "reason": "fall_limit",
                "pitch_deg": pitch_deg,
                "pitch_rate_dps": pitch_rate_dps,
            }

        base_cmd = self._compute_speed_cmd(pitch_deg, pitch_rate_dps, dt)
        left_cmd = base_cmd - int(turn_cmd) + self.config.left_trim
        right_cmd = base_cmd + int(turn_cmd) + self.config.right_trim

        left_motor = self.ctrl.motors_list[0]
        right_motor = self.ctrl.motors_list[1]
        if left_motor.reverse:
            left_cmd = -left_cmd
        if right_motor.reverse:
            right_cmd = -right_cmd

        ok_left = self.ctrl.set_motor_speed(left_motor.id, left_cmd, self.config.motor_acc)
        ok_right = self.ctrl.set_motor_speed(right_motor.id, right_cmd, self.config.motor_acc)

        return {
            "ok": ok_left and ok_right,
            "pitch_deg": pitch_deg,
            "pitch_rate_dps": pitch_rate_dps,
            "left_cmd": left_cmd,
            "right_cmd": right_cmd,
            "acc": acc,
            "gyro": gyro,
            "angle": angle,
        }
