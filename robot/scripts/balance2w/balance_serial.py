import argparse
import pathlib
import sys
import time

ROBOT_ROOT = pathlib.Path(__file__).resolve().parents[2]
if str(ROBOT_ROOT) not in sys.path:
    sys.path.insert(0, str(ROBOT_ROOT))

from src.controller.ControllerBalance2W import BalanceConfig, ControllerBalance2W


def parse_args():
    parser = argparse.ArgumentParser(description="2-wheel balance test loop")
    parser.add_argument("--motor-mode", default="serial", choices=["usb", "serial", "udp"])
    parser.add_argument("--motor-port", default="COM8")
    parser.add_argument("--imu-mode", default="usb", choices=["usb", "serial"])
    parser.add_argument("--imu-port", default="COM3")
    parser.add_argument("--kp", type=float, default=38.0)
    parser.add_argument("--kd", type=float, default=0.85)
    parser.add_argument("--ki", type=float, default=0.0)
    parser.add_argument("--pitch-axis", type=int, default=0, choices=[0, 1, 2])
    parser.add_argument("--pitch-sign", type=float, default=-1.0)
    parser.add_argument("--target-pitch-deg", type=float, default=0.0)
    parser.add_argument("--disable-auto-zero-target", action="store_true")
    parser.add_argument("--max-speed", type=int, default=1500)
    parser.add_argument("--speed-deadband", type=int, default=20)
    parser.add_argument("--control-dt", type=float, default=0.02)
    parser.add_argument("--pitch-fall-limit-deg", type=float, default=35.0)
    parser.add_argument("--left-trim", type=int, default=0)
    parser.add_argument("--right-trim", type=int, default=0)
    parser.add_argument("--motor-acc", type=int, default=0)
    return parser.parse_args()


def main():
    args = parse_args()
    config = BalanceConfig(
        target_pitch_deg=args.target_pitch_deg,
        auto_zero_target=not args.disable_auto_zero_target,
        pitch_axis=args.pitch_axis,
        pitch_sign=args.pitch_sign,
        kp=args.kp,
        kd=args.kd,
        ki=args.ki,
        max_speed=args.max_speed,
        speed_deadband=args.speed_deadband,
        control_dt=args.control_dt,
        pitch_fall_limit_deg=args.pitch_fall_limit_deg,
        left_trim=args.left_trim,
        right_trim=args.right_trim,
        motor_acc=args.motor_acc,
    )

    robot = ControllerBalance2W(
        motor_mode=args.motor_mode,
        motor_port=args.motor_port,
        imu_mode=args.imu_mode,
        imu_port=args.imu_port,
        config=config,
    )

    if not robot.hardware_init():
        raise SystemExit("hardware_init failed")
    if not robot.online_check():
        raise SystemExit("motor online_check failed")
    if not robot.wait_for_imu(2.0):
        raise SystemExit("imu data not ready")
    if not robot.enable_balance_mode():
        raise SystemExit("failed to enable wheel mode")

    print(
        "balance loop start",
        {
            "target_pitch_deg": robot.config.target_pitch_deg,
            "pitch_axis": robot.config.pitch_axis,
            "pitch_sign": robot.config.pitch_sign,
        },
    )
    try:
        while True:
            state = robot.update_balance()
            print(state)
            if not state["ok"]:
                break
            time.sleep(robot.config.control_dt)
    finally:
        robot.close()


if __name__ == "__main__":
    main()
