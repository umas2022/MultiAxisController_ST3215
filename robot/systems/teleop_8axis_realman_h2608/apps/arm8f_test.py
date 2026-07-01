import time

from robot.systems.teleop_8axis_realman_h2608.controller import ControllerArm

def main():
    # arm = ControllerArm(mode="usb", serial_port="COM8")
    arm = ControllerArm(mode="usb", serial_port="/dev/ttyACM0")
    arm.hardware_init()

    print("online_check")
    while not arm.online_check():
        print("not online")
        time.sleep(1)
    print("online")

    while True:
        print(arm.get_all_position())
        time.sleep(1)


if __name__ == "__main__":
    main()
