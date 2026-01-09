#!/usr/bin/env python3
"""
发布摇杆 5 个电机位置到 ROS2
"""

import time
import sys
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

sys.path.append(os.path.join(os.path.dirname(__file__), "..", ".."))
from src.controller.ControllerJoyStick5F import ControllerJoyStick


class JoyStickPublisher(Node):

    def __init__(self):
        super().__init__('joystick_publisher')

        # 创建 publisher
        self.pub = self.create_publisher(
            Int32MultiArray,
            'joystick/motor_position',
            10
        )

        # 初始化手柄
        self.joy_stick = ControllerJoyStick(
            mode="usb",
            serial_port="/dev/ttyACM0"
        )
        self.joy_stick.hardware_init()

        # 在线检查
        self.get_logger().info("Checking joystick online...")
        while rclpy.ok():
            if self.joy_stick.online_check():
                break
            self.get_logger().warn("Joystick not online, retrying...")
            time.sleep(1)

        self.get_logger().info("Joystick online ✔")

        # 定时器：100 Hz（可自行改）
        self.timer = self.create_timer(
            0.01,   # seconds
            self.timer_callback
        )

    def timer_callback(self):
        try:
            js_pos = self.joy_stick.get_all_position()  # [int x5]

            msg = Int32MultiArray()
            msg.data = js_pos

            self.pub.publish(msg)

            # self.get_logger().info(
            #     f"Published joystick positions: {js_pos}"
            # )

        except Exception as e:
            self.get_logger().error(f"Joystick read failed: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = JoyStickPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
