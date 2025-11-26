import sys
import time
import numpy as np
from abc import ABC, abstractmethod
from dataclasses import dataclass
import threading
import serial
from queue import Queue, Empty
import socket

from src.drivers.motor_driver.MultiAxisController import MultiAxisControllerInterface, MotorConfig
from src.drivers.motor_driver.STservo_sdk import * 

# 　指令id
CMD_PING = 0x01
CMD_SET_ID = 0x02
CMD_SET_ZERO = 0x03
CMD_MOVE_MAX_SPD = 0x04
CMD_MOVE_SPD_ACC = 0x05
CMD_GET_POS = 0x06
CMD_GET_LOAD = 0x07
CMD_GET_TEMP = 0x08
CMD_GET_POS_ALL = 0x09
CMD_GET_LOAD_ALL = 0x10
CMD_GET_TEMP_ALL = 0x11
CMD_GET_POS_LOAD_TEMP_ALL = 0x12

# global
RETRY_DELAY = 0.01  # 重试延时
RETRY_TIMES = 5  # 重试次数



class MultiAxisSerial(MultiAxisControllerInterface):
    """
    多轴控制器串口接口
    """

    def __init__(self, serial_port):
        self.serial_port = serial_port
        self.baudrate = 115200
        self.port_handler = None
        self.motors_list: list[MotorConfig] = []

        # 串口异步通信，线程在hardware_init之后启动
        self.read_queue = Queue()
        self.running = True
        self.read_thread = threading.Thread(target=self.read_loop)
        self.read_thread.daemon = True

    def read_loop(self):
        """
        异步读取串口数据
        """
        while self.running:
            if self.port_handler.in_waiting:
                data = self.port_handler.read(self.port_handler.in_waiting)
                self.read_queue.put(data)

    def hardware_init(self) -> bool:
        try:
            self.port_handler = serial.Serial(self.serial_port, baudrate=self.baudrate, timeout=0.1)
            print(f"已连接到 {self.serial_port}")
            self.read_thread.start()
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False

    def send_byte_command(self, byte_command):
        """
        发送字节命令到下位机
        """
        if not self.port_handler or not self.port_handler.is_open:
            print("串口未连接，无法发送指令！")
            return
        try:
            checksum = sum(byte_command) & 0xFF
            frame = bytearray([0xAA] + byte_command + [checksum])
            self.port_handler.write(frame)
        except Exception as e:
            print(f"发送失败: {e}")

    def get_response(self, cmd: int, timeout: float = 0.5):
        """
        从队列中获取匹配指定 cmd 的响应
        :param cmd: 期望的命令码（下位机返回的 cmd 通常是请求 cmd | 0x80）
        :param timeout: 最大等待时间（秒）
        :return: 匹配的消息 bytes 或 None
        """
        import time

        start_time = time.time()
        temp_queue = []

        while time.time() - start_time < timeout:
            try:
                # 尝试获取队列中的一条消息（不阻塞）
                response = self.read_queue.get_nowait()

                # 检查格式是否正确，以及 cmd 是否匹配
                if len(response) >= 3 and response[1] == (cmd | 0x80):
                    # 将暂存的其他消息重新放回队列
                    for item in temp_queue:
                        self.read_queue.put(item)
                    return response

                # 不符合就暂存
                temp_queue.append(response)

            except Exception:
                # 如果队列空了，稍等再试
                time.sleep(0.01)

        # 超时，恢复所有暂存的消息
        for item in temp_queue:
            self.read_queue.put(item)

        return None

    def online_check(self) -> bool:
        offline_list = []
        for each_motor in self.motors_list:
            self.send_byte_command([CMD_PING, 0x01, int(each_motor.id)])
            response = self.get_response(CMD_PING)
            if response is None:
                print(f"serial response error")
                continue
            if response[3] != 0x01:
                offline_list.append(each_motor.id)
        if len(offline_list) == 0:
            return True
        else:
            print(f"Offline motors: {offline_list}")
            return False

    def move_to_absolute(self, motor: MotorConfig, position: int, speed: int, acc: int) -> bool:
        range_check = True
        target_pos = np.clip(position, motor.min, motor.max)
        if target_pos != position:
            print(f"Target position {position} is out of range for motor {motor.id}.")
            range_check = False
        step_low = int(position) & 0xFF
        step_high = (int(position) >> 8) & 0xFF
        speed_low = int(speed) & 0xFF
        speed_high = (int(speed) >> 8) & 0xFF
        self.send_byte_command([CMD_MOVE_SPD_ACC, 0x06, motor.id, step_high, step_low, speed_high, speed_low, acc])
        return range_check

    def move_to_offset(self, motor: MotorConfig, offset: int, speed: int, acc: int) -> bool:
        if motor.reverse:
            offset = -offset
        return self.move_to_absolute(motor, motor.init + offset, speed, acc)

    def move_excute(self) -> None:
        pass

    def get_all_position(self):
        pos_list = []
        motor_count = len(self.motors_list)
        id_list = [int(each_motor.id) for each_motor in self.motors_list]
        self.send_byte_command([CMD_GET_POS_ALL, motor_count] + id_list)
        response = self.get_response(CMD_GET_POS_ALL)
        for i in range(motor_count):
            try:
                pos_high = response[3 + i * 2]
                pos_low = response[4 + i * 2]
                pos = (pos_high << 8) + pos_low
                pos_list.append(pos)
            except Exception as e:
                print(f"Motor {self.motors_list[i].id} response error: {e}")
                pos_list.append(0)
        return pos_list

    def get_all_load(self):
        load_list = []
        motor_count = len(self.motors_list)
        id_list = [int(each_motor.id) for each_motor in self.motors_list]
        self.send_byte_command([CMD_GET_LOAD_ALL, motor_count] + id_list)
        response = self.get_response(CMD_GET_LOAD_ALL)
        for i in range(motor_count):
            try:
                sign = response[3 + i * 3]
                load_high = response[4 + i * 3]
                load_low = response[5 + i * 3]
                load = (load_high << 8) + load_low
                if sign == 0x01:
                    load = -load
                load_list.append(load)
            except Exception as e:
                print(f"Motor {self.motors_list[i].id} response error: {e}")
                load_list.append(0)
        return load_list

    def get_all_temper(self):
        temper_list = []
        motor_count = len(self.motors_list)
        id_list = [int(each_motor.id) for each_motor in self.motors_list]
        self.send_byte_command([CMD_GET_TEMP_ALL, motor_count] + id_list)
        response = self.get_response(CMD_GET_TEMP_ALL)
        for i in range(motor_count):
            try:
                temper_high = response[3 + i * 2]
                temper_low = response[4 + i * 2]
                temper = (temper_high << 8) + temper_low
                temper_list.append(temper)
            except Exception as e:
                print(f"Motor {self.motors_list[i].id} response error: {e}")
                temper_list.append(0)
        return temper_list

    def get_all_position_load_temper(self) -> tuple:
        """
        获取所有电机位置、负载和温度
        :return: (positions, loads, tempers)
        注意由于esp32硬件FIFO限制，串口消息长度最大64（8个电机），超出8个电机时需要分批获取
        """
        if not self.motors_list:
            return ([], [], [])

        motor_count = len(self.motors_list)
        id_list = [int(motor.id) for motor in self.motors_list]
        cmd_list = [CMD_GET_POS_LOAD_TEMP_ALL, motor_count] + id_list
        print(f"Sending command: {cmd_list}")

        # 发送请求
        self.send_byte_command(cmd_list)

        # 获取响应
        response = self.get_response(CMD_GET_POS_LOAD_TEMP_ALL, timeout=0.5)
        if not response:
            print("获取数据超时")
            return ([0] * motor_count, [0] * motor_count, [0] * motor_count)

        # 检查数据长度 (帧头1 + CMD1 + 长度1 + 数据n + 校验和1)
        expected_len = 3 + motor_count * 7 + 1
        if len(response) != expected_len:
            print(f"数据长度错误，期望{expected_len}，实际{len(response)}")
            print("响应数据:", response)
            return ([0] * motor_count, [0] * motor_count, [0] * motor_count)

        pos_list = []
        load_list = []
        temper_list = []

        for i in range(motor_count):
            try:
                base = 3 + i * 7  # 跳过帧头(0xAA)、CMD和长度字节

                # 解析位置 (2字节)
                pos = (response[base] << 8) | response[base + 1]

                # 解析负载 (3字节: 符号1 + 值2)
                sign = response[base + 2]
                load = (response[base + 3] << 8) | response[base + 4]
                if sign == 0x01:
                    load = -load

                # 解析温度 (2字节)
                temper = (response[base + 5] << 8) | response[base + 6]

                pos_list.append(pos)
                load_list.append(load)
                temper_list.append(temper)

            except Exception as e:
                print(f"Motor {self.motors_list[i].id} 解析错误: {e}")
                pos_list.append(0)
                load_list.append(0)
                temper_list.append(0)
        return pos_list, load_list, temper_list

    def move_all_init(self, spd: int, acc: int) -> None:
        for each_motor in self.motors_list:
            self.move_to_absolute(each_motor, each_motor.init, spd, acc)
        self.move_excute()

    def move_all_absolute(self, pos_list: list, spd_list: list, acc_list: list) -> None:
        for i, each_motor in enumerate(self.motors_list):
            self.move_to_absolute(each_motor, pos_list[i], spd_list[i], acc_list[i])
        self.move_excute()

    def move_all_offset(self, pos_list: list, spd_list: list, acc_list: list) -> None:
        for i, each_motor in enumerate(self.motors_list):
            self.move_to_offset(each_motor, pos_list[i], spd_list[i], acc_list[i])
        self.move_excute()
