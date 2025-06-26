import sys
import time
import numpy as np
from abc import ABC, abstractmethod
from dataclasses import dataclass
import threading
import serial
from queue import Queue, Empty
import socket

from MultiAxisSystem.MultiAxisController import MultiAxisControllerInterface, MotorConfig

sys.path.append("..")
from STservo_sdk import *

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


class MultiAxisUdp(MultiAxisControllerInterface):
    """
    多轴控制器UDP连接
    """

    def __init__(self):
        self.local_ip = "0.0.0.0"
        self.esp32_ip = "192.168.4.1"
        self.esp32_port = 4210
        self.local_port = 4210
        self.sock = None
        self.motors_list: list[MotorConfig] = []

        # udp异步通信，线程在hardware_init之后启动
        self.read_queue = Queue()
        self.running = True
        self.read_thread = threading.Thread(target=self.read_loop)
        self.read_thread.daemon = True

    def hardware_init(self) -> bool:

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind((self.local_ip, self.local_port))
            print(f"UDP server started at {self.local_ip}:{self.local_port}")
            self.read_thread.start()
            return True
        except Exception as e:
            print(f"Failed to initialize UDP: {e}")
            return False

    def read_loop(self):
        """
        异步读取UDP数据
        """
        while self.running:
            data, addr = self.sock.recvfrom(1024)
            if data:
                self.read_queue.put(data)

    def send_byte_command(self, byte_command):
        """
        发送字节命令到下位机
        """
        if not self.sock:
            print("UDP socket未初始化，无法发送指令！")
            return
        try:
            checksum = sum(byte_command) & 0xFF
            frame = bytearray([0xAA] + byte_command + [checksum])
            self.sock.sendto(frame, (self.esp32_ip, self.esp32_port))
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

    def get_response_last(self):
        """
        接收来自下位机的最后一条返回消息
        """
        try:
            return self.read_queue.get(timeout=0.5)
        except:
            return None

    def online_check(self) -> bool:
        offline_list = []
        for each_motor in self.motors_list:
            self.send_byte_command([CMD_PING, 0x01, int(each_motor.id)])
            response = self.get_response(CMD_PING)
            if response is None:
                print(f"UDP response error for motor {each_motor.id}")
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
        # sts_comm_result, sts_error = self.packet_handler.RegWritePosEx(motor.id, target_pos, speed, acc)
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
        """
        执行移动命令
        """
        # UDP协议不需要额外的执行命令，直接发送命令即可
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
        获取所有电机位置、负载和温度（修正版）
        :return: (positions, loads, tempers)
        """
        if not self.motors_list:
            return ([], [], [])

        motor_count = len(self.motors_list)
        id_list = [int(motor.id) for motor in self.motors_list]

        # 发送请求
        self.send_byte_command([CMD_GET_POS_LOAD_TEMP_ALL, motor_count] + id_list)

        # 获取响应
        response = self.get_response(CMD_GET_POS_LOAD_TEMP_ALL, timeout=0.5)
        if not response:
            print("获取数据超时")
            return ([0] * motor_count, [0] * motor_count, [0] * motor_count)

        # 检查数据长度 (帧头1 + CMD1 + 长度1 + 数据n + 校验和1)
        expected_len = 3 + motor_count * 7 + 1
        if len(response) != expected_len:
            print(f"数据长度错误，期望{expected_len}，实际{len(response)}")
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

    def move_all_init(self, spd, acc) -> None:
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
