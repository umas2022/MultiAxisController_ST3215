import sys
import time
import numpy as np
from abc import ABC, abstractmethod
from dataclasses import dataclass
import threading
import serial
from queue import Queue


sys.path.append("..")
from STservo_sdk import *


CMD_PING = 0x01
CMD_SET_ID = 0x02
CMD_SET_ZERO = 0x03
CMD_MOVE_MAX_SPD = 0x04
CMD_MOVE_SPD_ACC = 0x05
CMD_GET_POS = 0x06
CMD_GET_LOAD = 0x07
CMD_GET_TEMP = 0x08


@dataclass
class MotorConfig:
    id: int
    min: int
    max: int
    init: int
    reverse: bool


class MultiAxisControllerInterface(ABC):
    '''
    多轴控制器接口
    '''

    @abstractmethod
    def hardware_init(self) -> bool:
        """
        硬件初始化
        """
        pass

    @abstractmethod
    def online_check(self) -> bool:
        """
        检查所有电机在线
        """
        pass

    @abstractmethod
    def move_to_absolute(self, motor: MotorConfig, position: int, speed: int, acc: int) -> bool:
        """
        Move to absolute position
        """
        pass

    @abstractmethod
    def move_to_offset(self, motor: MotorConfig, offset: int, speed: int, acc: int) -> bool:
        """
        Move to offset position
        """
        pass

    @abstractmethod
    def move_excute(self) -> None:
        """
        Execute the move command
        """
        pass

    @abstractmethod
    def get_all_position(self) -> list:
        """
        获取所有电机位置
        """
        pass

    @abstractmethod
    def get_all_load(self) -> list:
        """
        获取所有电机负载
        """
        pass

    @abstractmethod
    def get_all_temper(self) -> list:
        """
        获取所有电机温度
        """
        pass

    @abstractmethod
    def move_all_init(self, spd, acc) -> None:
        """
        驱动所有电机到初始位置（中点）
        """
        pass

    @abstractmethod
    def move_all_absolute(self, pos_list: list, spd_list: list, acc_list: list) -> None:
        """
        驱动所有电机到给定位置（输入绝对位置）
        """
        pass

    @abstractmethod
    def move_all_offset(self, pos_list: list, spd_list: list, acc_list: list) -> None:
        """
        驱动所有电机到给定位置（输入相对于中点的偏移量）
        """
        pass


class MultiAxisUSB(MultiAxisControllerInterface):
    """
    多轴控制器USB接口
    """

    def __init__(self, serial_port):
        self.serial_port = serial_port
        self.baudrate = 1000000

        self.motors_list: list[MotorConfig] = []

        try:
            self.port_handler = PortHandler(self.serial_port)
        except Exception as e:
            print(f"Failed to open the port: {e}")
            exit(1)
        self.packet_handler = sts(self.port_handler)

    def hardware_init(self) -> bool:
        # Open port
        if self.port_handler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            return False
        # Set port baudrate
        if self.port_handler.setBaudRate(self.baudrate):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            return False
        return True

    def online_check(self) -> bool:
        offline_list = []
        for each_motor in self.motors_list:
            sts_model_number, sts_comm_result, sts_error = self.packet_handler.ping(each_motor.id)
            if sts_comm_result != COMM_SUCCESS:
                offline_list.append(each_motor.id)
            if sts_error != 0:
                print("%s" % self.packet_handler.getRxPacketError(sts_error))
        if len(offline_list) == 0:
            print("All motors are online")
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
        sts_comm_result, sts_error = self.packet_handler.RegWritePosEx(motor.id, target_pos, speed, acc)
        return range_check

    def move_to_offset(self, motor: MotorConfig, offset: int, speed: int, acc: int) -> bool:
        if motor.reverse:
            offset = -offset
        return self.move_to_absolute(motor, motor.init + offset, speed, acc)

    def move_excute(self) -> None:
        self.packet_handler.RegAction()

    def move_all_init(self, spd, acc) -> None:
        for each_motor in self.motors_list:
            self.move_to_absolute(each_motor, each_motor.init, spd, acc)
        self.move_excute()

    def get_all_position(self) -> list:
        pos_list = []
        for each_motor in self.motors_list:
            sts_present_position, sts_comm_result, sts_error = self.packet_handler.ReadPos(each_motor.id)
            if sts_comm_result != COMM_SUCCESS:
                print(f"Motor {each_motor.id} is offline")
                return []
            else:
                pos_list.append(sts_present_position)
            if sts_error != 0:
                print("%s" % self.packet_handler.getRxPacketError(sts_error))
                return []
        return pos_list

    def get_all_load(self) -> list:
        load_list = []
        for each_motor in self.motors_list:
            sts_present_load, sts_comm_result, sts_error = self.packet_handler.ReadLoad(each_motor.id)
            if sts_comm_result != COMM_SUCCESS:
                print(f"Motor {each_motor.id} is offline")
                return []
            else:
                load_list.append(sts_present_load)
            if sts_error != 0:
                print("%s" % self.packet_handler.getRxPacketError(sts_error))
                return []
        return load_list

    def get_all_temper(self) -> list:
        temper_list = []
        for each_motor in self.motors_list:
            sts_present_load, sts_comm_result, sts_error = self.packet_handler.ReadTemprature(each_motor.id)
            if sts_comm_result != COMM_SUCCESS:
                print(f"Motor {each_motor.id} is offline")
                return []
            else:
                temper_list.append(sts_present_load)
            if sts_error != 0:
                print("%s" % self.packet_handler.getRxPacketError(sts_error))
                return []
        return temper_list

    def move_all_absolute(self, pos_list: list, spd_list: list, acc_list: list) -> None:
        for i, each_motor in enumerate(self.motors_list):
            self.move_to_absolute(each_motor, pos_list[i], spd_list[i], acc_list[i])
        self.move_excute()

    def move_all_offset(self, pos_list: list, spd_list: list, acc_list: list) -> None:
        for i, each_motor in enumerate(self.motors_list):
            self.move_to_offset(each_motor, pos_list[i], spd_list[i], acc_list[i])
        self.move_excute()


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
            # print(f"发送指令: {[hex(b) for b in frame]}")
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
            response = self.get_response(CMD_PING | 0x80)
            if response is None:
                print(f"serial response error")
                continue
            if response[3] != 0x01:
                offline_list.append(each_motor.id)
        if len(offline_list) == 0:
            print("All motors online")
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
        pass


    def get_all_position(self) -> list:
        pos_list = []
        for each_motor in self.motors_list:
            self.send_byte_command([CMD_GET_POS, 0x01, int(each_motor.id)])
            response = self.get_response(CMD_GET_POS)
            try:
                pos_high = response[3]
                pos_low = response[4]
                pos = (pos_high << 8) + pos_low
                pos_list.append(pos)
            except Exception as e:
                print(f"Motor {each_motor.id} response error: {e}")
                pos_list.append(0)
                continue
        return pos_list

    def get_all_load(self) -> list:
        load_list = []
        for each_motor in self.motors_list:
            self.send_byte_command([CMD_GET_LOAD, 0x01, int(each_motor.id)])
            response = self.get_response(CMD_GET_LOAD)
            try:
                sign = response[3]
                load_high = response[4]
                load_low = response[5]
                load = (load_high << 8) + load_low
                if sign == 0x01:
                    load = -load
                load_list.append(load)
            except Exception as e:
                print(f"Motor {each_motor.id} response error: {e}")
                load_list.append(0)
                continue
        return load_list


    def get_all_temper(self) -> list:
        temper_list = []
        for each_motor in self.motors_list:
            self.send_byte_command([CMD_GET_TEMP, 0x01, int(each_motor.id)])
            response = self.get_response(CMD_GET_TEMP)
            try:
                temper_high = response[3]
                temper_low = response[4]
                pos = (temper_high << 8) + temper_low
                temper_list.append(pos)
            except Exception as e:
                print(f"Motor {each_motor.id} response error: {e}")
                temper_list.append(0)
                continue
        return temper_list

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
