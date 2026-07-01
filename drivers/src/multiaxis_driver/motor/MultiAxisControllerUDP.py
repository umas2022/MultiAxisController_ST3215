import socket
import threading
import time
from queue import Empty, Queue

import numpy as np

from .MultiAxisController import MultiAxisControllerInterface, MotorConfig

CMD_PING = 0x01
CMD_SET_ID = 0x02
CMD_SET_ZERO = 0x03
CMD_MOVE_SPD_ACC = 0x05
CMD_GET_POS = 0x06
CMD_GET_LOAD = 0x07
CMD_GET_TEMP = 0x08
CMD_GET_POS_ALL = 0x09
CMD_GET_LOAD_ALL = 0x10
CMD_GET_TEMP_ALL = 0x11
CMD_GET_POS_LOAD_TEMP_ALL = 0x12
CMD_GET_SPEED_ALL = 0x13
CMD_SET_WHEEL_MODE = 0x14
CMD_SET_SPEED = 0x15


class MultiAxisUdp(MultiAxisControllerInterface):
    """
    Multi-axis controller over ESP32 UDP.
    """

    def __init__(self, esp32_ip="192.168.4.1", esp32_port=4210, local_ip="0.0.0.0", local_port=4210):
        self.local_ip = local_ip
        self.esp32_ip = esp32_ip
        self.esp32_port = esp32_port
        self.local_port = local_port
        self.sock = None
        self.motors_list: list[MotorConfig] = []

        self.read_queue = Queue()
        self.running = True
        self.read_thread = threading.Thread(target=self.read_loop, daemon=True)

    def hardware_init(self) -> bool:
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind((self.local_ip, self.local_port))
            self.sock.settimeout(0.1)
            self.read_thread.start()
            print(f"UDP server started at {self.local_ip}:{self.local_port}")
            return True
        except Exception as exc:
            print(f"Failed to initialize UDP: {exc}")
            return False

    def read_loop(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(1024)
                if data:
                    self.read_queue.put(data)
            except OSError:
                time.sleep(0.005)

    def send_byte_command(self, byte_command):
        if not self.sock:
            print("UDP socket未初始化，无法发送指令")
            return
        checksum = sum(byte_command) & 0xFF
        frame = bytearray([0xAA] + byte_command + [checksum])
        self.sock.sendto(frame, (self.esp32_ip, self.esp32_port))

    def get_response(self, cmd: int, timeout: float = 0.5):
        start_time = time.time()
        temp_queue = []

        while time.time() - start_time < timeout:
            try:
                response = self.read_queue.get_nowait()
                if len(response) >= 3 and response[1] == (cmd | 0x80):
                    for item in temp_queue:
                        self.read_queue.put(item)
                    return response
                temp_queue.append(response)
            except Empty:
                time.sleep(0.01)

        for item in temp_queue:
            self.read_queue.put(item)
        return None

    def _decode_signed_triplets(self, response, motor_count, value_name):
        values = []
        for index in range(motor_count):
            try:
                sign = response[3 + index * 3]
                value_high = response[4 + index * 3]
                value_low = response[5 + index * 3]
                value = (value_high << 8) + value_low
                if sign == 0x01:
                    value = -value
                values.append(value)
            except Exception as exc:
                motor_id = self.motors_list[index].id
                print(f"Motor {motor_id} {value_name} response error: {exc}")
                values.append(0)
        return values

    def ping_motor(self, motor_id: int) -> bool:
        self.send_byte_command([CMD_PING, 0x01, int(motor_id)])
        response = self.get_response(CMD_PING)
        if response is None or len(response) < 5:
            return False
        return response[3] == 0x01

    def online_check(self) -> bool:
        offline_list = [motor.id for motor in self.motors_list if not self.ping_motor(motor.id)]
        if not offline_list:
            return True
        print(f"Offline motors: {offline_list}")
        return False

    def move_to_absolute(self, motor: MotorConfig, position: int, speed: int, acc: int) -> bool:
        range_check = True
        target_pos = int(np.clip(position, motor.min, motor.max))
        if target_pos != position:
            print(f"Target position {position} is out of range for motor {motor.id}.")
            range_check = False
        step_low = target_pos & 0xFF
        step_high = (target_pos >> 8) & 0xFF
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

    def get_motor_position(self, motor_id: int):
        self.send_byte_command([CMD_GET_POS, 0x01, int(motor_id)])
        response = self.get_response(CMD_GET_POS)
        if response is None or len(response) < 6:
            return None
        return (response[3] << 8) + response[4]

    def get_all_position(self):
        motor_count = len(self.motors_list)
        id_list = [int(motor.id) for motor in self.motors_list]
        self.send_byte_command([CMD_GET_POS_ALL, motor_count] + id_list)
        response = self.get_response(CMD_GET_POS_ALL)
        if response is None:
            return [0] * motor_count

        values = []
        for index in range(motor_count):
            try:
                pos_high = response[3 + index * 2]
                pos_low = response[4 + index * 2]
                values.append((pos_high << 8) + pos_low)
            except Exception as exc:
                print(f"Motor {self.motors_list[index].id} response error: {exc}")
                values.append(0)
        return values

    def get_all_speed(self):
        motor_count = len(self.motors_list)
        id_list = [int(motor.id) for motor in self.motors_list]
        self.send_byte_command([CMD_GET_SPEED_ALL, motor_count] + id_list)
        response = self.get_response(CMD_GET_SPEED_ALL)
        if response is None:
            return [0] * motor_count
        return self._decode_signed_triplets(response, motor_count, "speed")

    def get_all_load(self):
        motor_count = len(self.motors_list)
        id_list = [int(motor.id) for motor in self.motors_list]
        self.send_byte_command([CMD_GET_LOAD_ALL, motor_count] + id_list)
        response = self.get_response(CMD_GET_LOAD_ALL)
        if response is None:
            return [0] * motor_count
        return self._decode_signed_triplets(response, motor_count, "load")

    def get_all_temper(self):
        motor_count = len(self.motors_list)
        id_list = [int(motor.id) for motor in self.motors_list]
        self.send_byte_command([CMD_GET_TEMP_ALL, motor_count] + id_list)
        response = self.get_response(CMD_GET_TEMP_ALL)
        if response is None:
            return [0] * motor_count

        values = []
        for index in range(motor_count):
            try:
                temp_high = response[3 + index * 2]
                temp_low = response[4 + index * 2]
                values.append((temp_high << 8) + temp_low)
            except Exception as exc:
                print(f"Motor {self.motors_list[index].id} response error: {exc}")
                values.append(0)
        return values

    def get_all_position_load_temper(self) -> tuple:
        if not self.motors_list:
            return ([], [], [])

        motor_count = len(self.motors_list)
        id_list = [int(motor.id) for motor in self.motors_list]
        self.send_byte_command([CMD_GET_POS_LOAD_TEMP_ALL, motor_count] + id_list)
        response = self.get_response(CMD_GET_POS_LOAD_TEMP_ALL, timeout=0.5)
        if response is None or len(response) != 3 + motor_count * 7 + 1:
            return ([0] * motor_count, [0] * motor_count, [0] * motor_count)

        positions = []
        loads = []
        tempers = []
        for index in range(motor_count):
            try:
                base = 3 + index * 7
                pos = (response[base] << 8) | response[base + 1]
                load = (response[base + 3] << 8) | response[base + 4]
                if response[base + 2] == 0x01:
                    load = -load
                temper = (response[base + 5] << 8) | response[base + 6]
                positions.append(pos)
                loads.append(load)
                tempers.append(temper)
            except Exception as exc:
                print(f"Motor {self.motors_list[index].id} response error: {exc}")
                positions.append(0)
                loads.append(0)
                tempers.append(0)
        return positions, loads, tempers

    def set_motor_id(self, old_id: int, new_id: int) -> bool:
        self.send_byte_command([CMD_SET_ID, 0x02, int(old_id), int(new_id)])
        response = self.get_response(CMD_SET_ID)
        return response is not None and len(response) >= 5 and response[3] == 0x01

    def set_motor_zero(self, motor_id: int) -> bool:
        self.send_byte_command([CMD_SET_ZERO, 0x01, int(motor_id)])
        response = self.get_response(CMD_SET_ZERO)
        return response is not None and len(response) >= 5 and response[3] == 0x01

    def set_wheel_mode(self, motor_id: int) -> bool:
        self.send_byte_command([CMD_SET_WHEEL_MODE, 0x01, int(motor_id)])
        response = self.get_response(CMD_SET_WHEEL_MODE)
        return response is not None and len(response) >= 5 and response[3] == 0x01

    def set_motor_speed(self, motor_id: int, speed: int, acc: int = 0) -> bool:
        raw_speed = int(speed)
        speed_low = raw_speed & 0xFF
        speed_high = (raw_speed >> 8) & 0xFF
        self.send_byte_command([CMD_SET_SPEED, 0x04, int(motor_id), speed_high, speed_low, int(acc) & 0xFF])
        response = self.get_response(CMD_SET_SPEED)
        return response is not None and len(response) >= 5 and response[3] == 0x01

    def move_all_init(self, spd: int, acc: int) -> None:
        for motor in self.motors_list:
            self.move_to_absolute(motor, motor.init, spd, acc)

    def move_all_absolute(self, pos_list: list, spd_list: list, acc_list: list) -> None:
        for index, motor in enumerate(self.motors_list):
            self.move_to_absolute(motor, pos_list[index], spd_list[index], acc_list[index])

    def move_all_offset(self, pos_list: list, spd_list: list, acc_list: list) -> None:
        for index, motor in enumerate(self.motors_list):
            self.move_to_offset(motor, pos_list[index], spd_list[index], acc_list[index])
