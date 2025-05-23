import sys
import time
import numpy as np

sys.path.append("..")
from STservo_sdk import *


from dataclasses import dataclass


@dataclass
class MotorConfig:
    id: int
    min: int
    max: int
    init: int
    reverse: bool


class MultiAxisController:
    def __init__(self, serial_port):
        self.serial_port = serial_port
        self.baudrate = 1000000

        self.motors_list: list[MotorConfig] = []
        self.motors_num = 0

        try:
            self.port_handler = PortHandler(self.serial_port)
        except Exception as e:
            print(f"Failed to open the port: {e}")
            exit(1)
        self.packet_handler = sts(self.port_handler)

    def hardware_init(self) -> None:
        """
        打开串口，设定电机波特率
        """
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
        """
        检查所有电机在线
        """
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

    def _get_raw_position(self, norm_pos: int, motor: MotorConfig) -> int:
        """
        position convert: normalized position to raw position
        """
        if motor.reverse:
            return motor.max - norm_pos
        else:
            return motor.min + norm_pos

    def _get_norm_position(self, raw_pos: int, motor: MotorConfig) -> int:
        """
        position convert: raw position to normalized position
        """
        if motor.reverse:
            return motor.max - raw_pos
        else:
            return raw_pos - motor.min

    def _move_to_absolute(self, motor: MotorConfig, position: int, speed: int, acc: int) -> bool:
        range_check = True
        target_pos = np.clip(position, motor.min, motor.max)
        if target_pos != position:
            print(f"Target position {position} is out of range for motor {motor.id}.")
            range_check = False
        sts_comm_result, sts_error = self.packet_handler.RegWritePosEx(motor.id, target_pos, speed, acc)
        return range_check

    def _move_to_offset(self, motor: MotorConfig, offset: int, speed: int, acc: int) -> bool:
        if motor.reverse:
            offset = -offset
        return self._move_to_absolute(motor, motor.init + offset, speed, acc)

    def _move_excute(self) -> None:
        self.packet_handler.RegAction()

    def move_all_init(self, spd, acc) -> None:
        """
        Move all motors to their initial position
        """
        for each_motor in self.motors_list:
            self._move_to_absolute(each_motor, each_motor.init, spd, acc)
        self._move_excute()

    def get_all_position_raw(self) -> list:
        """
        Get all motors position
        """
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

    def move_all_position_absolute(self, pos_list: list, spd_list: list, acc_list: list) -> None:
        """
        Move all motors to the given position
        """
        for i, each_motor in enumerate(self.motors_list):
            self._move_to_absolute(each_motor, pos_list[i], spd_list[i], acc_list[i])
        self._move_excute()

    def move_all_position_offset(self, pos_list: list, spd_list: list, acc_list: list) -> None:
        """
        Move all motors to the given position
        """
        for i, each_motor in enumerate(self.motors_list):
            self._move_to_offset(each_motor, pos_list[i], spd_list[i], acc_list[i])
        self._move_excute()
