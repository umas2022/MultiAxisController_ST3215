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

class MultiAxisUSB(MultiAxisControllerInterface):
    """
    多轴控制器USB连接
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

    def move_all_init(self, spd: int, acc: int) -> None:
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

    def get_all_speed(self) -> list:
        speed_list = []
        for each_motor in self.motors_list:
            sts_present_speed, sts_comm_result, sts_error = self.packet_handler.ReadSpeed(each_motor.id)
            if sts_comm_result != COMM_SUCCESS:
                print(f"Motor {each_motor.id} is offline")
                return []
            else:
                speed_list.append(sts_present_speed)
            if sts_error != 0:
                print("%s" % self.packet_handler.getRxPacketError(sts_error))
                return []
        return speed_list

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

    def get_all_position_load_temper(self) -> tuple:
        return self.get_all_position(), self.get_all_load(), self.get_all_temper()

    def move_all_absolute(self, pos_list: list, spd_list: list, acc_list: list) -> None:
        for i, each_motor in enumerate(self.motors_list):
            self.move_to_absolute(each_motor, pos_list[i], spd_list[i], acc_list[i])
        self.move_excute()

    def move_all_offset(self, pos_list: list, spd_list: list, acc_list: list) -> None:
        for i, each_motor in enumerate(self.motors_list):
            self.move_to_offset(each_motor, pos_list[i], spd_list[i], acc_list[i])
        self.move_excute()
