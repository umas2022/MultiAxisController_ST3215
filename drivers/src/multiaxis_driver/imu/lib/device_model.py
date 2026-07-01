# coding: UTF-8
import threading
import time

import serial
from serial import SerialException


class SerialConfig:
    portName = ""
    baud = 9600


class DeviceModel:
    deviceName = "device"
    ADDR = 0x50

    def __init__(self, deviceName, protocolResolver, dataProcessor, dataUpdateListener):
        self.deviceName = deviceName
        self.protocolResolver = protocolResolver
        self.dataProcessor = dataProcessor
        self.dataUpdateListener = dataUpdateListener

        self.deviceData = {}
        self.isOpen = False
        self.serialPort = None
        self.serialConfig = SerialConfig()

        self._read_thread = None
        self._stop_event = threading.Event()

        print("初始化设备模型")

    def setDeviceData(self, key, value):
        self.deviceData[key] = value

    def getDeviceData(self, key):
        return self.deviceData.get(key)

    def removeDeviceData(self, key):
        if key in self.deviceData:
            del self.deviceData[key]

    def readDataTh(self, threadName, delay):
        print("启动" + threadName)
        while not self._stop_event.is_set():
            if not self.isOpen or self.serialPort is None:
                break

            try:
                tlen = self.serialPort.in_waiting
                if tlen > 0:
                    data = self.serialPort.read(tlen)
                    if data:
                        self.onDataReceived(data)
                else:
                    time.sleep(0.01)
            except Exception as exc:
                if not self._stop_event.is_set():
                    print(exc)
                break

        print("暂停")

    def openDevice(self):
        self.closeDevice()
        try:
            self.serialPort = serial.Serial(self.serialConfig.portName, self.serialConfig.baud, timeout=0.5)
            self.isOpen = True
            self._stop_event.clear()
            self._read_thread = threading.Thread(
                target=self.readDataTh,
                args=("Data-Received-Thread", 10),
                daemon=True,
            )
            self._read_thread.start()
        except SerialException as exc:
            self.serialPort = None
            self.isOpen = False
            print(f"打开 {self.serialConfig.portName} @ {self.serialConfig.baud} 失败: {exc}")

    def closeDevice(self):
        self.isOpen = False
        self._stop_event.set()

        if self.serialPort is not None:
            try:
                if self.serialPort.is_open:
                    self.serialPort.close()
                    print("端口关闭了")
            except Exception:
                pass
            self.serialPort = None

        if self._read_thread is not None and self._read_thread.is_alive():
            self._read_thread.join(timeout=1.0)
        self._read_thread = None

        print("设备关闭了")

    def onDataReceived(self, data):
        if self.protocolResolver is not None:
            self.protocolResolver.passiveReceiveData(data, self)

    def get_int(self, dataBytes):
        return int.from_bytes(dataBytes, "little", signed=True)

    def get_unint(self, dataBytes):
        return int.from_bytes(dataBytes, "little")

    def sendData(self, data):
        if self.protocolResolver is not None:
            self.protocolResolver.sendData(data, self)

    def readReg(self, regAddr, regCount):
        if self.protocolResolver is not None:
            return self.protocolResolver.readReg(regAddr, regCount, self)
        return None

    def writeReg(self, regAddr, sValue):
        if self.protocolResolver is not None:
            self.protocolResolver.writeReg(regAddr, sValue, self)

    def unlock(self):
        if self.protocolResolver is not None:
            self.protocolResolver.unlock(self)

    def save(self):
        if self.protocolResolver is not None:
            self.protocolResolver.save(self)

    def AccelerationCalibration(self):
        if self.protocolResolver is not None:
            self.protocolResolver.AccelerationCalibration(self)

    def BeginFiledCalibration(self):
        if self.protocolResolver is not None:
            self.protocolResolver.BeginFiledCalibration(self)

    def EndFiledCalibration(self):
        if self.protocolResolver is not None:
            self.protocolResolver.EndFiledCalibration(self)

    def sendProtocolData(self, data):
        if self.protocolResolver is not None:
            self.protocolResolver.sendData(data)
