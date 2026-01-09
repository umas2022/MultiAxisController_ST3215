#!/usr/bin/env python3
"""
电机ID扫描脚本
遍历1-20号电机ID，检查总线上的在线电机
"""

import sys
import os

# 添加项目根目录到Python路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(project_root)

from robot.src.drivers.motor_driver.STservo_sdk import *


def scan_motors(serial_port, baudrate=1000000, start_id=1, end_id=20):
    """
    扫描总线上在线的电机ID
    
    Args:
        serial_port: 串口名称
        baudrate: 波特率，默认1000000
        start_id: 起始电机ID，默认1
        end_id: 结束电机ID，默认20
    
    Returns:
        list: 在线电机ID列表
    """
    online_motors = []
    
    # 初始化端口处理器
    try:
        port_handler = PortHandler(serial_port)
    except Exception as e:
        print(f"无法初始化端口处理器: {e}")
        return online_motors
    
    # 打开端口
    if not port_handler.openPort():
        print(f"无法打开串口: {serial_port}")
        return online_motors
    
    # 设置波特率
    if not port_handler.setBaudRate(baudrate):
        print(f"无法设置波特率: {baudrate}")
        port_handler.closePort()
        return online_motors
    
    # 初始化数据包处理器
    packet_handler = sts(port_handler)
    
    print(f"开始扫描电机ID ({start_id}-{end_id})...")
    print(f"串口: {serial_port}, 波特率: {baudrate}")
    print("-" * 50)
    
    # 遍历指定范围的电机ID
    for motor_id in range(start_id, end_id + 1):
        print(f"检查电机ID {motor_id:2d}...", end=" ")
        
        # 使用ping命令检查电机是否在线
        sts_model_number, sts_comm_result, sts_error = packet_handler.ping(motor_id)
        
        if sts_comm_result == COMM_SUCCESS:
            online_motors.append(motor_id)
            print(f"在线 (型号: {sts_model_number})")
        else:
            print("离线")
        
        # 检查错误
        if sts_error != 0:
            print(f"错误: {packet_handler.getRxPacketError(sts_error)}")
    
    # 关闭端口
    port_handler.closePort()
    
    return online_motors


def main():
    """主函数"""
    # 参数配置
    serial_port = "COM8"  # Windows串口，Linux可改为"/dev/ttyUSB0"
    baudrate = 1000000    # 波特率
    start_id = 1          # 起始电机ID
    end_id = 20           # 结束电机ID
    
    # 扫描电机
    online_motors = scan_motors(serial_port, baudrate, start_id, end_id)
    
    # 显示结果
    print("-" * 50)
    print("扫描完成!")
    
    if online_motors:
        print(f"发现 {len(online_motors)} 个在线电机:")
        print(f"在线电机ID: {online_motors}")
    else:
        print("未发现任何在线电机")
    
    print("-" * 50)


if __name__ == "__main__":
    main()