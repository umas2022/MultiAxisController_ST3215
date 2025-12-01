#!/usr/bin/env python3
"""
电机位置偏移设定脚本
设定当前位置为指定值（例如1024），通过写入偏移量寄存器实现
之后读取电机位置时会返回设定的值
"""

import sys
import os

# 添加项目根目录到Python路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(project_root)

from src.drivers.motor_driver.STservo_sdk import *


def set_current_position(serial_port, motor_id, target_position, baudrate=1000000):
    """
    设定当前位置为目标值
    通过写入偏移量寄存器，使电机读取位置返回目标值
    
    工作原理：
    1. 读取当前实际位置
    2. 计算偏移量 = 目标位置 - 实际位置
    3. 将偏移量写入EPROM的偏移寄存器
    4. 之后读取位置 = 实际位置 + 偏移量 = 目标位置
    
    Args:
        serial_port: 串口名称
        motor_id: 电机ID
        target_position: 目标位置值（0-4095）
        baudrate: 波特率，默认1000000
    
    Returns:
        bool: 是否设置成功
    """
    # 初始化端口处理器
    try:
        port_handler = PortHandler(serial_port)
    except Exception as e:
        print(f"无法初始化端口处理器: {e}")
        return False
    
    # 打开端口
    if not port_handler.openPort():
        print(f"无法打开串口: {serial_port}")
        return False
    
    # 设置波特率
    if not port_handler.setBaudRate(baudrate):
        print(f"无法设置波特率: {baudrate}")
        port_handler.closePort()
        return False
    
    # 初始化数据包处理器
    packet_handler = sts(port_handler)
    
    print(f"连接电机ID {motor_id}...")
    print(f"串口: {serial_port}, 波特率: {baudrate}")
    print("-" * 60)
    
    # 1. 检查电机是否在线
    print(f"1. 检查电机是否在线...")
    sts_model_number, sts_comm_result, sts_error = packet_handler.ping(motor_id)
    if sts_comm_result != COMM_SUCCESS:
        print(f"错误: 无法连接到电机ID {motor_id}")
        print(f"通信结果: {packet_handler.getTxRxResult(sts_comm_result)}")
        port_handler.closePort()
        return False
    print(f"   电机在线 (型号: {sts_model_number})")
    
    # 2. 读取当前实际位置（不含偏移）
    print(f"\n2. 读取当前位置...")
    # 先读取当前偏移量
    current_offset, sts_comm_result, sts_error = packet_handler.read2ByteTxRx(motor_id, STS_OFS_L)
    if sts_comm_result != COMM_SUCCESS:
        print(f"错误: 无法读取当前偏移量")
        port_handler.closePort()
        return False
    
    # 转换为有符号数（偏移量是有符号的，采用12-bit表示，符号位为bit11）
    current_offset = packet_handler.sts_tohost(current_offset, 11)
    print(f"   当前偏移量: {current_offset}")
    
    # 读取当前位置（包含偏移）
    current_position_with_offset, sts_comm_result, sts_error = packet_handler.ReadPos(motor_id)
    if sts_comm_result != COMM_SUCCESS:
        print(f"错误: 无法读取当前位置")
        port_handler.closePort()
        return False
    print(f"   当前位置（含偏移）: {current_position_with_offset}")
    
    # 计算实际物理位置（去除偏移）
    actual_position = current_position_with_offset - current_offset
    print(f"   实际物理位置: {actual_position}")
    
    # 3. 计算新的偏移量
    print(f"\n3. 计算新偏移量...")
    new_offset = target_position - actual_position
    print(f"   目标位置: {target_position}")
    print(f"   新偏移量: {new_offset}")
    
    # 检查偏移量范围（-2048到2047）
    if new_offset < -2048 or new_offset > 2047:
        print(f"警告: 偏移量 {new_offset} 超出范围 (-2048~2047)")
        print(f"建议将目标位置设置在 {actual_position - 2048} 到 {actual_position + 2047} 之间")
        port_handler.closePort()
        return False
    
    # 4. 解锁EPROM
    print(f"\n4. 解锁EPROM...")
    sts_comm_result, sts_error = packet_handler.unLockEprom(motor_id)
    if sts_comm_result != COMM_SUCCESS:
        print(f"错误: 无法解锁EPROM")
        port_handler.closePort()
        return False
    print(f"   EPROM已解锁")
    
    # 5. 写入新偏移量
    print(f"\n5. 写入新偏移量到寄存器...")
    # 将有符号数转换为无符号数写入（12-bit，符号位bit11）
    offset_value = packet_handler.sts_toscs(new_offset, 11)
    sts_comm_result, sts_error = packet_handler.write2ByteTxRx(
        motor_id, 
        STS_OFS_L, 
        offset_value
    )
    if sts_comm_result != COMM_SUCCESS:
        print(f"错误: 无法写入偏移量")
        packet_handler.LockEprom(motor_id)  # 即使失败也要锁定
        port_handler.closePort()
        return False
    print(f"   偏移量已写入")
    # 读取并打印写入后的偏移寄存器，便于诊断
    import time
    time.sleep(0.05)
    raw_offset_reg, sts_comm_result, sts_error = packet_handler.read2ByteTxRx(motor_id, STS_OFS_L)
    if sts_comm_result == COMM_SUCCESS:
        signed_offset_reg = packet_handler.sts_tohost(raw_offset_reg, 11)
        print(f"   写入后寄存器(原始): 0x{raw_offset_reg:04X}, 解析为: {signed_offset_reg}")
    else:
        print(f"   警告: 无法读取写入后的偏移寄存器以验证")
    
    # 6. 锁定EPROM
    print(f"\n6. 锁定EPROM...")
    sts_comm_result, sts_error = packet_handler.LockEprom(motor_id)
    if sts_comm_result != COMM_SUCCESS:
        print(f"警告: 无法锁定EPROM（偏移量已写入）")
    else:
        print(f"   EPROM已锁定")
    # 等待EPROM写入完成并再次读取偏移寄存器确认
    time.sleep(0.15)
    raw_offset_reg2, sts_comm_result2, sts_error2 = packet_handler.read2ByteTxRx(motor_id, STS_OFS_L)
    if sts_comm_result2 == COMM_SUCCESS:
        signed_offset_reg2 = packet_handler.sts_tohost(raw_offset_reg2, 11)
        print(f"   锁定后寄存器(原始): 0x{raw_offset_reg2:04X}, 解析为: {signed_offset_reg2}")
    else:
        print(f"   警告: 无法读取锁定后的偏移寄存器以验证")
    
    # 7. 验证新位置
    print(f"\n7. 验证新位置...")
    import time
    time.sleep(0.1)  # 等待寄存器更新
    
    verify_position, sts_comm_result, sts_error = packet_handler.ReadPos(motor_id)
    if sts_comm_result != COMM_SUCCESS:
        print(f"警告: 无法验证新位置")
    else:
        print(f"   读取位置: {verify_position}")
        if abs(verify_position - target_position) < 10:  # 允许小误差
            print(f"   ✓ 验证成功！位置已设定为 {target_position}")
        else:
            print(f"   ✗ 验证失败！期望 {target_position}，实际 {verify_position}")
    
    # 关闭端口
    port_handler.closePort()
    
    print("-" * 60)
    print("设置完成！")
    return True


def read_current_position(serial_port, motor_id, baudrate=1000000):
    """
    读取电机当前位置（含偏移量）
    
    Args:
        serial_port: 串口名称
        motor_id: 电机ID
        baudrate: 波特率，默认1000000
    
    Returns:
        int: 当前位置，失败返回None
    """
    # 初始化端口处理器
    try:
        port_handler = PortHandler(serial_port)
    except Exception as e:
        print(f"无法初始化端口处理器: {e}")
        return None
    
    # 打开端口
    if not port_handler.openPort():
        print(f"无法打开串口: {serial_port}")
        return None
    
    # 设置波特率
    if not port_handler.setBaudRate(baudrate):
        print(f"无法设置波特率: {baudrate}")
        port_handler.closePort()
        return None
    
    # 初始化数据包处理器
    packet_handler = sts(port_handler)
    
    # 读取位置
    position, sts_comm_result, sts_error = packet_handler.ReadPos(motor_id)
    
    # 关闭端口
    port_handler.closePort()
    
    if sts_comm_result != COMM_SUCCESS:
        print(f"错误: 无法读取位置")
        return None
    
    return position


def main():
    """主函数"""
    # 参数配置
    SERIAL_PORT = "COM8"      # Windows串口，Linux可改为"/dev/ttyUSB0"
    MOTOR_ID = 1              # 电机ID
    TARGET_POSITION = 1024    # 目标位置（设定当前位置为此值）
    BAUDRATE = 1000000        # 波特率
    
    print("=" * 60)
    print("电机位置偏移设定工具")
    print("=" * 60)
    
    # 方式1: 设定当前位置为指定值
    print("\n【功能】设定当前位置")
    success = set_current_position(SERIAL_PORT, MOTOR_ID, TARGET_POSITION, BAUDRATE)
    
    if success:
        # 读取验证
        print("\n【验证】读取当前位置...")
        position = read_current_position(SERIAL_PORT, MOTOR_ID, BAUDRATE)
        if position is not None:
            print(f"当前读取位置: {position}")
            print(f"设定目标位置: {TARGET_POSITION}")
            if abs(position - TARGET_POSITION) < 10:
                print("✓ 位置设定成功！")
            else:
                print("✗ 位置设定可能存在偏差")
    
    print("\n" + "=" * 60)


if __name__ == "__main__":
    main()
