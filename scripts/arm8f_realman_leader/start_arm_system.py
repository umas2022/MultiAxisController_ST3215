#!/usr/bin/env python3
"""
全局启动脚本，用于启动整个arm8f_realman_leader机器人系统
启动以下组件：
1. real_joint7_state.launch.py - 机器人核心launch文件
2. arm7f_publisher.py - 真机控制节点

source install/setup.sh
ros2 launch leader_3215_state_publisher real_joint7_state.launch.py

python arm7f_publisher.py
"""

import subprocess
import sys
import os
import signal
import time

# 存储所有启动的进程
processes = []

def signal_handler(sig, frame):
    """处理Ctrl+C信号，优雅关闭所有进程"""
    print("\n接收到终止信号，正在关闭所有进程...")
    for proc in processes:
        try:
            proc.terminate()
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
        except Exception as e:
            print(f"关闭进程时出错: {e}")
    print("所有进程已关闭")
    sys.exit(0)

def run_command(command, name):
    """运行命令并将其添加到进程列表"""
    print(f"启动 {name}...")
    print(f"命令: {' '.join(command)}")
    
    try:
        # 使用shell=False以更好地控制进程
        proc = subprocess.Popen(command)
        processes.append(proc)
        print(f"{name} 已启动 (PID: {proc.pid})")
        return proc
    except Exception as e:
        print(f"启动 {name} 失败: {e}")
        return None

def main():
    # 注册信号处理器
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # 获取项目根目录
    project_root = "/mnt/c/Users/umas_local/Documents/user/ws_local/ws_code/self/MultiAxisController_ST3215"
    ros_dir = os.path.join(project_root, "ros")
    
    # 1. 启动real_joint7_state.launch.py
    # 这需要在ROS环境中运行
    launch_cmd = [
        "bash", "-c",
        f"cd {ros_dir} && source install/setup.bash && ros2 launch leader_3215_state_publisher real_joint7_state.launch.py"
    ]
    launch_proc = run_command(launch_cmd, "real_joint7_state.launch.py")
    if not launch_proc:
        print("无法启动real_joint7_state.launch.py，退出")
        return
    
    # 等待一段时间让核心系统启动
    print("等待ros核心系统启动...")
    time.sleep(5)
    
    # 2. 启动arm7f_publisher.py
    arm_publisher_script = os.path.join(project_root, "ros", "src", "arm8_realman_leader", "scripts", "arm7f_publisher.py")
    arm_publisher_cmd = ["python3", arm_publisher_script]
    arm_publisher_proc = run_command(arm_publisher_cmd, "arm7f_publisher.py")
    
    print("所有组件已启动，按Ctrl+C退出")
    
    # 等待所有进程结束
    try:
        while True:
            # 检查进程是否仍在运行
            for proc in processes[:]:  # 使用切片创建副本以安全地迭代
                if proc.poll() is not None:  # 进程已结束
                    print(f"进程 {proc.pid} 已结束")
                    processes.remove(proc)
            
            if not processes:
                print("所有进程已完成")
                break
                
            time.sleep(1)
            
    except KeyboardInterrupt:
        signal_handler(signal.SIGINT, None)

if __name__ == "__main__":
    main()