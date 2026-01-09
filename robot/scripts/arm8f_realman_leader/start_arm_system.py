#!/usr/bin/env python3
"""
全局启动脚本，用于启动整个arm8f_realman_leader机器人系统
启动以下组件：
1. real_joint7_state.launch.py - 机器人核心launch文件
2. arm7f_publisher.py - 获取真机位置并发布
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

    # 获取项目根目录（基于当前脚本位置）
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Normalize path separators to handle cross-platform issues
    script_dir = os.path.normpath(script_dir)
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(script_dir)))
    project_root = os.path.normpath(project_root)
    ros_dir = os.path.join(project_root, "ros")
    ros_dir = os.path.normpath(ros_dir)

    # 调试：打印路径信息
    print(f"Script directory: {script_dir}")
    print(f"Project root: {project_root}")
    print(f"ROS directory: {ros_dir}")
    print(f"ROS directory exists: {os.path.exists(ros_dir)}")
    print(f"arm7f_publisher.py path: {os.path.join(ros_dir, 'src', 'arm8_realman_leader', 'scripts', 'arm7f_publisher.py')}")
    print(f"arm7f_publisher.py exists: {os.path.exists(os.path.join(ros_dir, 'src', 'arm8_realman_leader', 'scripts', 'arm7f_publisher.py'))}")

    # 确保ROS目录存在
    if not os.path.exists(ros_dir):
        print(f"错误: ROS目录不存在: {ros_dir}")
        return

    # Handle WSL/Unix path conversion if running on WSL
    import platform
    import pathlib

    # Convert Windows path to WSL path if needed
    if os.name == 'posix':  # Unix/Linux/WSL
        # Check if we're on WSL by looking for WSL-specific indicators
        try:
            with open('/proc/version', 'r') as f:
                if 'microsoft' in f.read().lower():
                    # We're in WSL, convert Windows path to WSL path
                    if ros_dir.startswith('C:'):
                        ros_dir = '/mnt/c' + ros_dir[2:].replace('\\', '/')
                    elif ros_dir.startswith('/'):  # already a Unix path
                        pass
                    else:
                        # Try to convert Windows-style path to Unix-style
                        ros_dir = ros_dir.replace('\\', '/')
                        if ros_dir[1:3] == ':\\':  # C:\ format
                            drive_letter = ros_dir[0].lower()
                            ros_dir = f'/mnt/{drive_letter}{ros_dir[2:]}'
        except:
            pass  # If /proc/version doesn't exist or we can't read it, continue with original path

    # 1. 启动real_joint7_state.launch.py
    # 这需要在ROS环境中运行
    launch_cmd = [
        "bash", "-c",
        f"cd '{ros_dir}' && source install/setup.bash && ros2 launch leader_3215_state_publisher real_joint7_state.launch.py"
    ]
    launch_proc = run_command(launch_cmd, "real_joint7_state.launch.py")
    if not launch_proc:
        print("无法启动real_joint7_state.launch.py，退出")
        return

    # 等待一段时间让核心系统启动
    print("等待ros核心系统启动...")
    time.sleep(5)

    # 2. 启动arm7f_publisher.py
    arm_publisher_script = os.path.join(ros_dir, "src", "arm8_realman_leader", "scripts", "arm7f_publisher.py")

    # 确保publisher脚本存在
    if not os.path.exists(arm_publisher_script):
        print(f"错误: arm_publisher脚本不存在: {arm_publisher_script}")
        return

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