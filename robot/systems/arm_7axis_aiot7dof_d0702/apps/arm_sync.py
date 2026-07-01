"""
机械臂主从机同步运动
"""

import time
from robot.systems.arm_7axis_aiot7dof_d0702.controller import ControllerArm

# 初始化控制器
arm_leader = ControllerArm(mode="usb", serial_port="COM7")
arm_follower = ControllerArm(mode="usb", serial_port="COM22")

arm_leader.hardware_init()
arm_follower.hardware_init()


# 在线检查
while True:
    print("leader arm online check ...")
    leader_online = arm_leader.online_check()
    print("follower arm online check ...")
    follower_online = arm_follower.online_check()
    if leader_online and follower_online:
        break
    print("\n")
    time.sleep(1)


arm_follower.move_all_absolute(arm_leader.get_all_position(), [500] * 7, [50] * 7)
print("Slave arm initialized to master position.")
print("start sync ...")


while True:
    arm_follower.move_all_absolute(arm_leader.get_all_position(), [2000] * 7, [0] * 7)
