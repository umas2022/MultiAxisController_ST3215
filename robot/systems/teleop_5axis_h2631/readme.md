# teleop_5axis_h2631

H2631 五轴 RealMan 遥操作手柄。

```sh
# start
python -m robot.systems.teleop_5axis_h2631.apps.raw_state_publisher
```

- topic name: joystick/motor_position

```sh
umas@DESKTOP-VIALD5L:~$ ros2 topic echo joystick/motor_position
layout:
  dim: []
  data_offset: 0
data:
- 2021
- 2056
- 2055
- 3528
- 2240
```


- 所有电机初始位置2048，误差偏移约±10

- joint1
    - 方向：右手摇杆前推减小，后拉增大
    - 范围：[1513,2590]

- joint2
    - 方向：右手摇杆左偏增大，右偏减小
    - 范围：[1390,2714]

- joint3
    - 方向：右手夹爪扳机打开增大，关闭减小
    - 范围：[2048,2583]

- joint4
    - 方向：左手拉杆后拉增大，前推减小
    - 范围：[2048,3527]

- joint5
    - 方向：左手旋钮俯视顺时针旋转增大，逆时针减小
    - 范围：[0,4095]整圈连续
