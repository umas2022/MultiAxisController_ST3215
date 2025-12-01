# leader_3215

- 3215 leader arm
- real robot controller : https://github.com/umas2022/MultiAxisController_ST3215

```sh
# view basic urdf description
ros2 launch leader_3215_description launch.py

# launch sim_env with joint7_state_publisher
ros2 launch leader_3215_state_publisher joint7_state.launch.py
ros2 topic echo /joint7_pose

# real real_env with joint7_state_publisher (without joint_state_publisher_gui)
ros2 launch leader_3215_state_publisher real_joint7_state.launch.py
# publish joint_state with random value
ros2 run leader_3215_state_publisher simple_joint_publisher
# publish joint_state with real leader arm
cd [path_to_MultiAxisController]
python scripts/arm7f/arm7f_publisher.py

# wsl serial
usbipd list
usbipd.exe bind --busid 1-1
usbipd.exe attach --wsl --busid 1-1
usbipd detach --busid 1-1
usbipd unbind --busid 1-1
```

## leader_3215_description

- urdf description

## leader_3215_state_publisher

- ./launch/joint7_state.launch.py
    - publish joint7_pose with ./src/publish_joint7_pose.py
    - launch joint_state_publisher_gui

- ./launch/real_joint7_state.launch.py
    - launch without joint_state_publisher_gui

- ./src/simple_joint_publisher.py
    - publish joint_state with random value

