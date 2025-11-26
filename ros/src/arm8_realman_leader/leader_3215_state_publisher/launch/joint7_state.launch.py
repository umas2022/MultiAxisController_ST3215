# src/leader_3215_state_publisher/launch/joint7_state.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # 1. 获取包路径
    pkg_description_path = get_package_share_directory('leader_3215_description')
    pkg_state_publisher_path = get_package_share_directory('leader_3215_state_publisher')

    # 2. 声明参数
    # Rviz 配置文件路径（使用 description 包中的默认文件）
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(pkg_description_path, 'rviz', 'default.rviz'),
        description='Full path to the RVIZ config file to use'
    )

    # Xacro 文件路径
    urdf_file = LaunchConfiguration('urdf_file')
    declare_urdf_cmd = DeclareLaunchArgument(
        'urdf_file',
        default_value=os.path.join(pkg_description_path, 'urdf', 'leader_3215_description.xacro'),
        description='Name of the used URDF file'
    )
    
    # 3. 解析 Xacro
    robot_description = ParameterValue(Command([
        'xacro ',
        ' ',
        urdf_file
    ]), value_type=str)

    # 4. 核心节点配置

    # 4.1. robot_state_publisher
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    # 4.2. joint_state_publisher_gui
    start_joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # 4.3. Rviz2 可视化工具
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    # 4.4. 您的 joint7 pose publisher 节点
    pose_publisher_node = Node(
        package='leader_3215_state_publisher',
        executable='publish_joint7_pose',
        output='screen',
    )


    # 5. 组合 LaunchDescription
    ld = LaunchDescription()

    # 声明参数
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_urdf_cmd)

    # 添加核心节点
    ld.add_action(start_joint_state_publisher_gui_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(pose_publisher_node)

    return ld