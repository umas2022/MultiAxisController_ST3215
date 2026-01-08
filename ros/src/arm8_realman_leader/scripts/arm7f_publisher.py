#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys
import os

# Add the project root to the Python path
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
sys.path.insert(0, project_root)

# Import the controller with the correct path
from robot_driver.src.controller.ControllerArm8F import ControllerArm

class Arm8FPublisher(Node):
    def __init__(self, serial_port="/dev/ttyACM0", mode="usb"):
        super().__init__('arm8f_publisher')
        
        # Create publisher for joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Initialize the robot arm controller with provided parameters
        self.arm = ControllerArm(mode=mode, serial_port=serial_port)
        
        # Initialize the arm hardware
        if not self.arm.hardware_init():
            self.get_logger().error('Failed to initialize robot arm hardware')
            return
            
        # Wait for motors to be online
        self.get_logger().info('Checking if motors are online...')
        while not self.arm.online_check():
            self.get_logger().warn('Motors not online, retrying...')
            
        # Define joint names (should match URDF)
        self.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4',
            'joint5', 'joint6', 'joint7', 'joint8'
        ]

        
        # Timer to publish joint states at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info(f'Arm7F Publisher initialized successfully with {mode} mode on port {serial_port}')

    def timer_callback(self):
        try:
            # Get joint positions from the real robot arm
            positions = self.arm.get_all_position()

            if positions and len(positions) == 8:
                # Convert positions from 0-4096 range to radians
                # 0-4096 corresponds to approximately -3.14 to 3.14 radians (full circle)
                joint_positions_rad = []
                for pos in positions:
                    # # Convert from 0-4096 to -π to π
                    # rad = (pos - 2048) * (2 * 3.14159) / 4096
                    # Convert from 0-2048 to 0 to π, 4096 to 2048 to 0 to -π
                    if pos <= 2048:
                        rad = (pos) * (3.14159) / 2048
                    else:
                        rad = (pos - 4096) * (3.14159) / 2048
                    joint_positions_rad.append(rad)
                
                # Create joint state message
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = self.joint_names
                msg.position = joint_positions_rad
                
                # Publish the joint state
                self.joint_pub.publish(msg)
                
                # Log for debugging
                self.get_logger().debug(f'Published joint positions: {[f"{pos:.2f}" for pos in joint_positions_rad]}')
            else:
                self.get_logger().warn('Failed to get valid joint positions')
                
        except Exception as e:
            self.get_logger().error(f'Error reading joint positions: {str(e)}')

def main(args=None):
    # Initialize rclpy first
    rclpy.init(args=args)
    
    serial_port = "/dev/ttyACM0"
    mode = "usb"
    
    node = Arm8FPublisher(serial_port=serial_port, mode=mode)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()