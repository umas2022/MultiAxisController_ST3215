#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class SimpleJointPublisher(Node):
    def __init__(self):
        super().__init__('simple_joint_publisher')
        
        # Create publisher for joint states
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Timer to publish joint states at 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Initialize joint positions (currently just joint7)
        self.joint_positions = {
            'joint1': 0.0,
            'joint2': 0.0,
            'joint3': 0.0,
            'joint4': 0.0,
            'joint5': 0.0,
            'joint6': 0.0,
            'joint7': 0.0
        }
        
        # Counter for simple movement simulation
        self.counter = 0
        
    def timer_callback(self):
        # Create joint state message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Set joint names and positions
        msg.name = list(self.joint_positions.keys())
        msg.position = list(self.joint_positions.values())
        
        # Publish the joint state
        self.joint_pub.publish(msg)
        
        # Simple movement simulation for joint7 (you can remove this when connecting to real robot)
        self.counter += 1
        self.joint_positions['joint7'] = 0.5 * (1 + 0.5 * (self.counter % 100) / 100.0)
        
        # Log for debugging
        self.get_logger().info(f'Publishing joint7 position: {self.joint_positions["joint7"]:.3f}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleJointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()