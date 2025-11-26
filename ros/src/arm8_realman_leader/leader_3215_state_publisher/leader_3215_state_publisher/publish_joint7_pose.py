#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_matrix
import numpy as np


class Joint7PosePublisher(Node):
    def __init__(self):
        super().__init__('joint7_pose_publisher')

        # 发布 joint7 位姿
        self.pose_pub = self.create_publisher(PoseStamped, '/joint7_pose', 10)

        # TF2 buffer 和 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 定时查询 TF
        self.timer = self.create_timer(0.02, self.timer_callback)   # 50 Hz

        # 固定父链名称（根据你的 URDF）
        self.base_frame = 'base_link'
        self.joint7_frame = 'link7'

    def timer_callback(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.joint7_frame,
                rclpy.time.Time()
            )

            msg = PoseStamped()
            msg.header.frame_id = self.base_frame
            msg.header.stamp = self.get_clock().now().to_msg()

            msg.pose.position.x = trans.transform.translation.x
            msg.pose.position.y = trans.transform.translation.y
            msg.pose.position.z = trans.transform.translation.z

            msg.pose.orientation.x = trans.transform.rotation.x
            msg.pose.orientation.y = trans.transform.rotation.y
            msg.pose.orientation.z = trans.transform.rotation.z
            msg.pose.orientation.w = trans.transform.rotation.w

            self.pose_pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Waiting for TF: {str(e)}")



def main(args=None):
    rclpy.init(args=args)
    node = Joint7PosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
