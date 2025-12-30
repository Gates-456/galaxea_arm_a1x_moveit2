#!/usr/bin/env python3

"""
End-effector control node using MoveIt2
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from rclpy.qos import QoSProfile
from tf2_ros import TransformBroadcaster
import sys


class EndEffectorControlNode(Node):
    def __init__(self):
        super().__init__('endeffector_control_node')
        
        # 声明参数，先检查是否已经存在
        if not self.has_parameter('move_group_name'):
            self.declare_parameter('move_group_name', 'a1x_group')  # 修正规划组名称
        if not self.has_parameter('end_effector_link'):
            self.declare_parameter('end_effector_link', 'arm_link6')  # 从SRDF文件中可以看到末端链接是arm_link6
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        # 获取参数
        self.move_group_name = self.get_parameter('move_group_name').value
        self.end_effector_link = self.get_parameter('end_effector_link').value
        self.use_sim_time = self.get_parameter('use_sim_time').value

        # 检查pymoveit2 Python接口是否可用
        try:
            from pymoveit2 import MoveIt2
            self.get_logger().info('Using pymoveit2 package')
            
            # 初始化MoveIt2接口 - 使用正确的API
            # 根据a1x_moveit_config中的配置，我们有6个关节
            joint_names = ["arm_joint1", "arm_joint2", "arm_joint3", 
                          "arm_joint4", "arm_joint5", "arm_joint6"]
            
            self.moveit2 = MoveIt2(
                node=self,
                joint_names=joint_names,
                base_link_name="base_link",  # 根据你的URDF配置调整
                end_effector_name=self.end_effector_link,
                group_name=self.move_group_name
            )
            
        except ImportError:
            self.get_logger().warn('pymoveit2 package not available, using alternative approach')
            self.moveit2 = None

        # 创建发布者和订阅者
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            'target_pose',
            self.target_pose_callback,
            QoSProfile(depth=10)
        )
        
        self.get_logger().info(f'EndEffectorControlNode initialized with group: {self.move_group_name}, '
                               f'end effector: {self.end_effector_link}')
        
    def target_pose_callback(self, msg: PoseStamped):
        """处理目标姿态消息"""
        self.get_logger().info(f'Received target pose: position=({msg.pose.position.x}, '
                               f'{msg.pose.position.y}, {msg.pose.position.z}), '
                               f'orientation=({msg.pose.orientation.x}, {msg.pose.orientation.y}, '
                               f'{msg.pose.orientation.z}, {msg.pose.orientation.w})')
        
        if self.moveit2 is not None:
            # 使用MoveIt2接口移动到目标姿态
            self.moveit2.move_to_pose(
                position=[msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
                quat_xyzw=[
                    msg.pose.orientation.x, 
                    msg.pose.orientation.y, 
                    msg.pose.orientation.z, 
                    msg.pose.orientation.w
                ]
            )
            
            # 等待结果
            self.moveit2.wait_until_executed()
            self.get_logger().info('Successfully moved end-effector to target pose')
        else:
            self.get_logger().error('MoveIt2 interface not available')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = EndEffectorControlNode()
        rclpy.spin(node)
        node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()