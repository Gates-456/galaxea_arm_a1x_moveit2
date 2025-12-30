#!/usr/bin/env python3

"""
Cartesian path planner using MoveIt2
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from moveit_msgs.msg import MoveItErrorCodes
from builtin_interfaces.msg import Duration
import numpy as np
from math import pi, cos, sin


class CartesianPathPlanner(Node):
    def __init__(self):
        super().__init__('cartesian_path_planner')
        
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

        self.get_logger().info(f'CartesianPathPlanner initialized with group: {self.move_group_name}, '
                               f'end effector: {self.end_effector_link}')

    def move_in_circle(self, center_x=0.3, center_y=0.0, center_z=0.3, radius=0.1, steps=20):
        """让末端执行器沿圆形轨迹移动"""
        self.get_logger().info(f'Moving in circle: center=({center_x}, {center_y}, {center_z}), radius={radius}')
        
        if self.moveit2 is None:
            self.get_logger().error('MoveIt2 interface not available')
            return False

        # 创建圆形路径的航路点
        waypoints = []
        for i in range(steps + 1):  # +1 以回到起始点
            angle = 2.0 * pi * i / steps
            
            # 计算圆形路径上的点
            x = center_x + radius * cos(angle)
            y = center_y + radius * sin(angle)
            z = center_z  # 保持z坐标不变
            
            # 创建姿态对象
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            
            # 保持一个简单的朝向（可以根据需要修改）
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0
            
            waypoints.append(pose)
        
        # 执行路径 - 由于pymoveit2可能不直接支持笛卡尔路径，我们使用一系列目标点来模拟
        success = True
        for waypoint in waypoints:
            self.moveit2.move_to_pose(
                position=[waypoint.position.x, waypoint.position.y, waypoint.position.z],
                quat_xyzw=[
                    waypoint.orientation.x,
                    waypoint.orientation.y,
                    waypoint.orientation.z,
                    waypoint.orientation.w
                ]
            )
            
            if not self.moveit2.wait_until_executed():
                success = False
                break
        
        if success:
            self.get_logger().info('Successfully executed circular path')
            return True
        else:
            self.get_logger().error('Failed to execute circular path')
            return False

    def move_in_line(self, start_pose, end_pose, steps=10):
        """让末端执行器沿直线移动"""
        self.get_logger().info('Moving in line')
        
        if self.moveit2 is None:
            self.get_logger().error('MoveIt2 interface not available')
            return False

        # 创建直线路径的航路点
        waypoints = []
        
        # 计算每步的增量
        dx = (end_pose.position.x - start_pose.position.x) / steps
        dy = (end_pose.position.y - start_pose.position.y) / steps
        dz = (end_pose.position.z - start_pose.position.z) / steps
        
        for i in range(1, steps):
            intermediate_pose = Pose()
            intermediate_pose.position.x = start_pose.position.x + dx * i
            intermediate_pose.position.y = start_pose.position.y + dy * i
            intermediate_pose.position.z = start_pose.position.z + dz * i
            # 保持朝向一致，这里简单地使用起始姿态的朝向
            intermediate_pose.orientation = start_pose.orientation
            
            waypoints.append(intermediate_pose)
        
        # 添加终点
        waypoints.append(end_pose)
        
        # 执行路径
        success = True
        for waypoint in waypoints:
            self.moveit2.move_to_pose(
                position=[waypoint.position.x, waypoint.position.y, waypoint.position.z],
                quat_xyzw=[
                    waypoint.orientation.x,
                    waypoint.orientation.y,
                    waypoint.orientation.z,
                    waypoint.orientation.w
                ]
            )
            
            if not self.moveit2.wait_until_executed():
                success = False
                break
        
        if success:
            self.get_logger().info('Successfully executed linear path')
            return True
        else:
            self.get_logger().error('Failed to execute linear path')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CartesianPathPlanner()
        
        # 示例：沿圆形轨迹移动
        node.get_logger().info("Starting circular motion example...")
        success = node.move_in_circle(center_x=0.3, center_y=0.0, center_z=0.3, radius=0.1)
        
        if success:
            node.get_logger().info("Circular motion completed successfully!")
        else:
            node.get_logger().error("Circular motion failed!")
        
        node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()