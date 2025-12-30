#!/usr/bin/env python3

"""
Move to pose node using MoveIt2
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.msg import MoveItErrorCodes
from rclpy.qos import QoSProfile
from math import pi
import time


class MoveToPoseNode(Node):
    def __init__(self):
        super().__init__("move_to_pose_node")

        # 声明参数，先检查是否已经存在
        if not self.has_parameter("move_group_name"):
            self.declare_parameter("move_group_name", "a1x_group")  # 修正规划组名称
        if not self.has_parameter("end_effector_link"):
            self.declare_parameter(
                "end_effector_link", "arm_link6"
            )  # 从SRDF文件中可以看到末端链接是arm_link6
        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", False)
        if not self.has_parameter("velocity_scaling_factor"):
            self.declare_parameter(
                "velocity_scaling_factor", 32.0
            )  # 添加速度缩放参数，默认0.1（较慢速度）
        if not self.has_parameter("acceleration_scaling_factor"):
            self.declare_parameter(
                "acceleration_scaling_factor", 10.0
            )  # 添加加速度缩放参数

        # 获取参数
        self.move_group_name = self.get_parameter("move_group_name").value
        self.end_effector_link = self.get_parameter("end_effector_link").value
        self.use_sim_time = self.get_parameter("use_sim_time").value
        self.velocity_scaling_factor = self.get_parameter(
            "velocity_scaling_factor"
        ).value
        self.acceleration_scaling_factor = self.get_parameter(
            "acceleration_scaling_factor"
        ).value

        # 检查pymoveit2 Python接口是否可用
        try:
            from pymoveit2 import MoveIt2

            self.get_logger().info("Using pymoveit2 package")

            # 初始化MoveIt2接口 - 使用正确的API
            # 根据a1x_moveit_config中的配置，我们有6个关节
            joint_names = [
                "arm_joint1",
                "arm_joint2",
                "arm_joint3",
                "arm_joint4",
                "arm_joint5",
                "arm_joint6",
            ]

            self.moveit2 = MoveIt2(
                node=self,
                joint_names=joint_names,
                base_link_name="base_link",  # 根据你的URDF配置调整
                end_effector_name=self.end_effector_link,
                group_name=self.move_group_name,
            )

            # 设置速度和加速度缩放因子
            self.moveit2.max_velocity_scaling_factor = self.velocity_scaling_factor
            self.moveit2.max_acceleration_scaling_factor = (
                self.acceleration_scaling_factor
            )

        except ImportError:
            self.get_logger().warn(
                "pymoveit2 package not available, using alternative approach"
            )
            self.moveit2 = None

        self.get_logger().info(
            f"MoveToPoseNode initialized with group: {self.move_group_name}, "
            f"end effector: {self.end_effector_link}, "
            f"velocity scaling: {self.velocity_scaling_factor}, "
            f"acceleration scaling: {self.acceleration_scaling_factor}"
        )

    def plan_to_pose(self, x, y, z, ox, oy, oz, ow):
        """规划到指定姿态的路径"""
        if self.moveit2 is not None:
            # 设置目标姿态
            self.moveit2.set_pose_goal(position=[x, y, z], quat_xyzw=[ox, oy, oz, ow])

            # 只规划，不执行
            plan = self.moveit2.plan()

            if plan is not None:
                self.get_logger().info("Successfully planned path to target pose")
                return True, plan
            else:
                self.get_logger().error("Failed to plan path to target pose")
                return False, None
        else:
            self.get_logger().error("MoveIt2 interface not available")
            return False, None

    def execute_plan(self, plan):
        """执行给定的规划"""
        if self.moveit2 is not None and plan is not None:
            # 执行规划
            self.moveit2.execute(plan)

            # 等待结果
            success = self.moveit2.wait_until_executed()

            if success:
                self.get_logger().info("Successfully executed planned path")
                return True
            else:
                self.get_logger().error("Failed to execute planned path")
                return False
        else:
            self.get_logger().error("MoveIt2 interface not available or plan is None")
            return False

    def move_to_pose_with_planning(self, x, y, z, ox, oy, oz, ow):
        """移动到指定姿态，包括规划和执行"""
        success, plan = self.plan_to_pose(x, y, z, ox, oy, oz, ow)

        if success and plan is not None:
            return self.execute_plan(plan)
            self.get_logger().info("Successfully plan_to_pose ")
            return True

        else:
            return False

    def move_to_named_pose(self, pose_name):
        """移动到命名的姿态（如果定义了的话）"""
        if self.moveit2 is not None:
            # pymoveit2可能不直接支持命名姿态，需要手动实现
            self.get_logger().warn(
                "Named poses not directly supported in this implementation"
            )
            return False
        else:
            self.get_logger().error("MoveIt2 interface not available")
            return False

    def get_current_pose(self):
        """获取当前末端执行器的姿态"""
        # 这个功能需要根据实际实现来完成
        self.get_logger().warn("Getting current pose not implemented in this version")
        return None

    def set_velocity_scaling(self, velocity_scaling_factor):
        """设置速度缩放因子"""
        if self.moveit2 is not None:
            self.moveit2.max_velocity_scaling_factor = velocity_scaling_factor
            self.velocity_scaling_factor = velocity_scaling_factor
            self.get_logger().info(
                f"Velocity scaling factor set to: {velocity_scaling_factor}"
            )
        else:
            self.get_logger().error("MoveIt2 interface not available")


def main(args=None):
    rclpy.init(args=args)

    try:
        node = MoveToPoseNode()

        # 示例：移动到一个更安全的位置 - 机械臂前方
        for _ in range(10):
            node.get_logger().info(
                "Moving to specific pose with planning and execution..."
            )
            success = node.move_to_pose_with_planning(0.1, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0)

            if success:
                node.get_logger().info("x0.1 Successfully moved to pose!")
            else:
                node.get_logger().error("x0.1 Failed to move to pose!")
            time.sleep(1.0)

            success = node.move_to_pose_with_planning(0.1, 0.1, 0.3, 0.0, 0.0, 0.0, 1.0)
            if success:
                node.get_logger().info("y0.1 Successfully moved to pose!")
            else:
                node.get_logger().error("y0.1  Failed to move to pose!")
            time.sleep(1.0)

        # 获取当前姿态
        current_pose = node.get_current_pose()
        if current_pose:
            node.get_logger().info(
                f"Current pose: pos=({current_pose.position.x}, {current_pose.position.y}, {current_pose.position.z}), "
                f"quat=({current_pose.orientation.x}, {current_pose.orientation.y}, "
                f"{current_pose.orientation.z}, {current_pose.orientation.w})"
            )

        rclpy.spin(node)
        node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
