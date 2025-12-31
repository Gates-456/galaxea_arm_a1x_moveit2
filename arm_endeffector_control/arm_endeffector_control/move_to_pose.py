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

        # 获取参数
        self.move_group_name = self.get_parameter("move_group_name").value
        self.end_effector_link = self.get_parameter("end_effector_link").value
        self.use_sim_time = self.get_parameter("use_sim_time").value

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

        except ImportError:
            self.get_logger().warn(
                "pymoveit2 package not available, using alternative approach"
            )
            self.moveit2 = None

        self.get_logger().info(
            f"MoveToPoseNode initialized with group: {self.move_group_name}, "
            f"end effector: {self.end_effector_link}"
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

    def plan_and_execute_to_pose(self, x, y, z, ox, oy, oz, ow):
        """规划并执行到指定姿态"""
        success, plan = self.plan_to_pose(x, y, z, ox, oy, oz, ow)

        if success and plan is not None:
            return self.execute_plan(plan)
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
        if self.moveit2 is not None:
            try:
                # 使用compute_fk方法获取当前末端执行器姿态
                current_joint_state = self.moveit2.joint_state
                if current_joint_state is not None:
                    # 使用当前关节状态计算正向运动学
                    pose_result = self.moveit2.compute_fk(
                        joint_state=current_joint_state,
                        fk_link_names=[self.end_effector_link],
                    )

                    if pose_result is not None:
                        # compute_fk可能返回单个PoseStamped或PoseStamped列表
                        if isinstance(pose_result, list):
                            if len(pose_result) > 0:
                                current_pose = pose_result[0].pose
                            else:
                                self.get_logger().warn(
                                    "FK computation returned an empty list"
                                )
                                return None
                        else:
                            # 假设返回的是单个PoseStamped
                            current_pose = pose_result.pose

                        self.get_logger().info(
                            f"Current pose retrieved: pos=({current_pose.position.x:.3f}, {current_pose.position.y:.3f}, {current_pose.position.z:.3f}), "
                            f"quat=({current_pose.orientation.x:.3f}, {current_pose.orientation.y:.3f}, "
                            f"{current_pose.orientation.z:.3f}, {current_pose.orientation.w:.3f})"
                        )
                        return current_pose
                    else:
                        self.get_logger().warn(
                            "Could not compute FK for current joint state"
                        )
                        return None
                else:
                    self.get_logger().warn("Joint states are not available yet")
                    return None
            except Exception as e:
                self.get_logger().error(f"Error getting current pose: {str(e)}")
                return None
        else:
            self.get_logger().error("MoveIt2 interface not available")
            return None


def main(args=None):
    rclpy.init(args=args)

    try:
        node = MoveToPoseNode()

        node.get_logger().info("MoveToPoseNode started.")

        # 等待一段时间确保关节状态可用
        node.get_logger().info("Waiting for joint states to be available...")
        timeout = 10  # 等待10秒
        for _ in range(timeout):
            if node.moveit2.joint_state is not None:
                node.get_logger().info("Joint states are available now")
                break
            node.get_logger().info("Waiting for joint states...")
            rclpy.spin_once(node, timeout_sec=1)
        else:
            node.get_logger().error(
                "Joint states are still not available after waiting!"
            )

        # 获取当前姿态
        current_pose = node.get_current_pose()
        if current_pose:
            node.get_logger().info(
                f"Initial pose: pos=({current_pose.position.x:.3f}, {current_pose.position.y:.3f}, {current_pose.position.z:.3f}), "
                f"quat=({current_pose.orientation.x:.3f}, {current_pose.orientation.y:.3f}, "
                f"{current_pose.orientation.z:.3f}, {current_pose.orientation.w:.3f})"
            )
        else:
            node.get_logger().warn("Could not get initial pose")

        # 示例：规划并执行到特定位置
        # 使用一个相对安全的初始位置，避免规划失败
        # node.get_logger().info("Planning and executing to specific pose...")
        # # 使用一个相对安全的位置作为示例
        # success = node.plan_and_execute_to_pose(0.3, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0)

        # if success:
        #     node.get_logger().info("Successfully moved to pose!")
        # else:
        #     node.get_logger().error("Failed to move to pose!")

        # 示例：只规划不执行
        node.get_logger().info("Planning only to another pose (not executing)...")
        plan_success, plan = node.plan_to_pose(0.3, 0.1, 0.3, 0.0, 0.0, 0.0, 1.0)

        if plan_success and plan is not None:
            node.get_logger().info("Successfully planned path, not executing yet.")
        else:
            node.get_logger().error("Failed to plan path!")

        # 再次获取当前姿态
        current_pose = node.get_current_pose()
        if current_pose:
            node.get_logger().info(
                f"Final pose: pos=({current_pose.position.x:.3f}, {current_pose.position.y:.3f}, {current_pose.position.z:.3f}), "
                f"quat=({current_pose.orientation.x:.3f}, {current_pose.orientation.y:.3f}, "
                f"{current_pose.orientation.z:.3f}, {current_pose.orientation.w:.3f})"
            )

        rclpy.spin(node)
        node.destroy_node()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
