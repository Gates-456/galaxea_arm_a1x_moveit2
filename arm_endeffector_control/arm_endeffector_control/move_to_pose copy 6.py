#!/usr/bin/env python3

"""
Move to pose node - 增强版本：增加执行验证和重试机制
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from std_msgs.msg import Header
from tf2_ros import TransformException, Buffer, TransformListener
import threading
import time
import queue
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import math

# MoveIt2 相关消息和服务
from moveit_msgs.srv import GetCartesianPath, GetMotionPlan
from moveit_msgs.msg import (
    RobotState,
    Constraints,
    MotionPlanRequest,
    MotionPlanResponse,
    MotionPlanDetailedResponse,
    PositionConstraint,
    OrientationConstraint,
)
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from moveit_msgs.action import ExecuteTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from shape_msgs.msg import SolidPrimitive


class EnhancedMoveToPoseNode(Node):
    def __init__(self):
        super().__init__("move_to_pose_node")

        # 声明参数
        self.declare_parameter("move_group_name", "a1x_group")
        self.declare_parameter("end_effector_link", "arm_link6")
        self.declare_parameter("velocity_scaling", 0.2)  # 进一步降低速度以提高精度
        self.declare_parameter("acceleration_scaling", 0.2)
        self.declare_parameter("cartesian_step_size", 0.002)  # 更小的步长
        self.declare_parameter("planning_time", 15.0)  # 增加规划时间
        self.declare_parameter("num_planning_attempts", 10)  # 更多尝试
        self.declare_parameter("position_tolerance", 0.01)  # 位置精度0.01米
        self.declare_parameter("orientation_tolerance", 0.05)  # 姿态精度0.05弧度
        self.declare_parameter("max_execution_time", 10.0)  # 最大执行时间
        self.declare_parameter("position_check_interval", 0.5)  # 位置检查间隔
        self.declare_parameter("max_retries", 3)  # 最大重试次数
        self.declare_parameter("use_cartesian_fallback", True)  # 使用笛卡尔回退
        self.declare_parameter("planner_id", "RRTConnect")  # 指定规划器

        # 获取参数
        self.move_group_name = self.get_parameter("move_group_name").value
        self.end_effector_link = self.get_parameter("end_effector_link").value
        self.velocity_scaling = self.get_parameter("velocity_scaling").value
        self.acceleration_scaling = self.get_parameter("acceleration_scaling").value
        self.cartesian_step_size = self.get_parameter("cartesian_step_size").value
        self.planning_time = self.get_parameter("planning_time").value
        self.num_planning_attempts = self.get_parameter("num_planning_attempts").value
        self.position_tolerance = self.get_parameter("position_tolerance").value
        self.orientation_tolerance = self.get_parameter("orientation_tolerance").value
        self.max_execution_time = self.get_parameter("max_execution_time").value
        self.position_check_interval = self.get_parameter(
            "position_check_interval"
        ).value
        self.max_retries = self.get_parameter("max_retries").value
        self.use_cartesian_fallback = self.get_parameter("use_cartesian_fallback").value
        self.planner_id = self.get_parameter("planner_id").value

        self.get_logger().info(f"参数设置:")
        self.get_logger().info(f"  - 运动组: {self.move_group_name}")
        self.get_logger().info(f"  - 末端连杆: {self.end_effector_link}")
        self.get_logger().info(f"  - 速度缩放: {self.velocity_scaling}")
        self.get_logger().info(f"  - 位置精度: {self.position_tolerance}m")
        self.get_logger().info(f"  - 最大重试: {self.max_retries}")
        self.get_logger().info(f"  - 规划器: {self.planner_id}")

        # 使用重入回调组
        self.callback_group = ReentrantCallbackGroup()

        # 初始化 TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 关节状态管理
        self.joint_state_lock = threading.Lock()
        self.current_joint_state = None
        self.last_joint_state_time = 0.0
        self.joint_names = [
            "arm_joint1",
            "arm_joint2",
            "arm_joint3",
            "arm_joint4",
            "arm_joint5",
            "arm_joint6",
        ]

        # 系统状态
        self.tf_ready = False
        self.joints_ready = False

        # 创建服务客户端
        self.get_logger().info("创建服务客户端...")

        # 创建笛卡尔路径规划客户端
        self.cartesian_path_client = self.create_client(
            GetCartesianPath,
            "/compute_cartesian_path",
            callback_group=self.callback_group,
        )

        # 创建运动规划服务客户端（用于关节空间规划）
        self.motion_plan_client = self.create_client(
            GetMotionPlan,
            "/plan_kinematic_path",
            callback_group=self.callback_group,
        )

        # 创建执行轨迹的 Action 客户端
        self.execute_action_client = ActionClient(
            self,
            ExecuteTrajectory,
            "/execute_trajectory",
            callback_group=self.callback_group,
        )

        # 等待服务
        self.get_logger().info("等待服务可用...")
        self.wait_for_services()

        # 创建订阅者
        self.create_subscriptions()

        # 创建发布者
        self.end_effector_pose_pub = self.create_publisher(
            PoseStamped, "/end_effector_pose", 10
        )

        # 创建定时器
        self.status_timer = self.create_timer(
            1.0, self.monitor_status, callback_group=self.callback_group
        )

        self.pose_timer = self.create_timer(
            0.1, self.publish_current_pose, callback_group=self.callback_group
        )

        self.init_timer = self.create_timer(
            0.5, self.check_initialization, callback_group=self.callback_group
        )

        # 任务队列
        self.task_queue = queue.Queue(maxsize=10)
        self.task_processing = False
        self.task_thread = threading.Thread(
            target=self.process_tasks, daemon=True, name="TaskProcessor"
        )
        self.task_thread.start()

        # 统计信息
        self.stats = {
            "success": 0,
            "failed": 0,
            "total": 0,
            "cartesian_success": 0,
            "joint_space_success": 0,
            "retries": 0,
        }

        self.get_logger().info("节点初始化完成，等待系统就绪...")

    def wait_for_services(self):
        """等待所需服务可用"""
        services_ready = False
        start_time = time.time()
        max_wait_time = 60.0  # 增加等待时间

        while (
            rclpy.ok()
            and not services_ready
            and time.time() - start_time < max_wait_time
        ):
            services_ready = True

            # 检查笛卡尔路径服务
            if not self.cartesian_path_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn("等待笛卡尔路径服务...")
                services_ready = False

            # 检查运动规划服务
            if not self.motion_plan_client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn("等待运动规划服务...")
                services_ready = False

            # 检查执行轨迹服务
            if not self.execute_action_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().warn("等待执行轨迹服务...")
                services_ready = False

        if services_ready:
            self.get_logger().info("✅ 所有服务已就绪!")
        else:
            self.get_logger().error("❌ 服务等待超时!")

        return services_ready

    def create_subscriptions(self):
        """创建所有订阅者"""
        # 关节状态订阅
        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
            callback_group=self.callback_group,
        )

        # 目标姿态订阅
        self.target_pose_sub = self.create_subscription(
            PoseStamped,
            "/target_end_effector_pose",
            self.target_pose_callback,
            10,
            callback_group=self.callback_group,
        )

        # 笛卡尔移动订阅
        self.cartesian_move_sub = self.create_subscription(
            PoseStamped,
            "/cartesian_linear_move",
            self.cartesian_move_callback,
            10,
            callback_group=self.callback_group,
        )

        # Z轴移动订阅
        self.z_move_sub = self.create_subscription(
            PoseStamped,
            "/z_axis_linear_move",
            self.z_move_callback,
            10,
            callback_group=self.callback_group,
        )

    def check_initialization(self):
        """检查系统初始化状态"""
        try:
            # 检查 TF
            if not self.tf_ready:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        "base_link",
                        self.end_effector_link,
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.1),
                    )
                    self.tf_ready = True
                    self.get_logger().info("✅ TF 系统已就绪")
                except (TransformException, Exception):
                    # 首次运行可能还不可用，正常
                    pass

            # 检查关节状态
            if not self.joints_ready:
                with self.joint_state_lock:
                    if self.current_joint_state is not None:
                        self.joints_ready = True
                        self.get_logger().info("✅ 关节状态已就绪")

            # 如果都准备好了，取消初始化定时器
            if self.tf_ready and self.joints_ready:
                self.destroy_timer(self.init_timer)
                self.get_logger().info("🎉 系统完全就绪，可以开始运动控制!")

        except Exception as e:
            self.get_logger().debug(f"初始化检查: {str(e)}")

    def joint_state_callback(self, msg):
        """关节状态回调"""
        try:
            # 创建新的关节状态消息，确保顺序正确
            ordered_joint_state = JointState()
            ordered_joint_state.header = msg.header

            # 按照期望的顺序排列关节
            for joint_name in self.joint_names:
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    ordered_joint_state.name.append(joint_name)
                    ordered_joint_state.position.append(msg.position[idx])
                    if msg.velocity:
                        ordered_joint_state.velocity.append(msg.velocity[idx])
                    if msg.effort:
                        ordered_joint_state.effort.append(msg.effort[idx])

            with self.joint_state_lock:
                self.current_joint_state = ordered_joint_state
                self.last_joint_state_time = time.time()

        except Exception as e:
            self.get_logger().error(f"关节状态回调错误: {str(e)}")

    def get_robot_state(self):
        """获取当前机器人状态"""
        with self.joint_state_lock:
            if self.current_joint_state is None:
                # 返回默认状态
                robot_state = RobotState()
                robot_state.joint_state = JointState()
                robot_state.joint_state.name = self.joint_names
                robot_state.joint_state.position = [0.0] * len(self.joint_names)
                robot_state.is_diff = True
                return robot_state

            # 使用当前关节状态
            robot_state = RobotState()
            robot_state.joint_state = self.current_joint_state
            robot_state.is_diff = True
            return robot_state

    def get_current_pose(self):
        """获取当前末端执行器位姿"""
        try:
            now = self.get_clock().now()

            transform = self.tf_buffer.lookup_transform(
                "base_link",
                self.end_effector_link,
                now,
                timeout=rclpy.duration.Duration(seconds=0.5),
            )

            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation

            return pose

        except TransformException as e:
            self.get_logger().debug(f"TF 获取失败: {str(e)}")
            return None
        except Exception as e:
            self.get_logger().error(f"获取当前姿态异常: {str(e)}")
            return None

    def compute_cartesian_path(self, waypoints):
        """计算笛卡尔路径（用于直线移动）"""
        try:
            # 创建请求
            request = GetCartesianPath.Request()
            request.header.stamp = self.get_clock().now().to_msg()
            request.header.frame_id = "base_link"
            request.start_state = self.get_robot_state()
            request.group_name = self.move_group_name
            request.link_name = self.end_effector_link
            request.waypoints = waypoints
            request.max_step = self.cartesian_step_size
            request.jump_threshold = 0.0
            request.avoid_collisions = True
            request.path_constraints = Constraints()

            self.get_logger().info(
                f"计算笛卡尔路径到: ({waypoints[0].position.x:.3f}, {waypoints[0].position.y:.3f}, {waypoints[0].position.z:.3f})"
            )

            # 调用服务
            future = self.cartesian_path_client.call_async(request)

            # 等待响应
            start_time = time.time()
            while rclpy.ok() and not future.done():
                if time.time() - start_time > 5.0:
                    self.get_logger().error("规划超时")
                    return None, 0.0
                time.sleep(0.01)

            if future.done():
                response = future.result()
                if response:
                    self.get_logger().info(
                        f"笛卡尔规划完成，覆盖率: {response.fraction*100:.1f}%"
                    )
                    return response.solution, response.fraction
                else:
                    self.get_logger().error("笛卡尔服务调用失败")
                    return None, 0.0
            else:
                self.get_logger().error("笛卡尔规划未完成")
                return None, 0.0

        except Exception as e:
            self.get_logger().error(f"笛卡尔路径计算异常: {str(e)}")
            return None, 0.0

    def plan_to_pose_direct(self, pose_stamped):
        """直接规划到目标姿态（使用关节空间规划，设置精确约束）"""
        try:
            # 创建运动规划请求
            request = GetMotionPlan.Request()

            # 设置规划请求
            motion_request = MotionPlanRequest()
            motion_request.workspace_parameters.header.frame_id = "base_link"
            motion_request.workspace_parameters.min_corner.x = -2.0
            motion_request.workspace_parameters.min_corner.y = -2.0
            motion_request.workspace_parameters.min_corner.z = -2.0
            motion_request.workspace_parameters.max_corner.x = 2.0
            motion_request.workspace_parameters.max_corner.y = 2.0
            motion_request.workspace_parameters.max_corner.z = 2.0

            # 设置起始状态
            motion_request.start_state = self.get_robot_state()

            # 创建精确的位置约束（使用立方体区域，而不是球体）
            position_constraint = PositionConstraint()
            position_constraint.header = pose_stamped.header
            position_constraint.link_name = self.end_effector_link

            # 创建一个小的立方体作为约束区域
            box = SolidPrimitive()
            box.type = SolidPrimitive.BOX
            box.dimensions = [
                self.position_tolerance,  # X方向容差
                self.position_tolerance,  # Y方向容差
                self.position_tolerance,  # Z方向容差
            ]
            position_constraint.constraint_region.primitives.append(box)

            # 将目标姿态作为约束区域的中心
            from geometry_msgs.msg import Pose

            region_pose = Pose()
            region_pose.position = pose_stamped.pose.position
            region_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            position_constraint.constraint_region.primitive_poses.append(region_pose)

            position_constraint.weight = 1.0

            # 创建姿态约束
            orientation_constraint = OrientationConstraint()
            orientation_constraint.header = pose_stamped.header
            orientation_constraint.link_name = self.end_effector_link
            orientation_constraint.orientation = pose_stamped.pose.orientation
            orientation_constraint.absolute_x_axis_tolerance = (
                self.orientation_tolerance
            )
            orientation_constraint.absolute_y_axis_tolerance = (
                self.orientation_tolerance
            )
            orientation_constraint.absolute_z_axis_tolerance = (
                self.orientation_tolerance
            )
            orientation_constraint.weight = 1.0

            # 创建约束
            constraints = Constraints()
            constraints.name = "precise_pose_constraint"
            constraints.position_constraints.append(position_constraint)
            constraints.orientation_constraints.append(orientation_constraint)

            motion_request.goal_constraints.append(constraints)

            # 设置其他参数
            motion_request.group_name = self.move_group_name
            motion_request.num_planning_attempts = self.num_planning_attempts
            motion_request.allowed_planning_time = self.planning_time
            motion_request.max_velocity_scaling_factor = self.velocity_scaling
            motion_request.max_acceleration_scaling_factor = self.acceleration_scaling
            motion_request.pipeline_id = ""  # 使用默认规划器
            motion_request.planner_id = self.planner_id  # 使用指定规划器

            request.motion_plan_request = motion_request

            self.get_logger().info("使用直接关节空间规划...")

            # 调用服务
            future = self.motion_plan_client.call_async(request)

            # 等待响应
            start_time = time.time()
            while rclpy.ok() and not future.done():
                if time.time() - start_time > self.planning_time + 2.0:
                    self.get_logger().error("关节空间规划超时")
                    return None
                time.sleep(0.01)

            if future.done():
                response = future.result()
                if response:
                    if response.motion_plan_response.error_code.val == 1:  # SUCCESS
                        self.get_logger().info("✅ 关节空间规划成功")
                        return response.motion_plan_response.trajectory
                    else:
                        self.get_logger().error(
                            f"关节空间规划失败，错误码: {response.motion_plan_response.error_code.val}"
                        )
                        return None
                else:
                    self.get_logger().error("关节空间规划服务调用失败")
                    return None
            else:
                self.get_logger().error("关节空间规划未完成")
                return None

        except Exception as e:
            self.get_logger().error(f"直接关节空间规划异常: {str(e)}")
            return None

    def execute_trajectory_with_verification(self, trajectory, target_pose=None):
        """执行轨迹并进行验证"""
        if trajectory is None:
            self.get_logger().error("轨迹为空")
            return False

        try:
            # 获取执行前的位姿（用于参考）
            initial_pose = self.get_current_pose()

            # 创建目标消息
            goal_msg = ExecuteTrajectory.Goal()
            goal_msg.trajectory = trajectory

            # 发送目标
            future = self.execute_action_client.send_goal_async(goal_msg)

            # 等待结果
            start_time = time.time()
            while rclpy.ok() and not future.done():
                if time.time() - start_time > self.max_execution_time:
                    self.get_logger().error("执行超时")
                    return False
                time.sleep(0.01)

            if future.done():
                goal_handle = future.result()
                if goal_handle.accepted:
                    self.get_logger().info("轨迹执行已接受")

                    # 等待执行完成并检查位置
                    if target_pose is not None:
                        time.sleep(1.0)  # 给机械臂一些时间稳定
                        return self.wait_for_position(target_pose)
                    else:
                        return True
                else:
                    self.get_logger().error("轨迹执行被拒绝")
                    return False
            else:
                self.get_logger().error("执行未完成")
                return False

        except Exception as e:
            self.get_logger().error(f"轨迹执行异常: {str(e)}")
            return False

    def wait_for_position(self, target_pose, timeout=10.0):
        """等待到达目标位置"""
        start_time = time.time()

        while rclpy.ok() and time.time() - start_time < timeout:
            # 获取当前位置
            current_pose = self.get_current_pose()
            if current_pose is None:
                time.sleep(0.1)
                continue

            # 计算位置误差
            pos_error = math.sqrt(
                (current_pose.position.x - target_pose.position.x) ** 2
                + (current_pose.position.y - target_pose.position.y) ** 2
                + (current_pose.position.z - target_pose.position.z) ** 2
            )

            # 如果误差在容差范围内，返回成功
            if pos_error <= self.position_tolerance:
                self.get_logger().info(
                    f"✅ 位置精度达标: {pos_error:.4f}m (目标: {self.position_tolerance}m)"
                )
                return True

            # 定期报告位置
            if time.time() - start_time > 1.0 and (time.time() - start_time) % 2 < 0.1:
                self.get_logger().info(
                    f"当前位置: ({current_pose.position.x:.3f}, {current_pose.position.y:.3f}, {current_pose.position.z:.3f}), "
                    f"误差: {pos_error:.4f}m"
                )

            time.sleep(self.position_check_interval)

        # 超时，获取最终位置
        final_pose = self.get_current_pose()
        if final_pose:
            final_error = math.sqrt(
                (final_pose.position.x - target_pose.position.x) ** 2
                + (final_pose.position.y - target_pose.position.y) ** 2
                + (final_pose.position.z - target_pose.position.z) ** 2
            )
            self.get_logger().warn(
                f"等待超时，当前位置: ({final_pose.position.x:.3f}, {final_pose.position.y:.3f}, {final_pose.position.z:.3f}), "
                f"距离: {final_error:.4f}, 目标: {self.position_tolerance}"
            )

        return False

    def move_to_pose_direct(self, pose_stamped, retry_count=0):
        """直接移动到指定姿态（带重试机制）"""
        try:
            # 提取姿态
            target_pose = pose_stamped.pose
            self.get_logger().info(
                f"直接移动到姿态: ({target_pose.position.x:.3f}, {target_pose.position.y:.3f}, {target_pose.position.z:.3f})"
            )
            self.get_logger().info(
                f"位置精度: {self.position_tolerance}m, 姿态精度: {self.orientation_tolerance}rad"
            )

            # 使用直接关节空间规划
            trajectory = self.plan_to_pose_direct(pose_stamped)

            if trajectory is not None:
                # 执行轨迹并进行验证
                success = self.execute_trajectory_with_verification(
                    trajectory, target_pose
                )

                if success:
                    self.stats["joint_space_success"] += 1
                    self.stats["success"] += 1
                    self.get_logger().info("✅ 直接运动完成")
                else:
                    # 如果失败且重试次数未达到上限，则重试
                    if retry_count < self.max_retries:
                        self.stats["retries"] += 1
                        self.get_logger().warn(f"第{retry_count+1}次重试...")
                        time.sleep(0.5)  # 短暂延迟后重试
                        return self.move_to_pose_direct(pose_stamped, retry_count + 1)
                    else:
                        # 如果重试失败且配置了回退，尝试笛卡尔规划
                        if self.use_cartesian_fallback:
                            self.get_logger().warn(
                                "关节空间规划失败，尝试笛卡尔回退..."
                            )
                            return self.move_to_pose_cartesian(pose_stamped)
                        else:
                            self.stats["failed"] += 1
                            self.get_logger().error("❌ 直接运动执行失败")

                self.stats["total"] += 1
                return success
            else:
                self.get_logger().error("❌ 直接规划失败")
                # 如果规划失败且重试次数未达到上限，则重试
                if retry_count < self.max_retries:
                    self.stats["retries"] += 1
                    self.get_logger().warn(f"第{retry_count+1}次重试...")
                    time.sleep(1.0)  # 更长延迟后重试
                    return self.move_to_pose_direct(pose_stamped, retry_count + 1)
                else:
                    self.stats["failed"] += 1
                    self.stats["total"] += 1
                    return False

        except Exception as e:
            self.get_logger().error(f"直接移动到姿态异常: {str(e)}")
            self.stats["failed"] += 1
            self.stats["total"] += 1
            return False

    def move_to_pose_cartesian(self, pose_stamped):
        """使用笛卡尔规划移动到目标姿态"""
        try:
            # 提取姿态
            target_pose = pose_stamped.pose
            self.get_logger().info(
                f"尝试笛卡尔规划到姿态: ({target_pose.position.x:.3f}, {target_pose.position.y:.3f}, {target_pose.position.z:.3f})"
            )

            # 计算笛卡尔路径
            trajectory, fraction = self.compute_cartesian_path([target_pose])

            if trajectory is None or fraction < 0.95:
                self.get_logger().error(f"笛卡尔规划失败，覆盖率: {fraction*100:.1f}%")
                self.stats["failed"] += 1
                self.stats["total"] += 1
                return False

            # 执行轨迹
            success = self.execute_trajectory_with_verification(trajectory, target_pose)

            if success:
                self.stats["cartesian_success"] += 1
                self.stats["success"] += 1
                self.get_logger().info("✅ 笛卡尔运动完成")
            else:
                self.stats["failed"] += 1
                self.get_logger().error("❌ 笛卡尔运动执行失败")

            self.stats["total"] += 1
            return success

        except Exception as e:
            self.get_logger().error(f"笛卡尔移动到姿态异常: {str(e)}")
            self.stats["failed"] += 1
            self.stats["total"] += 1
            return False

    def linear_move_cartesian(self, x_offset, y_offset, z_offset):
        """笛卡尔直线移动（使用笛卡尔规划）"""
        try:
            # 获取当前姿态
            current_pose = self.get_current_pose()
            if current_pose is None:
                self.get_logger().error("无法获取当前姿态")
                return False

            # 计算目标姿态
            target_pose = Pose()
            target_pose.position.x = current_pose.position.x + x_offset
            target_pose.position.y = current_pose.position.y + y_offset
            target_pose.position.z = current_pose.position.z + z_offset
            target_pose.orientation = current_pose.orientation

            self.get_logger().info(
                f"笛卡尔线性移动: Δ({x_offset:.3f}, {y_offset:.3f}, {z_offset:.3f})"
            )

            # 创建PoseStamped
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose = target_pose

            # 使用笛卡尔规划
            return self.move_to_pose_cartesian(pose_stamped)

        except Exception as e:
            self.get_logger().error(f"笛卡尔线性移动异常: {str(e)}")
            self.stats["failed"] += 1
            self.stats["total"] += 1
            return False

    def z_axis_move(self, z_distance):
        """沿Z轴移动（使用笛卡尔规划）"""
        return self.linear_move_cartesian(0.0, 0.0, z_distance)

    def process_tasks(self):
        """处理任务队列"""
        while rclpy.ok():
            try:
                # 获取任务
                task = self.task_queue.get(timeout=0.1)

                if task is None:
                    continue

                # 处理任务
                task_type = task.get("type")
                task_data = task.get("data")

                if task_type == "target_pose":
                    success = self.move_to_pose_direct(task_data)  # 修改为直接移动
                elif task_type == "cartesian_move":
                    success = self.linear_move_cartesian(*task_data)
                elif task_type == "z_move":
                    success = self.z_axis_move(task_data)
                else:
                    self.get_logger().warn(f"未知任务类型: {task_type}")
                    success = False

                # 标记任务完成
                self.task_queue.task_done()

                # 短暂延迟，避免过载
                time.sleep(0.1)

            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f"任务处理异常: {str(e)}")
                time.sleep(0.1)

    def target_pose_callback(self, msg):
        """目标姿态回调"""
        self.get_logger().info(
            f"收到目标姿态: ({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})"
        )

        # 确保消息有正确的frame_id
        if msg.header.frame_id == "":
            msg.header.frame_id = "base_link"
            msg.header.stamp = self.get_clock().now().to_msg()

        # 添加到任务队列
        try:
            self.task_queue.put_nowait({"type": "target_pose", "data": msg})
        except queue.Full:
            self.get_logger().warn("任务队列已满，丢弃任务")

    def cartesian_move_callback(self, msg):
        """笛卡尔移动回调"""
        x_offset = msg.pose.position.x
        y_offset = msg.pose.position.y
        z_offset = msg.pose.position.z

        self.get_logger().info(
            f"收到笛卡尔移动: ({x_offset:.3f}, {y_offset:.3f}, {z_offset:.3f})"
        )

        try:
            self.task_queue.put_nowait(
                {"type": "cartesian_move", "data": (x_offset, y_offset, z_offset)}
            )
        except queue.Full:
            self.get_logger().warn("任务队列已满，丢弃任务")

    def z_move_callback(self, msg):
        """Z轴移动回调"""
        z_distance = msg.pose.position.z

        self.get_logger().info(f"收到Z轴移动: {z_distance:.3f}")

        try:
            self.task_queue.put_nowait({"type": "z_move", "data": z_distance})
        except queue.Full:
            self.get_logger().warn("任务队列已满，丢弃任务")

    def publish_current_pose(self):
        """发布当前末端执行器位姿"""
        try:
            pose = self.get_current_pose()
            if pose is not None:
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = self.get_clock().now().to_msg()
                pose_stamped.header.frame_id = "base_link"
                pose_stamped.pose = pose

                self.end_effector_pose_pub.publish(pose_stamped)

        except Exception as e:
            self.get_logger().debug(f"发布姿态异常: {str(e)}")

    def monitor_status(self):
        """监控状态"""
        try:
            # 检查关节状态新鲜度
            joint_freshness = 0.0
            with self.joint_state_lock:
                if self.current_joint_state:
                    joint_freshness = time.time() - self.last_joint_state_time

            # 状态信息
            tf_status = "✅" if self.tf_ready else "❌"
            joints_status = "✅" if self.joints_ready else "❌"

            total_success = self.stats["success"]
            total_failed = self.stats["failed"]
            total = self.stats["total"]
            success_rate = (total_success / total * 100) if total > 0 else 0

            status_msg = (
                f"状态: TF={tf_status}, 关节={joints_status} | "
                f"队列: {self.task_queue.qsize()} | "
                f"关节新鲜度: {joint_freshness:.1f}s | "
                f"成功率: {success_rate:.1f}% ({total_success}/{total}) | "
                f"直接移动: {self.stats['joint_space_success']}, 笛卡尔: {self.stats['cartesian_success']} | "
                f"重试: {self.stats['retries']}"
            )

            # 只在需要时打印状态
            if (
                not self.tf_ready
                or not self.joints_ready
                or self.task_queue.qsize() > 0
            ):
                self.get_logger().info(status_msg)
            elif time.time() % 10 < 1:  # 每10秒打印一次
                self.get_logger().info(status_msg)

        except Exception as e:
            self.get_logger().debug(f"状态监控异常: {str(e)}")


def main(args=None):
    rclpy.init(args=args)

    try:
        # 创建节点
        node = EnhancedMoveToPoseNode()

        # 使用多线程执行器
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)

        # 运行节点
        executor.spin()

    except KeyboardInterrupt:
        print("\n收到中断信号，关闭节点...")
    except Exception as e:
        print(f"节点异常: {str(e)}")
    finally:
        try:
            if "node" in locals():
                node.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
