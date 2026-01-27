#!/usr/bin/env python3

"""
Move to pose node - 深度优化TF查询策略
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
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from concurrent.futures import ThreadPoolExecutor, as_completed
import logging

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
    BoundingVolume,
    JointConstraint,
    PlanningOptions,
    PlanningScene,
)
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from shape_msgs.msg import SolidPrimitive


class UltimateMoveToPoseNode(Node):
    def __init__(self):
        super().__init__("move_to_pose_node")

        # 声明参数
        self.declare_parameter("move_group_name", "a1x_group")
        self.declare_parameter("end_effector_link", "arm_link6")
        self.declare_parameter("velocity_scaling", 1.0)
        self.declare_parameter("acceleration_scaling", 1.0)
        self.declare_parameter("cartesian_step_size", 0.008)
        self.declare_parameter("planning_time", 5.0)
        self.declare_parameter("num_planning_attempts", 10)
        self.declare_parameter("position_tolerance", 0.005)
        self.declare_parameter("orientation_tolerance", 0.05)
        self.declare_parameter("use_execute_trajectory", True)
        self.declare_parameter("max_workers", 4)
        self.declare_parameter("task_queue_size", 5)
        self.declare_parameter("log_level", "info")
        self.declare_parameter("enable_perf_log", False)
        self.declare_parameter("tf_cache_time", 5.0)
        self.declare_parameter("joint_state_timeout", 1.0)
        self.declare_parameter("tf_retry_count", 2)  # 减少重试次数
        self.declare_parameter("tf_retry_delay", 0.02)  # 增加重试延迟
        self.declare_parameter("planning_retry_count", 3)
        self.declare_parameter("min_success_rate_log", 95.0)
        self.declare_parameter("tf_lookup_timeout", 0.01)  # TF查询超时
        self.declare_parameter("pose_publish_rate", 100.0)  # 姿态发布频率
        self.declare_parameter("status_monitor_rate", 100.0)  # 状态监控频率
        self.declare_parameter("use_async_tf", True)  # 使用异步TF查询
        self.declare_parameter("tf_use_latest", True)  # 使用最新TF数据

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
        self.use_execute_trajectory = self.get_parameter("use_execute_trajectory").value
        self.max_workers = self.get_parameter("max_workers").value
        self.task_queue_size = self.get_parameter("task_queue_size").value
        self.log_level = self.get_parameter("log_level").value
        self.enable_perf_log = self.get_parameter("enable_perf_log").value
        self.tf_cache_time = self.get_parameter("tf_cache_time").value
        self.joint_state_timeout = self.get_parameter("joint_state_timeout").value
        self.tf_retry_count = self.get_parameter("tf_retry_count").value
        self.tf_retry_delay = self.get_parameter("tf_retry_delay").value
        self.planning_retry_count = self.get_parameter("planning_retry_count").value
        self.min_success_rate_log = self.get_parameter("min_success_rate_log").value
        self.tf_lookup_timeout = self.get_parameter("tf_lookup_timeout").value
        self.pose_publish_rate = self.get_parameter("pose_publish_rate").value
        self.status_monitor_rate = self.get_parameter("status_monitor_rate").value
        self.use_async_tf = self.get_parameter("use_async_tf").value
        self.tf_use_latest = self.get_parameter("tf_use_latest").value

        # 设置日志级别
        self._setup_logging()

        # 记录参数
        self.get_logger().info("参数设置:")
        self.get_logger().info(f"  运动组: {self.move_group_name}")
        self.get_logger().info(f"  末端连杆: {self.end_effector_link}")
        self.get_logger().info(f"  速度缩放: {self.velocity_scaling}")
        self.get_logger().info(f"  加速度缩放: {self.acceleration_scaling}")
        self.get_logger().info(f"  位置容差: {self.position_tolerance}")
        self.get_logger().info(f"  姿态容差: {self.orientation_tolerance}")
        self.get_logger().info(f"  TF重试次数: {self.tf_retry_count}")
        self.get_logger().info(f"  TF查询超时: {self.tf_lookup_timeout}s")
        self.get_logger().info(f"  姿态发布频率: {self.pose_publish_rate}Hz")
        self.get_logger().info(f"  使用异步TF: {self.use_async_tf}")

        # 使用不同的回调组
        self.fast_callback_group = MutuallyExclusiveCallbackGroup()
        self.slow_callback_group = ReentrantCallbackGroup()

        # 初始化 TF
        self.tf_buffer = Buffer(
            cache_time=rclpy.duration.Duration(seconds=self.tf_cache_time)
        )
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 关节状态管理
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
        self.services_ready = False
        self.initialized = False
        self.shutting_down = False

        # TF状态缓存
        self.last_tf_success_time = 0.0
        self.tf_available = False
        self.cached_pose = None
        self.cached_pose_time = 0.0
        self.cached_pose_valid_for = 0.1  # 缓存姿态有效时间（秒）

        # 创建线程池
        self.thread_pool = ThreadPoolExecutor(
            max_workers=self.max_workers, thread_name_prefix="MoveItWorker"
        )

        # 任务队列
        self.task_queue = queue.Queue(maxsize=self.task_queue_size)
        self.pending_tasks = 0
        self.last_task_time = 0

        # 服务客户端
        self.cartesian_path_client = None
        self.motion_plan_client = None
        self.execute_action_client = None
        self.move_group_client = None

        # 统计信息
        self.stats = {
            "success": 0,
            "failed": 0,
            "total": 0,
            "target_pose_success": 0,
            "cartesian_success": 0,
            "queue_full": 0,
            "avg_response_time": 0.0,
            "max_response_time": 0.0,
            "min_response_time": float("inf"),
            "recent_success": 0,
            "recent_total": 0,
        }

        # 最近任务结果队列
        self.recent_results = queue.Queue(maxsize=100)

        # 性能监控
        self.perf_stats = {
            "tf_lookups": 0,
            "tf_failures": 0,
            "tf_success_rate": 0.0,
            "planning_time": [],
            "execution_time": [],
            "queue_wait_time": [],
            "tf_latency": [],
        }

        # 初始化关键组件
        self._setup_critical_components()

        self.get_logger().info("节点初始化完成")

    def _setup_logging(self):
        """设置日志级别"""
        # 根据参数设置日志级别
        log_levels = {
            "debug": rclpy.logging.LoggingSeverity.DEBUG,
            "info": rclpy.logging.LoggingSeverity.INFO,
            "warn": rclpy.logging.LoggingSeverity.WARN,
            "error": rclpy.logging.LoggingSeverity.ERROR,
        }

        if self.log_level in log_levels:
            rclpy.logging.set_logger_level(
                self.get_logger().name, log_levels[self.log_level]
            )

        # 初始化节流字典
        self._log_throttle = {}

    def log_throttled(self, level, message, key, interval=1.0):
        """节流日志输出"""
        current_time = time.time()
        if (
            key not in self._log_throttle
            or current_time - self._log_throttle[key] > interval
        ):
            self._log_throttle[key] = current_time
            if level == "debug":
                self.get_logger().debug(message)
            elif level == "info":
                self.get_logger().info(message)
            elif level == "warn":
                self.get_logger().warn(message)
            elif level == "error":
                self.get_logger().error(message)

    def _setup_critical_components(self):
        """设置关键组件"""
        # 创建订阅者
        self._create_subscriptions()

        # 创建发布者
        self.end_effector_pose_pub = self.create_publisher(
            PoseStamped, "/end_effector_pose", 10
        )

        # 启动初始化检查
        self.init_timer = self.create_timer(
            0.5, self._initialize_system, callback_group=self.fast_callback_group
        )

        # 启动状态监控
        status_interval = 1.0 / self.status_monitor_rate
        self.status_timer = self.create_timer(
            status_interval,
            self._monitor_status,
            callback_group=self.fast_callback_group,
        )

        # 启动姿态发布
        pose_interval = 1.0 / self.pose_publish_rate
        self.pose_timer = self.create_timer(
            pose_interval,
            self._publish_current_pose,
            callback_group=self.fast_callback_group,
        )

        # 启动任务处理器
        self.task_thread = threading.Thread(
            target=self._process_tasks, daemon=True, name="TaskProcessor"
        )
        self.task_thread.start()

    def _create_subscriptions(self):
        """创建订阅者"""
        # 关节状态订阅
        self.joint_state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_state_callback,
            10,
            callback_group=self.fast_callback_group,
        )

        # 目标姿态订阅
        self.target_pose_sub = self.create_subscription(
            PoseStamped,
            "/target_end_effector_pose",
            self._target_pose_callback,
            100,
            callback_group=self.fast_callback_group,
        )

        # 笛卡尔移动订阅
        self.cartesian_move_sub = self.create_subscription(
            PoseStamped,
            "/cartesian_linear_move",
            self._cartesian_move_callback,
            100,
            callback_group=self.fast_callback_group,
        )

        # Z轴移动订阅
        self.z_move_sub = self.create_subscription(
            PoseStamped,
            "/z_axis_linear_move",
            self._z_move_callback,
            100,
            callback_group=self.fast_callback_group,
        )

    def _joint_state_callback(self, msg):
        """关节状态回调"""
        try:
            # 快速提取关节位置
            positions = {}
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    positions[name] = msg.position[i]
                else:
                    positions[name] = 0.0

            # 创建有序关节状态
            joint_state = JointState()
            joint_state.header = msg.header
            joint_state.name = self.joint_names.copy()
            joint_state.position = [
                positions.get(name, 0.0) for name in self.joint_names
            ]

            # 原子更新
            self.current_joint_state = joint_state
            self.last_joint_state_time = time.time()

            if not self.joints_ready:
                self.joints_ready = True
                self.get_logger().info("关节状态就绪")

        except Exception as e:
            self.log_throttled(
                "debug", f"关节状态错误: {str(e)}", "joint_state_error", 10.0
            )

    def _initialize_system(self):
        """初始化系统"""
        if self.initialized:
            self.destroy_timer(self.init_timer)
            return

        try:
            # 检查TF
            if not self.tf_ready:
                try:
                    # 使用更宽松的TF查询
                    for i in range(max(1, self.tf_retry_count)):
                        try:
                            if self.tf_use_latest:
                                # 使用最新的TF数据，不指定时间
                                transform = self.tf_buffer.lookup_transform(
                                    "base_link",
                                    self.end_effector_link,
                                    rclpy.time.Time(),
                                    timeout=rclpy.duration.Duration(seconds=0.2),
                                )
                            else:
                                # 使用当前时间
                                transform = self.tf_buffer.lookup_transform(
                                    "base_link",
                                    self.end_effector_link,
                                    self.get_clock().now(),
                                    timeout=rclpy.duration.Duration(seconds=0.2),
                                )

                            self.tf_ready = True
                            self.tf_available = True
                            self.last_tf_success_time = time.time()
                            self.get_logger().info("TF系统就绪")
                            break
                        except (TransformException, Exception) as e:
                            if i < self.tf_retry_count - 1:
                                time.sleep(self.tf_retry_delay)
                            else:
                                self.log_throttled(
                                    "debug",
                                    f"TF初始化失败: {str(e)}",
                                    "tf_init_failure",
                                    5.0,
                                )
                                raise
                except Exception:
                    # TF初始化失败，但我们可以继续尝试
                    pass

            # 延迟初始化服务
            if not self.services_ready and (
                self.tf_ready or time.time() - self.start_time > 5.0
            ):
                self._initialize_services()

            # 检查是否完成初始化
            if (
                (self.tf_ready or time.time() - self.start_time > 10.0)
                and self.joints_ready
                and self.services_ready
            ):
                self.initialized = True
                self.destroy_timer(self.init_timer)
                self.get_logger().info("系统完全就绪，可以接收指令!")

        except Exception as e:
            self.log_throttled("debug", f"初始化错误: {str(e)}", "init_error", 10.0)

    def _initialize_services(self):
        """初始化服务客户端"""
        try:
            self.get_logger().info("正在初始化服务客户端...")

            # 创建服务客户端
            self.cartesian_path_client = self.create_client(
                GetCartesianPath,
                "/compute_cartesian_path",
                callback_group=self.slow_callback_group,
            )

            self.motion_plan_client = self.create_client(
                GetMotionPlan,
                "/plan_kinematic_path",
                callback_group=self.slow_callback_group,
            )

            if self.use_execute_trajectory:
                self.execute_action_client = ActionClient(
                    self,
                    ExecuteTrajectory,
                    "/execute_trajectory",
                    callback_group=self.slow_callback_group,
                )

            # 等待服务
            start_time = time.time()
            timeout = 15.0

            while rclpy.ok() and time.time() - start_time < timeout:
                ready = True

                if not self.cartesian_path_client.wait_for_service(timeout_sec=0.2):
                    ready = False

                if not self.motion_plan_client.wait_for_service(timeout_sec=0.2):
                    ready = False

                if self.use_execute_trajectory:
                    if not self.execute_action_client.wait_for_server(timeout_sec=0.2):
                        ready = False

                if ready:
                    self.services_ready = True
                    self.get_logger().info("所有服务就绪")
                    break

                time.sleep(0.2)

            if not self.services_ready:
                self.get_logger().warn("服务初始化超时，将继续尝试...")
                # 即使超时也标记为就绪，避免阻塞
                self.services_ready = True

        except Exception as e:
            self.get_logger().error(f"服务初始化失败: {str(e)}")
            self.services_ready = True

    def get_robot_state(self):
        """获取当前机器人状态 - 增加实时性"""
        try:
            # 增加状态新鲜度检查
            joint_state = self.current_joint_state

            # 检查关节状态是否过期或为空
            if (
                joint_state is None
                or time.time() - self.last_joint_state_time > 0.005  # 减少到50ms
            ):
                # 返回更保守的默认状态
                robot_state = RobotState()
                robot_state.joint_state = JointState()
                robot_state.joint_state.name = self.joint_names

                # 如果有缓存状态，使用缓存（即使稍微过期）
                if joint_state is not None:
                    robot_state.joint_state.position = list(joint_state.position)
                else:
                    robot_state.joint_state.position = [0.0] * len(self.joint_names)

                robot_state.is_diff = True
                robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
                return robot_state

            # 使用当前关节状态，但要确保时间戳是最新的
            robot_state = RobotState()
            robot_state.joint_state = joint_state
            robot_state.joint_state.header.stamp = self.get_clock().now().to_msg()
            robot_state.is_diff = True

            return robot_state

        except Exception as e:
            self.log_throttled(
                "debug", f"获取机器人状态错误: {str(e)}", "get_state_error", 2.0
            )
            # 返回默认状态
            robot_state = RobotState()
            robot_state.joint_state = JointState()
            robot_state.joint_state.name = self.joint_names
            robot_state.joint_state.position = [0.0] * len(self.joint_names)
            robot_state.is_diff = True
            return robot_state

    def _get_tf_transform(self):
        """获取TF变换（核心优化）"""
        try:
            self.perf_stats["tf_lookups"] += 1
            start_time = time.time()

            if self.tf_use_latest:
                # 使用最新的TF数据
                transform = self.tf_buffer.lookup_transform(
                    "base_link",
                    self.end_effector_link,
                    rclpy.time.Time(),  # 最新时间
                    timeout=rclpy.duration.Duration(seconds=self.tf_lookup_timeout),
                )
            else:
                # 使用当前时间
                transform = self.tf_buffer.lookup_transform(
                    "base_link",
                    self.end_effector_link,
                    self.get_clock().now(),
                    timeout=rclpy.duration.Duration(seconds=self.tf_lookup_timeout),
                )

            latency = time.time() - start_time
            self.perf_stats["tf_latency"].append(latency)
            if len(self.perf_stats["tf_latency"]) > 100:
                self.perf_stats["tf_latency"].pop(0)

            self.tf_available = True
            self.last_tf_success_time = time.time()

            # 计算TF成功率
            total_lookups = self.perf_stats["tf_lookups"]
            failures = self.perf_stats["tf_failures"]
            if total_lookups > 0:
                self.perf_stats["tf_success_rate"] = (
                    (total_lookups - failures) / total_lookups * 100
                )

            return transform

        except TransformException as e:
            self.perf_stats["tf_failures"] += 1

            # 记录TF失败率
            total_lookups = self.perf_stats["tf_lookups"]
            if (
                total_lookups > 100
                and self.perf_stats["tf_failures"] / total_lookups > 0.5
            ):
                # TF失败率超过50%，可能是TF树有问题
                self.log_throttled(
                    "warn",
                    f"TF失败率过高: {self.perf_stats['tf_failures']}/{total_lookups}",
                    "tf_high_failure_rate",
                    10.0,
                )

            self.tf_available = False
            return None

        except Exception as e:
            self.perf_stats["tf_failures"] += 1
            self.tf_available = False
            return None

    def _get_current_pose_optimized(self):
        """优化版获取当前末端位姿"""
        # 首先检查TF是否可用
        if not self.tf_ready and time.time() - self.start_time < 5.0:
            # 前5秒内TF可能还未就绪
            return None

        # 尝试获取TF变换
        transform = self._get_tf_transform()

        if transform is not None:
            # 成功获取变换，更新缓存
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation

            self.cached_pose = pose
            self.cached_pose_time = time.time()

            return pose
        else:
            # TF查询失败，检查是否有可用的缓存
            if (
                self.cached_pose is not None
                and time.time() - self.cached_pose_time < self.cached_pose_valid_for
            ):
                # 使用缓存的姿态（在有效时间内）
                self.log_throttled(
                    "debug", "使用缓存的末端姿态", "using_cached_pose", 2.0
                )
                return self.cached_pose

            # 检查TF最近是否成功过
            if self.tf_available and time.time() - self.last_tf_success_time < 2.0:
                # TF最近成功过，可能只是短暂失败
                return None
            else:
                # TF长时间不可用
                self.log_throttled(
                    "warn",
                    f"TF长时间不可用: {time.time() - self.last_tf_success_time:.1f}秒",
                    "tf_long_unavailable",
                    5.0,
                )
                return None

    def _compute_cartesian_path(self, waypoints, retry_count=0):
        """计算笛卡尔路径（带重试）"""
        last_exception = None

        for attempt in range(retry_count + 1):
            try:
                start_time = time.time()

                # 创建请求
                request = GetCartesianPath.Request()
                request.header.stamp = self.get_clock().now().to_msg()
                request.header.frame_id = "base_link"
                request.start_state = self.get_robot_state()
                request.group_name = self.move_group_name
                request.link_name = self.end_effector_link
                request.waypoints = waypoints
                request.max_step = self.cartesian_step_size
                request.jump_threshold = 3.0
                request.avoid_collisions = True
                request.path_constraints = Constraints()

                # 同步调用
                future = self.cartesian_path_client.call_async(request)

                # 等待响应
                start_wait = time.time()
                timeout = 2.0

                while rclpy.ok() and not future.done():
                    if time.time() - start_wait > timeout:
                        if attempt < retry_count:
                            time.sleep(0.05)
                            break
                        else:
                            self.log_throttled(
                                "warn", "笛卡尔规划超时", "cartesian_timeout", 2.0
                            )
                            return None, 0.0
                    time.sleep(0.001)

                if not future.done():
                    continue  # 超时，继续重试

                response = future.result()
                if response:
                    planning_time = time.time() - start_time
                    self.perf_stats["planning_time"].append(planning_time)

                    if len(self.perf_stats["planning_time"]) > 100:
                        self.perf_stats["planning_time"].pop(0)

                    fraction = response.fraction
                    if fraction > 0.90:
                        self.log_throttled(
                            "debug",
                            f"笛卡尔规划成功: {fraction*100:.1f}% ({planning_time:.2f}s)",
                            "cartesian_success",
                            1.0,
                        )
                    return response.solution, fraction
                elif attempt < retry_count:
                    time.sleep(0.05)  # 失败后短暂延迟再重试

            except Exception as e:
                last_exception = e
                if attempt < retry_count:
                    time.sleep(0.05)
                else:
                    self.log_throttled(
                        "debug", f"笛卡尔规划错误: {str(e)}", "cartesian_error", 2.0
                    )

        return None, 0.0

    def _plan_to_pose_moveit(self, pose_stamped, retry_count=0):
        """MoveIt2关节空间规划（带重试）"""
        last_exception = None

        for attempt in range(retry_count + 1):
            try:
                start_time = time.time()

                # 创建请求
                request = GetMotionPlan.Request()
                motion_request = MotionPlanRequest()
                motion_request.start_state = self.get_robot_state()

                # 设置目标约束
                constraints = self._create_precision_constraints(pose_stamped)
                motion_request.goal_constraints.append(constraints)

                # 设置参数
                motion_request.group_name = self.move_group_name
                motion_request.num_planning_attempts = self.num_planning_attempts
                motion_request.allowed_planning_time = self.planning_time
                motion_request.max_velocity_scaling_factor = self.velocity_scaling
                motion_request.max_acceleration_scaling_factor = (
                    self.acceleration_scaling
                )
                motion_request.planner_id = "RRTConnectkConfigDefault"

                request.motion_plan_request = motion_request

                # 调用服务
                future = self.motion_plan_client.call_async(request)

                # 等待响应
                start_wait = time.time()
                max_wait = self.planning_time + 2.0

                while rclpy.ok() and not future.done():
                    if time.time() - start_wait > max_wait:
                        if attempt < retry_count:
                            time.sleep(0.05)
                            break
                        else:
                            self.log_throttled(
                                "warn", "MoveIt2规划超时", "moveit_timeout", 2.0
                            )
                            return None
                    time.sleep(0.001)

                if not future.done():
                    continue  # 超时，继续重试

                response = future.result()
                if response and response.motion_plan_response.error_code.val == 1:
                    planning_time = time.time() - start_time
                    self.perf_stats["planning_time"].append(planning_time)

                    if len(self.perf_stats["planning_time"]) > 100:
                        self.perf_stats["planning_time"].pop(0)

                    self.log_throttled(
                        "debug",
                        f"MoveIt2规划成功 ({planning_time:.2f}s)",
                        "moveit_success",
                        1.0,
                    )
                    return response.motion_plan_response.trajectory
                elif attempt < retry_count:
                    time.sleep(0.05)

            except Exception as e:
                last_exception = e
                if attempt < retry_count:
                    time.sleep(0.05)
                else:
                    self.log_throttled(
                        "debug", f"MoveIt2规划错误: {str(e)}", "moveit_error", 2.0
                    )

        return None

    def _create_precision_constraints(self, pose_stamped):
        """创建精度约束"""
        constraints = Constraints()
        constraints.name = "precision_pose_constraint"

        # 位置约束
        position_constraint = PositionConstraint()
        position_constraint.header = pose_stamped.header
        position_constraint.link_name = self.end_effector_link

        bv = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.position_tolerance]
        bv.primitives.append(sphere)
        bv.primitive_poses.append(pose_stamped.pose)

        position_constraint.constraint_region = bv
        position_constraint.weight = 1.0

        # 姿态约束
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header = pose_stamped.header
        orientation_constraint.link_name = self.end_effector_link
        orientation_constraint.orientation = pose_stamped.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = self.orientation_tolerance
        orientation_constraint.absolute_y_axis_tolerance = self.orientation_tolerance
        orientation_constraint.absolute_z_axis_tolerance = self.orientation_tolerance
        orientation_constraint.weight = 1.0

        constraints.position_constraints.append(position_constraint)
        constraints.orientation_constraints.append(orientation_constraint)

        return constraints

    def _preprocess_trajectory(self, trajectory, current_state):
        """预处理轨迹，确保起始状态一致"""
        if trajectory is None or not trajectory.joint_trajectory.points:
            return trajectory

        try:
            # 获取轨迹的第一个点
            first_point = trajectory.joint_trajectory.points[0]

            # 获取当前关节位置
            current_positions = current_state.joint_state.position

            # 检查偏差
            max_deviation = 0.0
            deviation_joint = -1

            for i, (planned, current) in enumerate(
                zip(first_point.positions, current_positions)
            ):
                deviation = abs(planned - current)
                if deviation > max_deviation:
                    max_deviation = deviation
                    deviation_joint = i

            # 如果偏差超过阈值，调整轨迹起始点
            if max_deviation > 0.005:  # 5mm或度的阈值
                self.log_throttled(
                    "debug",
                    f"调整轨迹起始点，关节{deviation_joint}偏差: {max_deviation:.4f}",
                    "trajectory_adjust",
                    1.0,
                )

                # 创建调整后的轨迹点列表
                adjusted_points = []

                # 第一个点使用当前状态（带微小偏移以避免零速问题）
                adjusted_first_point = JointTrajectoryPoint()
                adjusted_first_point.positions = list(current_positions)

                # 如果只有一个点，直接返回当前位置
                if len(trajectory.joint_trajectory.points) == 1:
                    adjusted_first_point.time_from_start = (
                        trajectory.joint_trajectory.points[0].time_from_start
                    )
                    adjusted_points.append(adjusted_first_point)
                else:
                    # 保持原来的时间间隔，但位置从当前位置开始
                    adjusted_first_point.time_from_start = (
                        trajectory.joint_trajectory.points[0].time_from_start
                    )
                    adjusted_points.append(adjusted_first_point)

                    # 后续点保持相对关系
                    for i in range(1, len(trajectory.joint_trajectory.points)):
                        point = trajectory.joint_trajectory.points[i]
                        adjusted_point = JointTrajectoryPoint()
                        adjusted_point.positions = list(point.positions)
                        adjusted_point.time_from_start = point.time_from_start
                        adjusted_points.append(adjusted_point)

                # 更新轨迹
                trajectory.joint_trajectory.points = adjusted_points

            return trajectory

        except Exception as e:
            self.log_throttled(
                "debug", f"轨迹预处理错误: {str(e)}", "traj_preprocess_error", 2.0
            )
            return trajectory

    def _execute_trajectory(self, trajectory):
        """执行轨迹 - 增加状态同步"""
        if trajectory is None:
            return False

        try:
            # 在执行前获取最新状态
            current_state = self.get_robot_state()

            # 预处理轨迹，确保起始状态一致
            trajectory = self._preprocess_trajectory(trajectory, current_state)

            start_time = time.time()

            goal_msg = ExecuteTrajectory.Goal()
            goal_msg.trajectory = trajectory

            # 设置执行选项
            if hasattr(goal_msg, "allowed_start_tolerance"):
                goal_msg.allowed_start_tolerance = 0.02  # 增大容差

            future = self.execute_action_client.send_goal_async(goal_msg)

            # 等待响应
            start_wait = time.time()
            timeout = 10.0

            execution_success = False

            while rclpy.ok() and not future.done():
                if time.time() - start_wait > timeout:
                    self.log_throttled("warn", "轨迹执行超时", "execution_timeout", 2.0)
                    return False
                time.sleep(0.001)

            if future.done():
                goal_handle = future.result()
                execution_time = time.time() - start_time
                self.perf_stats["execution_time"].append(execution_time)

                if len(self.perf_stats["execution_time"]) > 100:
                    self.perf_stats["execution_time"].pop(0)

                if goal_handle.accepted:
                    # 等待执行完成
                    self.log_throttled(
                        "debug",
                        f"轨迹执行已接受 ({execution_time:.2f}s)",
                        "execution_accepted",
                        1.0,
                    )

                    # 获取结果
                    result_future = goal_handle.get_result_async()
                    result_timeout = execution_time * 1000.0  # 预估时间的两倍
                    # result_timeout = execution_time * 500.0  # 预估时间的两倍

                    result_start = time.time()

                    while rclpy.ok() and not result_future.done():
                        if time.time() - result_start > result_timeout:
                            self.log_throttled(
                                "warn", "等待结果超时", "result_timeout", 2.0
                            )
                            break
                        time.sleep(0.001)

                    if result_future.done():
                        result = result_future.result()
                        if result and result.result.error_code.val == 1:  # SUCCESS
                            execution_success = True
                        else:
                            self.log_throttled(
                                "warn",
                                f"轨迹执行失败: {result.result.error_code.val if result else '无结果'}",
                                "execution_failed",
                                2.0,
                            )
                    else:
                        execution_success = True  # 如果没收到结果但动作已接受，假设成功

            return execution_success

        except Exception as e:
            self.log_throttled(
                "debug", f"轨迹执行错误: {str(e)}", "execution_error", 2.0
            )
            return False

    def _move_to_pose_precision(self, pose_stamped):
        """高精度移动到姿态 - 增加重试和状态同步"""
        max_retries = 3
        last_exception = None

        for attempt in range(max_retries):
            try:
                # 等待新鲜状态
                if not self._wait_for_fresh_joint_state(0.05):
                    self.log_throttled(
                        "debug",
                        f"尝试{attempt+1}: 等待新鲜状态超时",
                        "wait_state_timeout",
                        1.0,
                    )

                # 使用MoveIt2规划
                trajectory = self._plan_to_pose_moveit(
                    pose_stamped, retry_count=max(1, self.planning_retry_count // 3)
                )

                if trajectory is not None:
                    # 预处理轨迹
                    current_state = self.get_robot_state()
                    trajectory = self._preprocess_trajectory(trajectory, current_state)

                    success = self._execute_trajectory(trajectory)

                    if success:
                        self.stats["target_pose_success"] += 1
                        self.stats["success"] += 1
                        return True
                    else:
                        if attempt < max_retries - 1:
                            self.log_throttled(
                                "info",
                                f"执行失败，重试 {attempt+1}/{max_retries}",
                                "execution_retry",
                                1.0,
                            )
                            time.sleep(0.05)  # 短暂延迟后重试
                        else:
                            # 尝试笛卡尔规划作为后备
                            self.log_throttled(
                                "info",
                                "高精度执行失败，尝试笛卡尔规划",
                                "fallback_cartesian",
                                2.0,
                            )
                            return self._move_to_pose_cartesian(pose_stamped)
                else:
                    if attempt < max_retries - 1:
                        time.sleep(0.05)
                    else:
                        # 直接尝试笛卡尔规划
                        self.log_throttled(
                            "info",
                            "高精度规划失败，尝试笛卡尔规划",
                            "direct_cartesian",
                            2.0,
                        )
                        return self._move_to_pose_cartesian(pose_stamped)

            except Exception as e:
                last_exception = e
                if attempt < max_retries - 1:
                    time.sleep(0.05)
                else:
                    self.log_throttled(
                        "debug", f"高精度移动错误: {str(e)}", "precision_error", 2.0
                    )

        self.stats["failed"] += 1
        return False

    def _move_to_pose_cartesian(self, pose_stamped):
        """笛卡尔移动到姿态"""
        try:
            target_pose = pose_stamped.pose

            for _ in range(10):
                # 计算笛卡尔路径（带重试）
                trajectory, fraction = self._compute_cartesian_path(
                    [target_pose], retry_count=self.planning_retry_count
                )
                if fraction > 0.90:
                    break

            if trajectory is None or fraction < 0.90:
                self.log_throttled(
                    "warn",
                    f"笛卡尔规划失败: {fraction*100:.1f}%",
                    "cartesian_failure",
                    2.0,
                )
                self.stats["failed"] += 1
                return False

            # 执行轨迹
            success = self._execute_trajectory(trajectory)

            if success:
                self.stats["cartesian_success"] += 1
                self.stats["success"] += 1
            else:
                self.stats["failed"] += 1

            return success

        except Exception as e:
            self.log_throttled(
                "debug", f"笛卡尔移动错误: {str(e)}", "cartesian_move_error", 2.0
            )
            self.stats["failed"] += 1
            return False

    def _linear_move_cartesian(self, x_offset, y_offset, z_offset):
        """笛卡尔直线移动"""
        try:
            # 获取当前姿态（使用优化版本）
            current_pose = self._get_current_pose_optimized()

            if current_pose is None:
                # 无法获取当前姿态，但有缓存可用吗？
                if (
                    self.cached_pose is not None
                    and time.time() - self.cached_pose_time < 1.0
                ):  # 1秒内的缓存
                    current_pose = self.cached_pose
                    self.log_throttled(
                        "info", "使用缓存姿态进行直线移动", "use_cached_for_move", 2.0
                    )
                else:
                    self.log_throttled(
                        "warn",
                        "无法获取当前姿态，跳过直线移动",
                        "no_current_pose_skip",
                        2.0,
                    )
                    self.stats["failed"] += 1
                    return False

            # 计算目标姿态
            target_pose = Pose()
            target_pose.position.x = current_pose.position.x + x_offset
            target_pose.position.y = current_pose.position.y + y_offset
            target_pose.position.z = current_pose.position.z + z_offset
            target_pose.orientation = current_pose.orientation

            # 创建PoseStamped
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose = target_pose

            # 使用笛卡尔规划
            return self._move_to_pose_cartesian(pose_stamped)

        except Exception as e:
            self.log_throttled(
                "debug", f"直线移动错误: {str(e)}", "linear_move_error", 2.0
            )
            self.stats["failed"] += 1
            return False

    def _linear_move_cartesian_absolute(
        self, target_x, target_y, target_z, target_orientation
    ):
        """笛卡尔直线移动到绝对位置"""
        try:
            # 创建目标姿态
            target_pose = Pose()
            target_pose.position.x = target_x
            target_pose.position.y = target_y
            target_pose.position.z = target_z
            target_pose.orientation = target_orientation

            # 创建PoseStamped
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "base_link"
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose = target_pose

            # 使用笛卡尔规划
            return self._move_to_pose_cartesian(pose_stamped)

        except Exception as e:
            self.log_throttled(
                "debug", f"直线移动错误: {str(e)}", "linear_move_error", 2.0
            )
            self.stats["failed"] += 1
            return False

    def _z_axis_move(self, z_distance):
        """Z轴移动"""
        return self._linear_move_cartesian(0.0, 0.0, z_distance)

    def _process_tasks(self):
        """处理任务"""
        while rclpy.ok() and not self.shutting_down:
            try:
                # 获取任务
                task = self.task_queue.get(timeout=0.05)

                if task is None:
                    continue

                # 提交到线程池处理
                task_start = time.time()
                self.thread_pool.submit(self._process_single_task, task, task_start)
                self.pending_tasks += 1

            except queue.Empty:
                continue
            except Exception as e:
                self.log_throttled(
                    "debug", f"任务获取错误: {str(e)}", "task_fetch_error", 10.0
                )
                time.sleep(0.01)

    def _process_single_task(self, task, task_start):
        """处理单个任务 - 增加队列管理"""
        try:
            # 检查是否有任务在执行，如果有则稍等
            while self.pending_tasks > 0 and time.time() - self.last_task_time < 0.1:
                time.sleep(0.01)  # 等待10ms

            queue_wait_time = time.time() - task_start
            self.perf_stats["queue_wait_time"].append(queue_wait_time)

            if len(self.perf_stats["queue_wait_time"]) > 100:
                self.perf_stats["queue_wait_time"].pop(0)

            task_type = task.get("type")
            task_data = task.get("data")

            success = False

            # 在执行前同步等待最新关节状态
            self._wait_for_fresh_joint_state(timeout=0.1)

            if task_type == "target_pose":
                success = self._move_to_pose_precision(task_data)
            elif task_type == "cartesian_move":
                if len(task_data) == 4:
                    target_x, target_y, target_z, target_orientation = task_data
                    success = self._linear_move_cartesian_absolute(
                        target_x, target_y, target_z, target_orientation
                    )
                else:
                    success = self._linear_move_cartesian(*task_data)
            elif task_type == "z_move":
                success = self._z_axis_move(task_data)

            # 任务完成后短暂延迟，避免状态冲突
            time.sleep(0.01)

            # 更新统计
            self._update_stats(success, task_start)

            # 标记任务完成
            self.task_queue.task_done()
            self.pending_tasks -= 1
            self.last_task_time = time.time()

        except Exception as e:
            self.log_throttled(
                "debug", f"任务处理错误: {str(e)}", "task_process_error", 2.0
            )
            self.stats["failed"] += 1
            self.stats["total"] += 1
            self.pending_tasks -= 1

            # 失败也计入最近成功率
            self._update_recent_stats(False)

    def _wait_for_fresh_joint_state(self, timeout=0.1):
        """等待新鲜的关节状态"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if (
                self.current_joint_state is not None
                and time.time() - self.last_joint_state_time < 0.05
            ):  # 50ms内的状态
                return True
            time.sleep(0.001)
        return False

    def _target_pose_callback(self, msg):
        """目标姿态回调"""
        if not self.initialized:
            self.log_throttled(
                "warn", "系统未就绪，忽略目标姿态", "not_ready_target", 5.0
            )
            return

        try:
            if self.task_queue.qsize() < self.task_queue_size:
                self.task_queue.put_nowait({"type": "target_pose", "data": msg})
                self.log_throttled(
                    "debug",
                    f"收到目标姿态，队列: {self.task_queue.qsize()}",
                    "target_pose_received",
                    1.0,
                )
            else:
                self.stats["queue_full"] += 1
                self.log_throttled(
                    "warn",
                    f"任务队列已满({self.task_queue.qsize()})",
                    "queue_full",
                    1.0,
                )
        except queue.Full:
            self.stats["queue_full"] += 1

    def _cartesian_move_callback(self, msg):
        """笛卡尔移动回调 - 绝对位置模式"""
        if not self.initialized:
            return

        # 获取目标位置和姿态
        target_x = msg.pose.position.x
        target_y = msg.pose.position.y
        target_z = msg.pose.position.z
        target_orientation = msg.pose.orientation

        try:
            if self.task_queue.qsize() < self.task_queue_size:
                self.task_queue.put_nowait(
                    {
                        "type": "cartesian_move",
                        "data": (target_x, target_y, target_z, target_orientation),
                    }
                )
            else:
                self.stats["queue_full"] += 1
        except queue.Full:
            self.stats["queue_full"] += 1

    def _z_move_callback(self, msg):
        """Z轴移动回调"""
        if not self.initialized:
            return

        z_distance = msg.pose.position.z

        try:
            if self.task_queue.qsize() < self.task_queue_size:
                self.task_queue.put_nowait({"type": "z_move", "data": z_distance})
            else:
                self.stats["queue_full"] += 1
        except queue.Full:
            self.stats["queue_full"] += 1

    def _publish_current_pose(self):
        """发布当前位姿"""
        try:
            pose = self._get_current_pose_optimized()
            if pose is not None:
                pose_stamped = PoseStamped()
                pose_stamped.header.stamp = self.get_clock().now().to_msg()
                pose_stamped.header.frame_id = "base_link"
                pose_stamped.pose = pose

                self.end_effector_pose_pub.publish(pose_stamped)

        except Exception:
            pass

    def _monitor_status(self):
        """监控状态"""
        try:
            # 构建状态信息
            status_parts = []

            if not self.initialized:
                status_parts.append("初始化中")
                if self.tf_ready:
                    status_parts.append("TF✓")
                if self.joints_ready:
                    status_parts.append("关节✓")
                if self.services_ready:
                    status_parts.append("服务✓")
            else:
                status_parts.append("运行中")

            # 队列信息
            queue_size = self.task_queue.qsize()
            if queue_size > 0 or self.pending_tasks > 0:
                status_parts.append(f"队列:{queue_size}(处理:{self.pending_tasks})")

            # 统计信息
            total = self.stats["total"]
            if total > 0:
                success_rate = (self.stats["success"] / total * 100) if total > 0 else 0

                # 计算最近100次成功率
                recent_total = self.stats["recent_total"]
                if recent_total > 0:
                    recent_success_rate = (
                        self.stats["recent_success"] / recent_total * 100
                    )

                    # 如果最近成功率低于阈值，发出警告
                    if (
                        recent_total >= 10
                        and recent_success_rate < self.min_success_rate_log
                    ):
                        self.log_throttled(
                            "warn",
                            f"最近{recent_total}次成功率较低: {recent_success_rate:.1f}%",
                            "low_recent_success",
                            10.0,
                        )

                    status_parts.append(
                        f"成功率:{success_rate:.1f}%({recent_success_rate:.1f}%)"
                    )
                else:
                    status_parts.append(f"成功率:{success_rate:.1f}%")

                status_parts.append(f"响应:{self.stats['avg_response_time']:.4f}s")

                # TF成功率
                if self.perf_stats["tf_lookups"] > 0:
                    tf_success_rate = self.perf_stats["tf_success_rate"]
                    if tf_success_rate < 90.0:  # 低于90%时显示
                        status_parts.append(f"TF:{tf_success_rate:.1f}%")

                    # 显示TF延迟
                    if self.perf_stats["tf_latency"]:
                        avg_latency = sum(self.perf_stats["tf_latency"]) / len(
                            self.perf_stats["tf_latency"]
                        )
                        if avg_latency > 0.01:  # 延迟大于10ms时显示
                            status_parts.append(f"TF延迟:{avg_latency*1000:.1f}ms")

            # 只在需要时打印
            should_log = (
                queue_size > 0
                or self.pending_tasks > 0
                or not self.initialized
                or time.time() % 30 < 2
            )  # 每30秒打印一次完整状态

            if should_log and status_parts:
                self.get_logger().info(" | ".join(status_parts))

        except Exception as e:
            self.log_throttled(
                "debug", f"状态监控错误: {str(e)}", "status_monitor_error", 2.0
            )

    def destroy_node(self):
        """清理资源"""
        self.get_logger().info("正在关闭节点...")
        self.shutting_down = True

        # 等待任务完成
        if hasattr(self, "task_thread") and self.task_thread.is_alive():
            self.task_thread.join(timeout=2.0)

        # 关闭线程池
        if hasattr(self, "thread_pool"):
            try:
                self.thread_pool.shutdown(wait=True, timeout=2.0)
            except Exception:
                pass

        self.get_logger().info("节点关闭完成")
        super().destroy_node()

    def __del__(self):
        """析构函数"""
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        # 创建节点
        node = UltimateMoveToPoseNode()
        node.start_time = time.time()  # 记录启动时间

        # 使用多线程执行器
        executor = MultiThreadedExecutor(num_threads=6)
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
