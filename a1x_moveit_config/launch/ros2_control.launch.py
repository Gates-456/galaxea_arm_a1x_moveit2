from launch import LaunchDescription
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessStart
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # 获取机器人描述 - 修正URDF文件路径
    robot_description_content = ParameterValue(
        Command(
            [
                "xacro ",
                PathJoinSubstitution([FindPackageShare("a1x"), "urdf", "a1x.urdf"]),  # 确保路径正确
            ]
        ),
        value_type=str
    )
    
    robot_description = {"robot_description": robot_description_content}

    # 启动控制器管理器
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description,
                   PathJoinSubstitution([FindPackageShare("a1x_moveit_config"), "config", "ros2_controllers.yaml"])],
        output="both",
    )

    # 启动joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "60",
                   "--namespace", ""],
        output="screen",
    )

    # 定义a1x_group_controller启动节点
    a1x_group_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["a1x_group_controller",
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "60",
                   "--namespace", ""],
        output="screen",
    )

    # 注册事件处理器，确保在joint_state_broadcaster启动后再启动控制器
    delay_a1x_group_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[
                a1x_group_controller_spawner,
            ],
        )
    )

    return LaunchDescription([
        control_node,
        joint_state_broadcaster_spawner,
        delay_a1x_group_controller_spawner_after_joint_state_broadcaster_spawner,
    ])