# demo_with_hardware.launch.py
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 启动机器人状态发布器
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("a1x_moveit_config"), "launch", "rsp.launch.py"]
            )
        ),
    )

    # 启动ros2_control，但不启动joint_state_broadcaster
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("a1x_moveit_config"),
                    "launch",
                    "ros2_control.launch.py",
                ]
            )
        ),
        launch_arguments={
            "use_joint_state_broadcaster": "false",  # 禁用 joint_state_broadcaster
            "not_use_joint_state_broadcaster": "true",  # 启用直接启动a1x_group_controller
        }.items(),
    )

    # 启动move_group
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("a1x_moveit_config"),
                    "launch",
                    "move_group.launch.py",
                ]
            )
        ),
    )

    # 启动RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("a1x_moveit_config"),
                    "launch",
                    "moveit_rviz.launch.py",
                ]
            )
        ),
    )

    # 启动硬件桥接节点
    bridge_node = Node(
        package="a1x_hardware_bridge",
        executable="a1x_bridge",
        name="a1x_trajectory_bridge",
        output="screen",
    )

    # 启动关节状态重映射节点
    joint_state_remap_node = Node(
        package="a1x_hardware_bridge",
        executable="joint_state_remap",
        name="joint_state_remap",
        output="screen",
    )

    return LaunchDescription(
        [
            rsp_launch,  # 启动机器人状态发布器
            ros2_control_launch,  # 启动ros2_control，但不启动joint_state_broadcaster
            move_group_launch,  # 启动move_group
            rviz_launch,  # 启动RViz
            # bridge_node,  # 启动硬件桥接节点
            # joint_state_remap_node,  # 启动关节状态重映射节点
        ]
    )
