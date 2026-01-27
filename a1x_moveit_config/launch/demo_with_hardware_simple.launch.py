from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 启动 robot_state_publisher
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("a1x_moveit_config"), "launch", "rsp.launch.py"]
            )
        )
    )

    # 启动 ros2_control（但不启动 joint_state_broadcaster）
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("a1x_moveit_config"), "launch", "ros2_control.launch.py"]
            )
        ),
        launch_arguments={
            "controllers": "a1x_group_controller",
        }.items()
    )

    # 启动 move_group
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("a1x_moveit_config"), "launch", "move_group.launch.py"]
            )
        )
    )

    # 启动 RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("a1x_moveit_config"), "launch", "moveit_rviz.launch.py"]
            )
        )
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
            rsp_launch,
            ros2_control_launch,
            move_group_launch,
            rviz_launch,
            joint_state_remap_node,
        ]
    )
