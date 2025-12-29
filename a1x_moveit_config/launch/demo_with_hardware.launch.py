from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 启动MoveIt demo (包括move_group, RViz等)
    moveit_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("a1x_moveit_config"), "launch", "demo.launch.py"]
            )
        )
    )

    # 启动硬件桥接节点
    bridge_node = Node(
        package="a1x_hardware_bridge",
        executable="a1x_bridge",
        name="a1x_trajectory_bridge",
        output="screen",
    )

    return LaunchDescription(
        [
            moveit_demo_launch,
            # bridge_node,
        ]
    )
