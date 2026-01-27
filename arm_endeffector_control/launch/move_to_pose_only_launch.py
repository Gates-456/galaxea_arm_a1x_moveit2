from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 声明launch参数
    move_group_name_arg = DeclareLaunchArgument(
        "move_group_name",
        default_value="a1x_group",  # 使用正确的规划组名称
        description="Name of the move_group to use for planning",
    )

    end_effector_link_arg = DeclareLaunchArgument(
        "end_effector_link",
        default_value="arm_link6",  # 使用正确的末端执行器链接名称
        description="Name of the end effector link",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="false", description="Use simulation time"
    )

    velocity_scaling_arg = DeclareLaunchArgument(
        "velocity_scaling",
        default_value="1.0",
        description="Velocity scaling factor for motion planning (0.0-1.0)",
    )

    acceleration_scaling_arg = DeclareLaunchArgument(
        "acceleration_scaling",
        default_value="1.0",
        description="Acceleration scaling factor for motion planning (0.0-1.0)",
    )

    # 获取参数
    move_group_name = LaunchConfiguration("move_group_name")
    end_effector_link = LaunchConfiguration("end_effector_link")
    use_sim_time = LaunchConfiguration("use_sim_time")
    velocity_scaling = LaunchConfiguration("velocity_scaling")
    acceleration_scaling = LaunchConfiguration("acceleration_scaling")

    return LaunchDescription(
        [
            move_group_name_arg,
            end_effector_link_arg,
            use_sim_time_arg,
            velocity_scaling_arg,
            acceleration_scaling_arg,
            Node(
                package="arm_endeffector_control",
                executable="move_to_pose",
                name="move_to_pose_node",
                output="screen",
                parameters=[
                    {"move_group_name": move_group_name},
                    {"end_effector_link": end_effector_link},
                    {"use_sim_time": use_sim_time},
                    {"velocity_scaling": velocity_scaling},
                    {"acceleration_scaling": acceleration_scaling},
                ],
            ),
        ]
    )
