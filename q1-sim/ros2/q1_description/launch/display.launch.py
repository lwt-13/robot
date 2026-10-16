"""在 RViz2 中显示 Q1 模型，并用 joint_state_publisher_gui 拖动关节。

用法:
    ros2 launch q1_description display.launch.py
    ros2 launch q1_description display.launch.py gui:=false      # 不带关节滑块
    ros2 launch q1_description display.launch.py rviz:=false      # 只起 RSP，不开 RViz
    ros2 launch q1_description display.launch.py urdf:=/path/to/x.urdf
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(text: str) -> bool:
    return text.strip().lower() in ("1", "true", "yes", "on")


def _launch_setup(context, *args, **kwargs):
    urdf_path = LaunchConfiguration("urdf").perform(context)
    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    use_gui = _as_bool(LaunchConfiguration("gui").perform(context))
    use_rviz = _as_bool(LaunchConfiguration("rviz").perform(context))

    if not os.path.isfile(urdf_path):
        raise RuntimeError(f"URDF 文件不存在: {urdf_path}")
    with open(urdf_path, "r", encoding="utf-8") as handle:
        robot_description = handle.read()

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui" if use_gui else "joint_state_publisher",
            executable="joint_state_publisher_gui" if use_gui else "joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
        ),
    ]

    if use_rviz:
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            )
        )

    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory("q1_description")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf",
                default_value=os.path.join(pkg_share, "urdf", "q1_v3.urdf"),
                description="要加载的 URDF 文件路径",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(pkg_share, "rviz", "q1.rviz"),
                description="RViz2 配置文件路径",
            ),
            DeclareLaunchArgument("gui", default_value="true", description="是否显示关节滑块"),
            DeclareLaunchArgument("rviz", default_value="true", description="是否启动 RViz2"),
            OpaqueFunction(function=_launch_setup),
        ]
    )