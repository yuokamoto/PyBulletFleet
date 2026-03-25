"""Launch bridge with an arm robot and RViz for arm control demo.

Usage::

    ros2 launch pybullet_fleet_ros arm_demo.launch.py
    ros2 launch pybullet_fleet_ros arm_demo.launch.py gui:=true
    ros2 launch pybullet_fleet_ros arm_demo.launch.py gui:=true rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("pybullet_fleet_ros")
    src_config = "/opt/bridge_ws/src/pybullet_fleet_ros/config"

    rviz_config = os.path.join(pkg_dir, "config", "arm_demo.rviz")
    if not os.path.exists(rviz_config):
        rviz_config = os.path.join(src_config, "arm_demo.rviz")

    # Config yaml — try installed share dir first, fall back to source mount.
    # Docker volume-mounts overlay /opt/bridge_ws/src/ but not install/,
    # so newly added configs may only exist in the source tree.
    config_yaml = os.path.join(pkg_dir, "config", "bridge_arm.yaml")
    if not os.path.exists(config_yaml):
        config_yaml = os.path.join(src_config, "bridge_arm.yaml")

    # Read URDF for robot_state_publisher (RViz model visualisation)
    urdf_path = "/opt/pybullet_fleet/robots/arm_robot.urdf"
    try:
        with open(urdf_path, "r") as f:
            robot_description = f.read()
    except FileNotFoundError:
        robot_description = ""

    nodes = [
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true", description="Launch RViz"),
        DeclareLaunchArgument("target_rtf", default_value="1.0"),
        DeclareLaunchArgument("publish_rate", default_value="50.0"),
        Node(
            package="pybullet_fleet_ros",
            executable="bridge_node",
            name="pybullet_fleet_bridge",
            parameters=[
                {
                    "config_yaml": config_yaml,
                    "gui": LaunchConfiguration("gui"),
                    "target_rtf": LaunchConfiguration("target_rtf"),
                    "publish_rate": LaunchConfiguration("publish_rate"),
                }
            ],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ]

    # robot_state_publisher for URDF visualisation in RViz
    # frame_prefix ensures TF frames are arm0/base_link, arm0/shoulder_link, etc.
    # matching the bridge's TF: odom → arm0/base_link
    if robot_description:
        nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                namespace="arm0",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "frame_prefix": "arm0/",
                    }
                ],
                remappings=[("joint_states", "/arm0/joint_states")],
                output="screen",
                condition=IfCondition(LaunchConfiguration("rviz")),
            )
        )

    return LaunchDescription(nodes)
