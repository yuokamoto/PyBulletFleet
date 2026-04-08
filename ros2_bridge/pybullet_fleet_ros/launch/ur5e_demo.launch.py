"""Launch bridge with a UR5e arm and RViz (Tier 2 ROS model demo).

Demonstrates Tier 2 URDF resolution: ``urdf_path: "ur5e"`` in the config
is auto-resolved to the installed ``ur_description`` ROS package.
The xacro is expanded at launch time so that ``package://`` mesh
references are preserved for RViz rendering.

Prerequisites::

    apt install ros-${ROS_DISTRO}-ur-description

Usage::

    ros2 launch pybullet_fleet_ros ur5e_demo.launch.py
    ros2 launch pybullet_fleet_ros ur5e_demo.launch.py gui:=true
    ros2 launch pybullet_fleet_ros ur5e_demo.launch.py gui:=true rviz:=false
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# UR5e xacro parameters
_UR5E_XACRO_MAPPINGS = {
    "ur_type": "ur5e",
    "name": "ur5e",
    "safety_limits": "true",
    "safety_pos_margin": "0.15",
    "safety_k_position": "20",
}


def _generate_ur5e_description() -> str:
    """Expand ur_description xacro at launch time → URDF XML string."""
    try:
        ur_dir = get_package_share_directory("ur_description")
        xacro_file = os.path.join(ur_dir, "urdf", "ur.urdf.xacro")
        doc = xacro.process_file(xacro_file, mappings=_UR5E_XACRO_MAPPINGS)
        return doc.toxml()
    except Exception as e:
        print(f"[ur5e_demo] Warning: could not expand UR5e xacro: {e}")
        return ""


def generate_launch_description():
    pkg_dir = get_package_share_directory("pybullet_fleet_ros")
    src_config = "/opt/bridge_ws/src/pybullet_fleet_ros/config"

    # RViz config
    rviz_config = os.path.join(pkg_dir, "config", "ur5e_demo.rviz")
    if not os.path.exists(rviz_config):
        rviz_config = os.path.join(src_config, "ur5e_demo.rviz")

    # Config yaml — try installed share dir first, fall back to source mount
    config_yaml = os.path.join(pkg_dir, "config", "bridge_ur5e.yaml")
    if not os.path.exists(config_yaml):
        config_yaml = os.path.join(src_config, "bridge_ur5e.yaml")

    # Expand UR5e xacro at launch time for robot_state_publisher (RViz)
    robot_description = _generate_ur5e_description()

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
    if robot_description:
        nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                namespace="ur5e_0",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "frame_prefix": "ur5e_0/",
                    }
                ],
                remappings=[("joint_states", "/ur5e_0/joint_states")],
                output="screen",
                condition=IfCondition(LaunchConfiguration("rviz")),
            )
        )

    return LaunchDescription(nodes)
