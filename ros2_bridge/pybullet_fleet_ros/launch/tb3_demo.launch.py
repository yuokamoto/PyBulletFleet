"""Launch bridge with a TurtleBot3 and RViz (Tier 2 ROS model demo).

Demonstrates Tier 2 URDF resolution for TurtleBot3 models.
Supports ``burger`` (default) and ``waffle`` via the ``model`` argument.

Prerequisites::

    apt install ros-${ROS_DISTRO}-turtlebot3-description

Usage::

    ros2 launch pybullet_fleet_ros tb3_demo.launch.py
    ros2 launch pybullet_fleet_ros tb3_demo.launch.py model:=waffle gui:=true
    ros2 launch pybullet_fleet_ros tb3_demo.launch.py model:=burger gui:=true rviz:=false
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

# TB3 URDF file names (for robot_state_publisher / RViz)
_TB3_URDF_FILES = {
    "burger": "turtlebot3_burger.urdf",
    "waffle": "turtlebot3_waffle.urdf",
}


def _load_tb3_urdf(model: str) -> str:
    """Process TurtleBot3 URDF via xacro for robot_state_publisher.

    TB3 ships plain ``.urdf`` files that contain xacro constructs
    (``${namespace}``, ``<xacro:arg .../>``).  Processing through xacro
    expands these properly (namespace defaults to empty string).
    """
    try:
        tb3_dir = get_package_share_directory("turtlebot3_description")
        urdf_file = _TB3_URDF_FILES.get(model, _TB3_URDF_FILES["burger"])
        urdf_path = os.path.join(tb3_dir, "urdf", urdf_file)
        doc = xacro.process_file(urdf_path)
        return doc.toxml()
    except Exception as e:
        print(f"[tb3_demo] Warning: could not load TB3 URDF: {e}")
        return ""


def _resolve_config(model: str, pkg_dir: str) -> str:
    """Resolve the bridge config YAML for the selected TB3 model.

    Looks for ``config/bridge_tb3_{model}.yaml`` in the package share
    directory first, then falls back to the dev-mount source path.
    """
    filename = f"bridge_tb3_{model}.yaml"
    config_path = os.path.join(pkg_dir, "config", filename)
    if not os.path.exists(config_path):
        # Dev-mount fallback (Docker volume overlay)
        config_path = os.path.join("/rmf_demos_ws/src/pybullet_fleet_ros/config", filename)
    if not os.path.exists(config_path):
        raise FileNotFoundError(
            f"TB3 config not found for model '{model}': tried {filename} "
            f"in {pkg_dir}/config/ and /rmf_demos_ws/src/pybullet_fleet_ros/config/"
        )
    return config_path


def _launch_setup(context: LaunchContext):
    model = context.launch_configurations.get("model", "burger")
    gui = context.launch_configurations.get("gui", "")
    rviz = context.launch_configurations.get("rviz", "true")
    target_rtf = context.launch_configurations.get("target_rtf", "")

    pkg_dir = get_package_share_directory("pybullet_fleet_ros")
    src_config = "/rmf_demos_ws/src/pybullet_fleet_ros/config"

    # RViz config
    rviz_config = os.path.join(pkg_dir, "config", "tb3_demo.rviz")
    if not os.path.exists(rviz_config):
        rviz_config = os.path.join(src_config, "tb3_demo.rviz")

    # Resolve model-specific bridge config from package share
    config_yaml = _resolve_config(model, pkg_dir)

    # Load URDF for robot_state_publisher (RViz rendering)
    robot_description = _load_tb3_urdf(model)

    bridge_params = {
        "config_yaml": config_yaml,
    }
    if gui:
        bridge_params["gui"] = gui.lower() in ("true", "1", "yes")
    if target_rtf:
        bridge_params["target_rtf"] = float(target_rtf)

    nodes = [
        Node(
            package="pybullet_fleet_ros",
            executable="bridge_node",
            name="pybullet_fleet_bridge",
            parameters=[bridge_params],
            output="screen",
        ),
    ]

    if rviz.lower() == "true":
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            )
        )

    if robot_description and rviz.lower() == "true":
        nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                namespace="tb3_0",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "frame_prefix": "tb3_0/",
                    }
                ],
                remappings=[("joint_states", "/tb3_0/joint_states")],
                output="screen",
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("model", default_value="burger", description="TB3 model: burger or waffle"),
            DeclareLaunchArgument("gui", default_value=""),
            DeclareLaunchArgument("rviz", default_value="true", description="Launch RViz"),
            DeclareLaunchArgument("target_rtf", default_value=""),
            OpaqueFunction(function=_launch_setup),
        ]
    )
