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
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node

# TB3 model parameters
_TB3_MODELS = {
    "burger": {
        "urdf_file": "turtlebot3_burger.urdf",
        "model_name": "turtlebot3_burger",
        "max_linear_vel": 0.22,
        "max_angular_vel": 2.84,
    },
    "waffle": {
        "urdf_file": "turtlebot3_waffle.urdf",
        "model_name": "turtlebot3_waffle",
        "max_linear_vel": 0.26,
        "max_angular_vel": 1.82,
    },
}


def _load_tb3_urdf(model: str) -> str:
    """Process TurtleBot3 URDF via xacro for robot_state_publisher.

    TB3 ships plain ``.urdf`` files that contain xacro constructs
    (``${namespace}``, ``<xacro:arg .../>``).  Processing through xacro
    expands these properly (namespace defaults to empty string).
    """
    try:
        tb3_dir = get_package_share_directory("turtlebot3_description")
        urdf_file = _TB3_MODELS.get(model, _TB3_MODELS["burger"])["urdf_file"]
        urdf_path = os.path.join(tb3_dir, "urdf", urdf_file)
        doc = xacro.process_file(urdf_path)
        return doc.toxml()
    except Exception as e:
        print(f"[tb3_demo] Warning: could not load TB3 URDF: {e}")
        return ""


def _generate_config(model: str, gui: bool = False) -> str:
    """Generate a bridge config YAML for the selected TB3 model."""
    params = _TB3_MODELS.get(model, _TB3_MODELS["burger"])
    config = {
        "simulation": {"gui": gui, "physics": False},
        "robots": [
            {
                "name": "tb3_0",
                "urdf_path": params["model_name"],
                "pose": [0.0, 0.0, 0.01],
                "yaw": 0.0,
                "motion_mode": "differential",
                "max_linear_vel": params["max_linear_vel"],
                "max_linear_accel": 2.5,
                "max_angular_vel": params["max_angular_vel"],
                "max_angular_accel": 10.0,
            }
        ],
    }
    tmp = os.path.join(tempfile.gettempdir(), f"bridge_tb3_{model}.yaml")
    with open(tmp, "w") as f:
        yaml.dump(config, f)
    return tmp


def _launch_setup(context: LaunchContext):
    model = context.launch_configurations.get("model", "burger")
    gui = context.launch_configurations.get("gui", "false")
    rviz = context.launch_configurations.get("rviz", "true")
    target_rtf = context.launch_configurations.get("target_rtf", "1.0")
    publish_rate = context.launch_configurations.get("publish_rate", "50.0")

    pkg_dir = get_package_share_directory("pybullet_fleet_ros")
    src_config = "/rmf_demos_ws/src/pybullet_fleet_ros/config"

    # RViz config
    rviz_config = os.path.join(pkg_dir, "config", "tb3_demo.rviz")
    if not os.path.exists(rviz_config):
        rviz_config = os.path.join(src_config, "tb3_demo.rviz")

    # Generate model-specific bridge config
    config_yaml = _generate_config(model, gui=gui.lower() == "true")

    # Load URDF for robot_state_publisher (RViz rendering)
    robot_description = _load_tb3_urdf(model)

    nodes = [
        Node(
            package="pybullet_fleet_ros",
            executable="bridge_node",
            name="pybullet_fleet_bridge",
            parameters=[
                {
                    "config_yaml": config_yaml,
                    "gui": gui.lower() == "true",
                    "target_rtf": float(target_rtf),
                    "publish_rate": float(publish_rate),
                }
            ],
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
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("rviz", default_value="true", description="Launch RViz"),
            DeclareLaunchArgument("target_rtf", default_value="1.0"),
            DeclareLaunchArgument("publish_rate", default_value="50.0"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
