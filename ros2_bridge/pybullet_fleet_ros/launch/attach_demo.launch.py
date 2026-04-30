"""Launch bridge with a mobile robot and pickable objects for attach/detach demo.

Spawns 1 TurtleBot3 + 3 pickable boxes.  Use the ``attach_object`` or
``toggle_attach`` ROS 2 services (or the helper scripts) to pick up and
drop objects.

Usage::

    ros2 launch pybullet_fleet_ros attach_demo.launch.py
    ros2 launch pybullet_fleet_ros attach_demo.launch.py gui:=true

Then in another terminal::

    # Navigate to a box
    python3 /opt/pybullet_fleet/scripts/send_nav_goal.py --robot tb3_0 --x 0.5 --y 0.0

    # Attach nearest pickable object
    python3 /opt/pybullet_fleet/scripts/attach_object.py --robot tb3_0 --attach

    # Detach
    python3 /opt/pybullet_fleet/scripts/attach_object.py --robot tb3_0 --detach
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def _launch_setup(context: LaunchContext):
    pkg_dir = get_package_share_directory("pybullet_fleet_ros")
    src_config = "/rmf_demos_ws/src/pybullet_fleet_ros/config"

    config_yaml = os.path.join(pkg_dir, "config", "bridge_attach_demo.yaml")
    if not os.path.exists(config_yaml):
        config_yaml = os.path.join(src_config, "bridge_attach_demo.yaml")

    gui = context.launch_configurations.get("gui", "")
    target_rtf = context.launch_configurations.get("target_rtf", "")

    bridge_params = {
        "config_yaml": config_yaml,
    }
    if gui:
        bridge_params["gui"] = gui.lower() in ("true", "1", "yes")
    if target_rtf:
        bridge_params["target_rtf"] = float(target_rtf)

    return [
        Node(
            package="pybullet_fleet_ros",
            executable="bridge_node",
            name="pybullet_fleet_bridge",
            parameters=[bridge_params],
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="", description="Enable PyBullet GUI"),
            DeclareLaunchArgument("target_rtf", default_value="", description="Real-time factor"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
