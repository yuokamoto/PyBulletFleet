"""Launch model catalog demo inside Docker (includes Tier 2 ROS models).

Spawns all available KNOWN_MODELS in a grid — including Tier 2 ROS
packages (ur5e, turtlebot3, fetch) that are only available inside the
Docker container where ROS description packages are installed.

Prerequisites::

    # Inside docker/ directory:
    GUI=true docker compose run --rm bridge \
        ros2 launch pybullet_fleet_ros model_catalog_demo.launch.py

    # Without GUI (headless dry-run):
    docker compose run --rm bridge \
        ros2 launch pybullet_fleet_ros model_catalog_demo.launch.py gui:=false
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    gui_arg = DeclareLaunchArgument("gui", default_value="true", description="Enable PyBullet GUI")
    columns_arg = DeclareLaunchArgument("columns", default_value="5", description="Grid columns")
    spacing_arg = DeclareLaunchArgument("spacing", default_value="3.0", description="Grid spacing (m)")
    floor_arg = DeclareLaunchArgument("floor", default_value="plain", description="Floor: samurai|plain")

    # Run the catalog demo script directly (it lives in the mounted workspace)
    catalog_script = os.path.join(
        os.path.dirname(__file__),
        "..",
        "..",
        "..",
        "..",
        "examples",
        "models",
        "model_catalog_demo.py",
    )

    # Fallback — inside Docker the workspace is at /opt/pybullet_fleet
    if not os.path.isfile(catalog_script):
        catalog_script = "/opt/pybullet_fleet/examples/models/model_catalog_demo.py"

    catalog_process = ExecuteProcess(
        cmd=[
            sys.executable,
            catalog_script,
            "--columns",
            LaunchConfiguration("columns"),
            "--spacing",
            LaunchConfiguration("spacing"),
            "--floor",
            LaunchConfiguration("floor"),
            "--gui",
            LaunchConfiguration("gui"),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            gui_arg,
            columns_arg,
            spacing_arg,
            floor_arg,
            catalog_process,
        ]
    )
