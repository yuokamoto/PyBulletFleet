#!/usr/bin/env bash
# Build the rmf_demos workspace needed by PyBulletFleet's native ROS 2 Jazzy
# workflow. This installs apt dependencies, then builds only the rmf_demos
# packages used by the PyBulletFleet RMF launch files.
set -euo pipefail

RMF_WS="${RMF_WS:-$HOME/rmf_demos_ws}"
RMF_DEMOS_VERSION="${RMF_DEMOS_VERSION:-2.3.0}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"

if [ ! -f "$ROS_SETUP" ]; then
    echo "ROS setup not found: $ROS_SETUP" >&2
    exit 1
fi

sudo apt update
sudo apt install -y git python3-colcon-common-extensions python3-vcstool python3-rosdep python3-catkin-pkg python3-empy python3-lark ros-jazzy-rmf-dev ros-jazzy-rmf-demos-assets ros-jazzy-rmf-demos-bridges ros-jazzy-rmf-demos-fleet-adapter ros-jazzy-rmf-demos-tasks ros-jazzy-nav2-msgs ros-jazzy-control-msgs ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs ros-jazzy-trajectory-msgs ros-jazzy-diagnostic-msgs ros-jazzy-simulation-interfaces ros-jazzy-teleop-twist-keyboard ros-jazzy-xacro ros-jazzy-turtlebot3-description ros-jazzy-ur-description python3-tk

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi

rosdep update
mkdir -p "$RMF_WS/src"

if [ ! -d "$RMF_WS/src/rmf_demos/.git" ]; then
    git clone --branch "$RMF_DEMOS_VERSION" --depth 1 https://github.com/open-rmf/rmf_demos.git "$RMF_WS/src/rmf_demos"
else
    git -C "$RMF_WS/src/rmf_demos" fetch --tags --depth 1 origin "$RMF_DEMOS_VERSION"
    git -C "$RMF_WS/src/rmf_demos" checkout "$RMF_DEMOS_VERSION"
fi

rm -rf "$RMF_WS/build" "$RMF_WS/log"

set +u
source "$ROS_SETUP"
set -u

rosdep install --from-paths "$RMF_WS/src/rmf_demos/rmf_demos" "$RMF_WS/src/rmf_demos/rmf_demos_assets" "$RMF_WS/src/rmf_demos/rmf_demos_bridges" "$RMF_WS/src/rmf_demos/rmf_demos_fleet_adapter" "$RMF_WS/src/rmf_demos/rmf_demos_maps" "$RMF_WS/src/rmf_demos/rmf_demos_tasks" --ignore-src -r -y
colcon --log-base "$RMF_WS/log" build --base-paths "$RMF_WS/src/rmf_demos" --build-base "$RMF_WS/build" --install-base "$RMF_WS/install" --symlink-install --packages-select rmf_demos rmf_demos_assets rmf_demos_bridges rmf_demos_fleet_adapter rmf_demos_maps rmf_demos_tasks --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3

ls "$RMF_WS/install/setup.bash"
