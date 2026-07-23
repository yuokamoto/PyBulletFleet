#!/bin/bash
# Shared helpers for Docker-based ROS bridge checks.

source_ros_env() {
    local ros_setup="${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
    if [ -f "$ros_setup" ]; then
        source "$ros_setup"
    fi
    if [ -n "${RMF_SETUP:-}" ] && [ -f "$RMF_SETUP" ]; then
        source "$RMF_SETUP"
    elif [ -f /rmf_demos_ws/install/setup.bash ]; then
        source /rmf_demos_ws/install/setup.bash
    elif [ -f "$HOME/rmf_demos_ws/install/setup.bash" ]; then
        source "$HOME/rmf_demos_ws/install/setup.bash"
    fi
    if [ -n "${PBF_ROS_SETUP:-}" ] && [ -f "$PBF_ROS_SETUP" ]; then
        source "$PBF_ROS_SETUP"
    elif [ -f .ros2_ws/install/setup.bash ]; then
        source .ros2_ws/install/setup.bash
    fi
}

bridge_repo_root() {
    if [ -n "${PBF_REPO_ROOT:-}" ]; then
        cd "$PBF_REPO_ROOT"
    elif [ -d /opt/pybullet_fleet ]; then
        cd /opt/pybullet_fleet
    fi
}

start_bridge_node() {
    local config_yaml="$1"
    local gui="$2"
    local publish_rate="$3"

    ros2 run pybullet_fleet_ros bridge_node \
        --ros-args -p config_yaml:="$config_yaml" -p gui:="$gui" -p publish_rate:="$publish_rate" &
    BRIDGE_PID=$!
}

stop_bridge_node() {
    local pid="${1:-${BRIDGE_PID:-}}"
    if [ -z "$pid" ]; then
        return
    fi
    kill "$pid" 2>/dev/null || true
    sleep 1
    kill -9 "$pid" 2>/dev/null || true
}
