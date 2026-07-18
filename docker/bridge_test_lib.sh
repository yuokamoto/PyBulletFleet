#!/bin/bash
# Shared helpers for Docker-based ROS bridge checks.

source_ros_env() {
    source /opt/ros/jazzy/setup.bash
    source /rmf_demos_ws/install/setup.bash
}

bridge_repo_root() {
    cd /opt/pybullet_fleet
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
