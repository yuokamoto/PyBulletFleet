#!/bin/bash
# Launch a non-RMF fleet API navigation demo.
set -e

ROBOTS=100
ROBOT_MODEL=simple_cube
GUI=true
TARGET_RTF=1.0

usage() {
    echo "usage: run_fleet_api_demo.sh [--robots N] [--robot-model simple_cube|mobile_robot|tb3_burger|tb3_waffle] [--gui true|false] [--target-rtf RTF]" >&2
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --robots)
            ROBOTS="$2"
            shift 2
            ;;
        --robot-model)
            ROBOT_MODEL="$2"
            shift 2
            ;;
        --gui)
            GUI="$2"
            shift 2
            ;;
        --target-rtf)
            TARGET_RTF="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            usage
            exit 2
            ;;
    esac
done

exec ros2 launch pybullet_fleet_ros fleet_demo.launch.py \
    robots:="$ROBOTS" \
    robot_model:="$ROBOT_MODEL" \
    gui:="$GUI" \
    target_rtf:="$TARGET_RTF"
