#!/bin/bash
# Generate and launch a non-RMF fleet API navigation demo config.
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

ROBOTS=100
ROBOT_MODEL=simple_cube
GUI=true
TARGET_RTF=1.0
CONFIG_OUT=/tmp/bridge_fleet_demo.yaml

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

python3 "${ROOT_DIR}/fleet_scale_config.py" \
    --robots "$ROBOTS" \
    --robot-model "$ROBOT_MODEL" \
    --target-rtf "$TARGET_RTF" \
    --interface-mode fleet \
    --config-out "$CONFIG_OUT" \
    $(if [ "$GUI" = "true" ]; then printf "%s" "--gui"; fi)

exec ros2 launch pybullet_fleet_ros bridge.launch.py \
    config_yaml:="$CONFIG_OUT" \
    gui:="$GUI" \
    target_rtf:="$TARGET_RTF"
