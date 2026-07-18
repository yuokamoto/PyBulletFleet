#!/bin/bash
# docker/test_fleet_scale.sh
# Run a ROS fleet API scale check with the same launcher/checker split as
# test_integration.sh.
set -e

ROBOTS=100
PUBLISH_RATE=5.0
TIMEOUT=60.0
GUI=false
TARGET_RTF=0.0
INTERFACE_MODE=fleet
COMMAND_INTERFACE=fleet
CONFIG_OUT=
GENERATE_ONLY=false
TEMPLATE=
PER_ROBOT_GROUPS=default
VERIFY_MOTION=true
PER_ROBOT_PUBLISH_REPEATS=3
PER_ROBOT_PUBLISH_BATCH_SIZE=0
MEASURE_RTF=false
RTF_WARMUP=1.0
RTF_DURATION=10.0

while [ "$#" -gt 0 ]; do
    case "$1" in
        --robots)
            ROBOTS="$2"
            shift 2
            ;;
        --publish-rate)
            PUBLISH_RATE="$2"
            shift 2
            ;;
        --timeout)
            TIMEOUT="$2"
            shift 2
            ;;
        --gui)
            GUI=true
            shift
            ;;
        --target-rtf)
            TARGET_RTF="$2"
            shift 2
            ;;
        --interface-mode)
            INTERFACE_MODE="$2"
            shift 2
            ;;
        --command-interface)
            COMMAND_INTERFACE="$2"
            shift 2
            ;;
        --template)
            TEMPLATE="$2"
            shift 2
            ;;
        --per-robot-groups)
            PER_ROBOT_GROUPS="$2"
            shift 2
            ;;
        --no-verify-motion)
            VERIFY_MOTION=false
            shift
            ;;
        --per-robot-publish-repeats)
            PER_ROBOT_PUBLISH_REPEATS="$2"
            shift 2
            ;;
        --per-robot-publish-batch-size)
            PER_ROBOT_PUBLISH_BATCH_SIZE="$2"
            shift 2
            ;;
        --measure-rtf)
            MEASURE_RTF=true
            shift
            ;;
        --rtf-warmup)
            RTF_WARMUP="$2"
            shift 2
            ;;
        --rtf-duration)
            RTF_DURATION="$2"
            shift 2
            ;;
        --config-out)
            CONFIG_OUT="$2"
            shift 2
            ;;
        --generate-only)
            GENERATE_ONLY=true
            shift
            ;;
        *)
            echo "unknown option: $1" >&2
            exit 2
            ;;
    esac
done

echo "=== Fleet API scale check: ${ROBOTS} robots (${INTERFACE_MODE}/${COMMAND_INTERFACE}) ==="

LIB=${BRIDGE_TEST_LIB:-/bridge_test_lib.sh}
if [ ! -f "$LIB" ]; then
    LIB=/docker/bridge_test_lib.sh
fi
if [ ! -f "$LIB" ]; then
    LIB=/docker/docker/bridge_test_lib.sh
fi
if [ ! -f "$LIB" ]; then
    LIB=/opt/pybullet_fleet/docker/bridge_test_lib.sh
fi
source "$LIB"

CHECKER=${FLEET_SCALE_CHECK:-/fleet_scale_check.py}
if [ ! -f "$CHECKER" ]; then
    CHECKER=/docker/fleet_scale_check.py
fi
if [ ! -f "$CHECKER" ]; then
    CHECKER=/docker/docker/fleet_scale_check.py
fi
if [ ! -f "$CHECKER" ]; then
    CHECKER=/opt/pybullet_fleet/docker/fleet_scale_check.py
fi
CONFIG_GENERATOR=${FLEET_SCALE_CONFIG:-/fleet_scale_config.py}
if [ ! -f "$CONFIG_GENERATOR" ]; then
    CONFIG_GENERATOR=/docker/fleet_scale_config.py
fi
if [ ! -f "$CONFIG_GENERATOR" ]; then
    CONFIG_GENERATOR=/docker/docker/fleet_scale_config.py
fi
if [ ! -f "$CONFIG_GENERATOR" ]; then
    CONFIG_GENERATOR=/opt/pybullet_fleet/docker/fleet_scale_config.py
fi

source_ros_env
bridge_repo_root

TMPDIR=$(mktemp -d -t pbf_fleet_scale_XXXXXX)
CONFIG_PATH="$TMPDIR/bridge_fleet_scale.yaml"

cleanup() {
    stop_bridge_node "${BRIDGE_PID:-}"
    rm -rf "$TMPDIR"
}
trap cleanup EXIT

CONFIG_ARGS=(
    --robots "$ROBOTS"
    --target-rtf "$TARGET_RTF"
    --interface-mode "$INTERFACE_MODE"
    --per-robot-groups "$PER_ROBOT_GROUPS"
    --config-out "$CONFIG_PATH"
)
if [ "$GUI" = true ]; then
    CONFIG_ARGS+=(--gui)
fi
if [ -n "$TEMPLATE" ]; then
    CONFIG_ARGS+=(--template "$TEMPLATE")
fi
echo "--- Phase 1/2: generate bridge config ---"
python3 "$CONFIG_GENERATOR" "${CONFIG_ARGS[@]}"

if [ -n "$CONFIG_OUT" ]; then
    cp "$CONFIG_PATH" "$CONFIG_OUT"
    echo "Wrote generated bridge config: $CONFIG_OUT"
fi

if [ "$GENERATE_ONLY" = true ]; then
    trap - EXIT
    rm -rf "$TMPDIR"
    exit 0
fi

start_bridge_node "$CONFIG_PATH" "$GUI" "$PUBLISH_RATE"

echo "--- Phase 2/2: run ROS scale checks ---"
CHECK_ARGS=(
    --robots "$ROBOTS"
    --timeout "$TIMEOUT"
    --interface-mode "$INTERFACE_MODE"
    --command-interface "$COMMAND_INTERFACE"
    --per-robot-publish-repeats "$PER_ROBOT_PUBLISH_REPEATS"
    --per-robot-publish-batch-size "$PER_ROBOT_PUBLISH_BATCH_SIZE"
)
if [ "$VERIFY_MOTION" = false ]; then
    CHECK_ARGS+=(--no-verify-motion)
fi
if [ "$MEASURE_RTF" = true ]; then
    CHECK_ARGS+=(--measure-rtf --rtf-warmup "$RTF_WARMUP" --rtf-duration "$RTF_DURATION")
fi

python3 "$CHECKER" "${CHECK_ARGS[@]}"

trap - EXIT
cleanup

echo ""
echo "=== Fleet API scale check PASSED ==="
