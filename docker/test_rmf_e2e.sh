#!/bin/bash
# docker/test_rmf_e2e.sh [--client-mode MODE] <launch_stem> <scenario> [<scenario> ...]
#
# Parametrized RMF dispatch end-to-end test. Launches an RMF demo headless and
# runs rmf_dispatch_check.py with the given scenarios; each dispatched task must
# finish according to the portable /fleet_states task assignment signal, with
# /task_state_update and /task_summaries used only when the launched RMF stack
# publishes them. Used for several demo/task combinations from bridge.yml:
#
#   bash test_rmf_e2e.sh --client-mode fleet_ros office_pybullet \
#       "patrol:lounge,coe" \
#       "delivery:pantry,coke_dispenser,hardware_2,coke_ingestor"
#   bash test_rmf_e2e.sh hotel_pybullet \
#       "patrol:lobby,L2_room1" "clean:clean_lobby"
#
# Mounted into the container (cf. test_integration.sh):
#   docker compose run --rm --no-deps \
#     -v "$(pwd)/test_rmf_e2e.sh:/test_rmf_e2e.sh:ro" \
#     -v "$(pwd)/rmf_dispatch_check.py:/rmf_dispatch_check.py:ro" \
#     bridge bash /test_rmf_e2e.sh <launch_stem> <scenario...>
#
# target_rtf:=1.0 is essential: the office/hotel YAML defaults leave RTF uncapped,
# so a headless run advances sim time ~10x and desyncs RMF. Pacing at real time
# keeps RMF in lockstep (what GUI runs do implicitly).
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CHECKER="${ROOT_DIR}/rmf_dispatch_check.py"
if [ ! -f "$CHECKER" ]; then
    CHECKER=/rmf_dispatch_check.py
fi

usage() {
    echo "usage: test_rmf_e2e.sh [--client-mode per_robot_ros|fleet_ros|python_fleet] [--ready-only] <launch_stem> [<scenario> ...]" >&2
}

# Validate arg count before `shift` — under `set -e`, `shift` with no args would
# abort with "shift count out of range" before we could print usage.
CLIENT_MODE=python_fleet
READY_ONLY=false
while [ "$#" -gt 0 ]; do
    case "$1" in
        --client-mode)
            if [ "$#" -lt 2 ]; then
                usage
                exit 2
            fi
            CLIENT_MODE="$2"
            shift 2
            ;;
        --ready-only)
            READY_ONLY=true
            shift
            ;;
        --)
            shift
            break
            ;;
        *)
            break
            ;;
    esac
done

if [ "$#" -lt 1 ] || { [ "$READY_ONLY" = false ] && [ "$#" -lt 2 ]; }; then
    usage
    exit 2
fi
LAUNCH="$1"
shift
SCENARIOS=("$@")

case "$LAUNCH" in
    office_pybullet)
        EXPECTED_ROBOTS="tinyRobot1,tinyRobot2"
        ;;
    hotel_pybullet)
        EXPECTED_ROBOTS="tinyBot_1,deliveryBot_1,cleanerBotA_1,cleanerBotA_2"
        ;;
    clinic_pybullet)
        EXPECTED_ROBOTS="deliveryRobot_0,deliveryRobot_1,tinyRobot_0,tinyRobot_1"
        ;;
    airport_terminal_pybullet)
        EXPECTED_ROBOTS="tinyRobot_0,tinyRobot_1,tinyRobot_2,tinyRobot_3,deliveryRobot_0,deliveryRobot_1,deliveryRobot_2,cleanerBotA_0,cleanerBotE_0,cleanerBotE_1"
        ;;
    campus_pybullet)
        EXPECTED_ROBOTS="deliveryRobot_1,deliveryRobot_2,deliveryRobot_3"
        ;;
    battle_royale_pybullet)
        EXPECTED_ROBOTS="tinyRobotA,tinyRobot_1,tinyRobot_2,tinyRobot_3"
        ;;
    *)
        EXPECTED_ROBOTS=""
        ;;
esac

if [ "$READY_ONLY" = true ]; then
    echo "=== RMF readiness: ${LAUNCH} (${CLIENT_MODE}) ==="
else
    echo "=== RMF dispatch E2E: ${LAUNCH} (${CLIENT_MODE}) — ${SCENARIOS[*]} ==="
fi

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/pybullet_fleet_ros_logs}"
mkdir -p "$ROS_LOG_DIR"

source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
source "${RMF_SETUP:-/rmf_demos_ws/install/setup.bash}"
if [ -n "${PBF_ROS_SETUP:-}" ]; then
    source "$PBF_ROS_SETUP"
fi
if [ -n "${PBF_REPO_ROOT:-}" ]; then
    export PYTHONPATH="${PBF_REPO_ROOT}:${PYTHONPATH:-}"
fi
if [ -n "${PBF_VENV:-}" ]; then
    PBF_VENV_SITE="$("$PBF_VENV/bin/python" -c "import sysconfig; print(sysconfig.get_paths()['purelib'])")"
    export PYTHONPATH="${PBF_VENV_SITE}:${PYTHONPATH:-}"
fi

LAUNCH_LOG="/tmp/${LAUNCH}_launch.log"
# RMF's schedule node writes a local persistence file by default. Reusing it
# across sequential smoke/e2e runs can replay old tasks and lift requests into
# the next launch, so keep each scripted run isolated.
rm -f .rmf_schedule_node.yaml
if [ -n "${PBF_REPO_ROOT:-}" ]; then
    rm -f "${PBF_REPO_ROOT}/.rmf_schedule_node.yaml"
fi
ros2 launch pybullet_fleet_rmf "${LAUNCH}.launch.py" \
    gui:=false headless:=true target_rtf:=1.0 client_mode:="$CLIENT_MODE" > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

cleanup() {
    echo "--- Shutting down demo (pid $LAUNCH_PID) ---"
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    sleep 3
    kill -9 "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT

echo "--- ${LAUNCH} launched (pid $LAUNCH_PID); running dispatch checker ---"

# rmf_dispatch_check.py waits for the fleet to come up itself.
set +e
if [ "$READY_ONLY" = true ]; then
    python3 "$CHECKER" --ready-only --expected-robots "$EXPECTED_ROBOTS"
else
    python3 "$CHECKER" --expected-robots "$EXPECTED_ROBOTS" "${SCENARIOS[@]}"
fi
RC=$?
set -e

if [ "$RC" -ne 0 ]; then
    echo "--- Dispatch E2E FAILED; launch log tail: ---"
    tail -n 40 "$LAUNCH_LOG" || true
    echo ""
    if [ "$READY_ONLY" = true ]; then
        echo "=== RMF readiness (${LAUNCH}, ${CLIENT_MODE}) FAILED ==="
    else
        echo "=== RMF dispatch E2E (${LAUNCH}, ${CLIENT_MODE}) FAILED ==="
    fi
    exit 1
fi

echo ""
if [ "$READY_ONLY" = true ]; then
    echo "=== RMF readiness (${LAUNCH}, ${CLIENT_MODE}) PASSED ==="
else
    echo "=== RMF dispatch E2E (${LAUNCH}, ${CLIENT_MODE}) PASSED ==="
fi
