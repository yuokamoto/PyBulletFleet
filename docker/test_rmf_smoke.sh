#!/bin/bash
# docker/test_rmf_smoke.sh
# Reliable RMF integration smoke test. Run inside the bridge container (image
# pybullet-fleet-rmf:jazzy). Thin wrapper: launches the office demo fully headless
# and runs rmf_smoke_check.py, which does all the asserting —
#   - the RMF protocol topics for every handler are present (door/lift/dispenser/
#     ingestor/fleet) -> the handlers loaded;
#   - the fleet adapter reports all robots, and a direct NavigateToPose goal drives
#     a robot (bridge execution contract).
#
# This is deterministic (no RMF dispatcher / traffic-schedule dependency), so it
# is the fast blocking RMF gate. The full dispatch chain is covered separately by
# test_rmf_e2e.sh (also blocking).
#
# Mounted into the container (cf. test_integration.sh):
#   docker compose run --rm --no-deps \
#     -v "$(pwd)/test_rmf_smoke.sh:/test_rmf_smoke.sh:ro" \
#     -v "$(pwd)/rmf_smoke_check.py:/rmf_smoke_check.py:ro" \
#     bridge bash /test_rmf_smoke.sh
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CHECKER="${ROOT_DIR}/rmf_smoke_check.py"
if [ ! -f "$CHECKER" ]; then
    CHECKER=/rmf_smoke_check.py
fi

usage() {
    echo "usage: test_rmf_smoke.sh [--client-mode per_robot_ros|fleet_ros|python_fleet]" >&2
}

CLIENT_MODE=python_fleet
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
        *)
            usage
            exit 2
            ;;
    esac
done

echo "=== RMF integration smoke: stack up + bridge executes nav (${CLIENT_MODE}) ==="

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

# Fully headless: gui:=false (PyBullet DIRECT), headless:=true (no rviz).
# target_rtf:=1.0 paces sim at real time (the office YAML default leaves RTF
# uncapped, which would run a headless sim ~10x and desync RMF).
LAUNCH_LOG=/tmp/office_launch.log
ros2 launch pybullet_fleet_rmf office_pybullet.launch.py \
    gui:=false headless:=true target_rtf:=1.0 client_mode:="$CLIENT_MODE" > "$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

cleanup() {
    echo "--- Shutting down demo (pid $LAUNCH_PID) ---"
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    sleep 3
    kill -9 "$LAUNCH_PID" 2>/dev/null || true
}
trap cleanup EXIT

echo "--- Office demo launched (pid $LAUNCH_PID); running smoke checker ---"

# rmf_smoke_check.py does all readiness waiting itself: it waits for the RMF
# protocol topics (fleet/door/lift/dispenser/ingestor), the fleet adapter, and
# odom, then drives a direct NavigateToPose.
set +e
python3 "$CHECKER"
RC=$?
set -e

if [ "$RC" -ne 0 ]; then
    echo "--- Smoke FAILED; launch log tail: ---"
    tail -n 40 "$LAUNCH_LOG" || true
    echo ""
    echo "=== RMF integration smoke FAILED ==="
    exit 1
fi

echo ""
echo "=== RMF integration smoke PASSED ==="
