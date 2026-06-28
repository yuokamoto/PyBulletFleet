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

echo "=== RMF integration smoke: stack up + bridge executes nav ==="

source /opt/ros/jazzy/setup.bash
source /rmf_demos_ws/install/setup.bash

# Fully headless: gui:=false (PyBullet DIRECT), headless:=true (no rviz).
# target_rtf:=1.0 paces sim at real time (the office YAML default leaves RTF
# uncapped, which would run a headless sim ~10x and desync RMF).
LAUNCH_LOG=/tmp/office_launch.log
ros2 launch pybullet_fleet_rmf office_pybullet.launch.py \
    gui:=false headless:=true target_rtf:=1.0 > "$LAUNCH_LOG" 2>&1 &
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
python3 /rmf_smoke_check.py
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
