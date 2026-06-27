#!/bin/bash
# docker/test_rmf_smoke.sh
# Reliable RMF integration smoke test. Run inside the bridge container (image
# pybullet-fleet-rmf:jazzy). Launches the office demo fully headless and asserts:
#   - the RMF protocol topics for every handler are present (door/lift/dispenser/
#     ingestor/fleet) -> the handlers loaded and are publishing;
#   - rmf_smoke_check.py passes: the fleet adapter reports all robots, and a
#     direct NavigateToPose goal drives a robot (bridge execution contract).
#
# This is deterministic (no RMF dispatcher / traffic-schedule dependency), so it
# is the BLOCKING CI gate. The full dispatch->patrol chain is covered separately
# by test_rmf_dispatch_e2e.sh (non-blocking; needs a stable host clock + DDS).
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

echo "--- Office demo launched (pid $LAUNCH_PID); waiting for topics ---"

# Wait for the stack to advertise topics (up to ~100s), then assert the RMF
# protocol topics each handler owns are present.
for i in $(seq 1 50); do
    sleep 2
    ros2 topic list 2>/dev/null | grep -q /fleet_states && break
done

TOPICS=$(ros2 topic list 2>/dev/null)
echo "--- Checking RMF protocol topics (handlers up) ---"
for topic in /fleet_states /door_states /lift_states /dispenser_states /ingestor_states; do
    if echo "$TOPICS" | grep -q "$topic"; then
        echo "  ✓ $topic"
    else
        echo "  ✗ $topic MISSING"
        echo "--- launch log tail: ---"; tail -n 40 "$LAUNCH_LOG" || true
        echo "=== RMF integration smoke FAILED ==="
        exit 1
    fi
done

echo "--- Running smoke checker (fleet up + direct NavigateToPose) ---"
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
