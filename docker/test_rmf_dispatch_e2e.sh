#!/bin/bash
# docker/test_rmf_dispatch_e2e.sh
# FULL RMF dispatch end-to-end test. Launches the office demo headless, then runs
# rmf_dispatch_check.py which dispatches a patrol and asserts a robot drives the
# whole chain: dispatch_patrol -> RMF bidding -> fleet adapter -> NavigateToPose
# -> bridge -> PyBullet -> odom, plus the task reaching `underway` on
# /task_state_update.
#
# NOTE ON RELIABILITY: this exercises RMF's dispatcher + traffic-schedule, which
# depend on a stable host clock and reliable DDS delivery. On a WSL2 host the
# default clock makes ~2.5s backward jumps (`timedatectl set-ntp false` stops
# them), and host-network DDS can drop the traffic-schedule mirror's large
# transient-local snapshots — both stall the dispatch. So this is wired into CI
# as NON-blocking (continue-on-error) and is primarily a manual/stable-env check.
# The deterministic gate is test_rmf_smoke.sh (no dispatcher/traffic dependency).
#
# Mounted into the container (cf. test_integration.sh):
#   docker compose run --rm --no-deps \
#     -v "$(pwd)/test_rmf_dispatch_e2e.sh:/test_rmf_dispatch_e2e.sh:ro" \
#     -v "$(pwd)/rmf_dispatch_check.py:/rmf_dispatch_check.py:ro" \
#     bridge bash /test_rmf_dispatch_e2e.sh
set -e

echo "=== RMF dispatch E2E: office demo, dispatch a patrol, assert robot moves ==="

source /opt/ros/jazzy/setup.bash
source /rmf_demos_ws/install/setup.bash

# Launch the office demo fully headless: gui:=false (PyBullet DIRECT, no window),
# headless:=true (skip rviz). The DataMonitor's tkinter thread will log a benign
# "no display" error under headless and is harmless — fleet_states still publishes.
#
# target_rtf:=1.0 is essential: the office YAML default leaves RTF uncapped, so a
# headless run (no GUI render to throttle it) advances sim time ~10x real time.
# RMF's fleet adapter uses /clock, and at 10x its planning perpetually "times out",
# so the robot never departs. Pacing at real time keeps RMF in lockstep (this is
# what GUI runs do implicitly).
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

echo "--- Office demo launched (pid $LAUNCH_PID); running dispatch checker ---"

set +e
python3 /rmf_dispatch_check.py
RC=$?
set -e

if [ "$RC" -ne 0 ]; then
    echo "--- Dispatch E2E FAILED; launch log tail: ---"
    tail -n 40 "$LAUNCH_LOG" || true
    echo ""
    echo "=== RMF dispatch E2E test FAILED ==="
    exit 1
fi

echo ""
echo "=== RMF dispatch E2E test PASSED ==="
