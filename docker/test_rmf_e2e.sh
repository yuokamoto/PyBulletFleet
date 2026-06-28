#!/bin/bash
# docker/test_rmf_e2e.sh <launch_stem> <scenario> [<scenario> ...]
#
# Parametrized RMF dispatch end-to-end test. Launches an RMF demo headless and
# runs rmf_dispatch_check.py with the given scenarios; each dispatched task must
# reach `completed`. Used for several demo/task combinations from bridge.yml:
#
#   bash test_rmf_e2e.sh office_pybullet \
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

LAUNCH="$1"
shift
SCENARIOS=("$@")

if [ -z "$LAUNCH" ] || [ "${#SCENARIOS[@]}" -eq 0 ]; then
    echo "usage: test_rmf_e2e.sh <launch_stem> <scenario> [<scenario> ...]" >&2
    exit 2
fi

echo "=== RMF dispatch E2E: ${LAUNCH} — ${SCENARIOS[*]} ==="

source /opt/ros/jazzy/setup.bash
source /rmf_demos_ws/install/setup.bash

LAUNCH_LOG="/tmp/${LAUNCH}_launch.log"
ros2 launch pybullet_fleet_rmf "${LAUNCH}.launch.py" \
    gui:=false headless:=true target_rtf:=1.0 > "$LAUNCH_LOG" 2>&1 &
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
python3 /rmf_dispatch_check.py "${SCENARIOS[@]}"
RC=$?
set -e

if [ "$RC" -ne 0 ]; then
    echo "--- Dispatch E2E FAILED; launch log tail: ---"
    tail -n 40 "$LAUNCH_LOG" || true
    echo ""
    echo "=== RMF dispatch E2E (${LAUNCH}) FAILED ==="
    exit 1
fi

echo ""
echo "=== RMF dispatch E2E (${LAUNCH}) PASSED ==="
