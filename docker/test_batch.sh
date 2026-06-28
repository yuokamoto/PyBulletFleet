#!/bin/bash
# docker/test_batch.sh
# Pattern 2 (Batch API ROS Wrapper) smoke test. Run inside the bridge container.
# Builds the new fleet msgs, launches the bridge with fleet_interface: batch, and
# runs batch_check.py to verify /fleet/states + /fleet/navigate (service & topic).
#
# Mounted into the container:
#   docker compose run --rm --no-deps \
#     -v "$(pwd)/../ros2_bridge/pybullet_fleet_msgs:/rmf_demos_ws/src/pybullet_fleet_msgs:ro" \
#     -v "$(pwd)/test_batch.sh:/test_batch.sh:ro" \
#     -v "$(pwd)/batch_check.py:/batch_check.py:ro" \
#     bridge bash /test_batch.sh
set -e

echo "=== Pattern 2 (batch API) smoke test ==="

source /opt/ros/jazzy/setup.bash
source /rmf_demos_ws/install/setup.bash

# The prebuilt image predates the fleet msgs; rebuild pybullet_fleet_msgs from the
# mounted source so FleetState/FleetGoal/RobotGoal/FleetNavigate are available.
echo "--- Building pybullet_fleet_msgs (new fleet interfaces) ---"
cd /rmf_demos_ws
if ! colcon build --packages-select pybullet_fleet_msgs > /tmp/msgs_build.log 2>&1; then
    echo "✗ pybullet_fleet_msgs build FAILED"; tail -n 30 /tmp/msgs_build.log; exit 1
fi
source install/setup.bash
echo "  ✓ msgs built"

cd /opt/pybullet_fleet
TEST_CONFIG=/rmf_demos_ws/install/pybullet_fleet_ros/share/pybullet_fleet_ros/config/bridge_batch_test.yaml
ros2 run pybullet_fleet_ros bridge_node \
    --ros-args -p config_yaml:=$TEST_CONFIG -p gui:=false -p publish_rate:=20.0 &
BRIDGE_PID=$!
cleanup() { kill -INT "$BRIDGE_PID" 2>/dev/null || true; sleep 1; kill -9 "$BRIDGE_PID" 2>/dev/null || true; }
trap cleanup EXIT
sleep 5

echo "--- Running batch checker ---"
set +e
python3 /batch_check.py
RC=$?
set -e

if [ "$RC" -ne 0 ]; then
    echo "=== Pattern 2 batch smoke FAILED ==="
    exit 1
fi
echo ""
echo "=== Pattern 2 batch smoke PASSED ==="
