#!/bin/bash
# docker/test_integration.sh
# Run inside Docker container to verify full stack
set -e

echo "=== Integration Test: PyBulletFleet ROS 2 Bridge ==="

LIB=${BRIDGE_TEST_LIB:-/bridge_test_lib.sh}
if [ ! -f "$LIB" ]; then
    LIB=/docker/bridge_test_lib.sh
fi
if [ ! -f "$LIB" ]; then
    LIB=/opt/pybullet_fleet/docker/bridge_test_lib.sh
fi
source "$LIB"

# URDF paths are relative to pybullet_fleet root
source_ros_env
bridge_repo_root

# Start bridge in background.
# The bridge is config_yaml-driven (num_robots/robot_urdf were removed); spawn
# the three test robots (robot0/1/2) from bridge_test.yaml.
TEST_CONFIG=/rmf_demos_ws/install/pybullet_fleet_ros/share/pybullet_fleet_ros/config/bridge_test.yaml
start_bridge_node "$TEST_CONFIG" false 10.0

cleanup() {
    # Force-kill and don't `wait` — bridge_node does not always exit on a plain
    # SIGTERM, and `wait` on it would hang the test after all checks passed.
    stop_bridge_node "$BRIDGE_PID"
}
trap cleanup EXIT

CHECKER=${INTEGRATION_CHECK:-/integration_check.py}
if [ ! -f "$CHECKER" ]; then
    CHECKER=/docker/integration_check.py
fi
if [ ! -f "$CHECKER" ]; then
    CHECKER=/opt/pybullet_fleet/docker/integration_check.py
fi
python3 "$CHECKER"

trap - EXIT
cleanup

echo ""
echo "=== All integration tests PASSED ==="
