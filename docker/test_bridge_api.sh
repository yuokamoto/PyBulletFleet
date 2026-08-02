#!/bin/bash
# docker/test_bridge_api.sh
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

source_ros_env
# Source-tree Docker checks use repository-relative assets. The APT runtime
# gate deliberately has no checkout, so it must use installed package paths.
if [ "${PBF_APT_RUNTIME:-false}" != "true" ]; then
    bridge_repo_root
fi

# Start bridge in background.
# The bridge is config_yaml-driven (num_robots/robot_urdf were removed); spawn
# the three test robots (robot0/1/2) from bridge_test.yaml.
TEST_CONFIG=${TEST_CONFIG:-}
if [ -z "$TEST_CONFIG" ]; then
    PKG_PREFIX=$(ros2 pkg prefix pybullet_fleet_ros)
    TEST_CONFIG="${PKG_PREFIX}/share/pybullet_fleet_ros/config/bridge_test.yaml"
fi
start_bridge_node "$TEST_CONFIG" false 10.0

cleanup() {
    # Force-kill and don't `wait` — bridge_node does not always exit on a plain
    # SIGTERM, and `wait` on it would hang the test after all checks passed.
    stop_bridge_node "$BRIDGE_PID"
}
trap cleanup EXIT

CHECKER=${BRIDGE_API_CHECK:-${INTEGRATION_CHECK:-/bridge_api_check.py}}
if [ ! -f "$CHECKER" ]; then
    CHECKER=/docker/bridge_api_check.py
fi
if [ ! -f "$CHECKER" ]; then
    CHECKER=docker/bridge_api_check.py
fi
if [ ! -f "$CHECKER" ]; then
    CHECKER=/opt/pybullet_fleet/docker/bridge_api_check.py
fi
python3 "$CHECKER"

trap - EXIT
cleanup

echo ""
echo "=== All integration tests PASSED ==="
