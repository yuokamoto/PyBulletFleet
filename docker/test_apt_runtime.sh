#!/usr/bin/env bash
# Verify a clean Jazzy runtime can install and run the locally built APT packages.
# The base image provides only the official rmf_demos underlay; PyBulletFleet
# bridge/RMF packages must come from APT and the Python core from PBF_WHEEL.
set -euo pipefail

APT_REPO_DIR="${APT_REPO_DIR:-/apt-repo}"
PBF_WHEEL="${PBF_WHEEL:-}"
TEST_ROOT="${TEST_ROOT:-/opt/pbf-apt-test}"

if [[ -z "$PBF_WHEEL" ]]; then
    PBF_WHEEL="$(find /artifacts -maxdepth 1 -name 'pybullet_fleet-*.whl' -print -quit)"
fi

if [[ ! -f "$PBF_WHEEL" ]]; then
    echo "Python core wheel not found: $PBF_WHEEL" >&2
    exit 2
fi
if [[ ! -f "$APT_REPO_DIR/Packages.gz" ]]; then
    echo "APT repository metadata not found: $APT_REPO_DIR/Packages.gz" >&2
    exit 2
fi

cat >/etc/apt/sources.list.d/pybullet-fleet-local.list <<EOF
deb [trusted=yes] file:${APT_REPO_DIR} ./
EOF

apt-get update
apt-get install -y \
    ros-jazzy-pybullet-fleet-msgs \
    ros-jazzy-pybullet-fleet-ros \
    ros-jazzy-pybullet-fleet-rmf

# ROS console scripts use the system interpreter. Keep this explicit so the
# test catches accidental virtual-environment-only installs.
python3 -m pip install --break-system-packages "${PBF_WHEEL}[sdf]"

# ROS setup scripts reference optional variables that may be unset in a clean
# container. Match the native setup helper and relax nounset only while sourcing.
set +u
source /opt/ros/jazzy/setup.bash
source /rmf_demos_ws/install/setup.bash
set -u

for package in pybullet_fleet_msgs pybullet_fleet_ros pybullet_fleet_rmf; do
    prefix="$(ros2 pkg prefix "$package")"
    if [[ "$prefix" != /opt/ros/jazzy* ]]; then
        echo "$package resolved outside the APT install: $prefix" >&2
        exit 1
    fi
done

python3 - <<'PY'
import pybullet_fleet
from pybullet_fleet import sdf_loader

assert "/workspace" not in pybullet_fleet.__file__, pybullet_fleet.__file__
assert "/workspace" not in sdf_loader.__file__, sdf_loader.__file__
print(f"Python core installed from wheel: {pybullet_fleet.__file__}")
PY

export BRIDGE_TEST_LIB="$TEST_ROOT/bridge_test_lib.sh"
export BRIDGE_API_CHECK="$TEST_ROOT/bridge_api_check.py"
export PBF_APT_RUNTIME=true
bash "$TEST_ROOT/test_bridge_api.sh"

# This starts the installed Office launch and asserts RMF adapter/handler
# readiness. Source-image CI separately owns full task completion coverage.
bash "$TEST_ROOT/test_rmf_dispatch.sh" --ready-only office_pybullet

echo "=== APT bridge and RMF runtime checks PASSED ==="
