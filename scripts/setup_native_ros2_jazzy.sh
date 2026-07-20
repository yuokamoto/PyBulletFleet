#!/usr/bin/env bash
# Set up a native ROS 2 Jazzy workspace for PyBulletFleet ROS/RMF development.
#
# This script intentionally does not install apt packages. Install ROS 2 Jazzy,
# Open-RMF/rmf_demos, colcon, and the package dependencies first, then run this
# script to create a symlinked overlay workspace and build the local bridge
# packages.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"
RMF_WS="${RMF_WS:-${HOME}/rmf_demos_ws}"
RMF_SETUP="${RMF_SETUP:-${RMF_WS}/install/setup.bash}"
PBF_ROS_WS="${PBF_ROS_WS:-${REPO_ROOT}/.ros2_ws}"
PBF_VENV="${PBF_VENV:-${REPO_ROOT}/.venv}"

if [ ! -f "$ROS_SETUP" ]; then
    echo "ROS setup not found: $ROS_SETUP" >&2
    echo "Install ROS 2 Jazzy first, or set ROS_SETUP=/path/to/setup.bash." >&2
    exit 1
fi

if [ ! -f "$RMF_SETUP" ]; then
    echo "RMF setup not found: $RMF_SETUP" >&2
    echo "Install/build rmf_demos, or set RMF_SETUP=/path/to/rmf_demos_ws/install/setup.bash." >&2
    exit 1
fi

if ! command -v colcon >/dev/null 2>&1; then
    echo "colcon not found. Install python3-colcon-common-extensions." >&2
    exit 1
fi

if [ ! -d "$PBF_VENV" ]; then
    python3 -m venv "$PBF_VENV"
fi

"$PBF_VENV/bin/python" -m pip install -U pip
"$PBF_VENV/bin/python" -m pip install -e "${REPO_ROOT}[sdf,dev]" "numpy<2" "pytest<9" catkin_pkg empy lark
PBF_VENV_SITE="$("$PBF_VENV/bin/python" -c "import sysconfig; print(sysconfig.get_paths()['purelib'])")"

mkdir -p "$PBF_ROS_WS/src"
ln -sfn "$REPO_ROOT/ros2_bridge/pybullet_fleet_msgs" "$PBF_ROS_WS/src/pybullet_fleet_msgs"
ln -sfn "$REPO_ROOT/ros2_bridge/pybullet_fleet_ros" "$PBF_ROS_WS/src/pybullet_fleet_ros"
ln -sfn "$REPO_ROOT/ros2_bridge/pybullet_fleet_rmf" "$PBF_ROS_WS/src/pybullet_fleet_rmf"
rm -rf "$PBF_ROS_WS/build" "$PBF_ROS_WS/install" "$PBF_ROS_WS/log"

set +u
source "$ROS_SETUP"
source "$RMF_SETUP"
set -u

cd "$PBF_ROS_WS"
colcon build --symlink-install \
    --packages-select pybullet_fleet_msgs pybullet_fleet_ros pybullet_fleet_rmf \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE="$PBF_VENV/bin/python"

cat > "${REPO_ROOT}/.ros2_native_env" <<EOF
# Source this file from the PyBulletFleet repo root for native ROS/RMF work.
source "$ROS_SETUP"
source "$RMF_SETUP"
source "$PBF_ROS_WS/install/setup.bash"
export PYTHONPATH="$REPO_ROOT:$PBF_VENV_SITE:\${PYTHONPATH:-}"
source "$PBF_VENV/bin/activate"
export PBF_REPO_ROOT="$REPO_ROOT"
export PBF_ROS_WS="$PBF_ROS_WS"
export PBF_VENV="$PBF_VENV"
export PBF_ROS_SETUP="$PBF_ROS_WS/install/setup.bash"
export RMF_WS="$RMF_WS"
export ROS_SETUP="$ROS_SETUP"
export RMF_SETUP="$RMF_SETUP"
EOF

echo ""
echo "Native ROS workspace ready:"
echo "  workspace: $PBF_ROS_WS"
echo "  env file:  $REPO_ROOT/.ros2_native_env"
echo ""
echo "Next:"
echo "  source .ros2_native_env"
echo "  pytest ros2_bridge/pybullet_fleet_rmf/test/test_rmf_launch_config.py -q"
