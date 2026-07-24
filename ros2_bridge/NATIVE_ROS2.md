# Native ROS 2 Jazzy Setup

This is the non-Docker workflow for Ubuntu 24.04 + ROS 2 Jazzy users. Docker
remains the CI/reference environment, but native setup gives a much faster edit
and test loop when ROS/RMF are installed on the host.

## Prerequisites

Install ROS 2 Jazzy first. The official guide is:

- [ROS 2 Jazzy Ubuntu deb packages](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

For Ubuntu 24.04, the minimum one-command-at-a-time setup is:

```bash
sudo apt update
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y
sudo apt update
sudo apt install -y ros-dev-tools ros-jazzy-desktop
source /opt/ros/jazzy/setup.bash
```

If `ros-jazzy-desktop` is not found, your ROS apt source is probably missing.
Follow the official guide above, or use this fallback setup:

```bash
sudo apt install -y curl gnupg
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo nano /etc/apt/sources.list.d/ros2.list
```

Add this single line in the editor:

```text
deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main
```

Then update apt again:

```bash
sudo apt update
```

Then build or install an Open-RMF demos workspace that provides:

- `rmf_demos`
- `rmf_demos_maps`
- `rmf_demos_assets`
- `rmf_adapter`
- `rmf_fleet_msgs`, `rmf_door_msgs`, `rmf_lift_msgs`
- `rmf_dispenser_msgs`, `rmf_ingestor_msgs`

The helper script assumes:

```bash
/opt/ros/jazzy/setup.bash
~/rmf_demos_ws/install/setup.bash
```

Override them with `ROS_SETUP=/path/to/setup.bash` and
`RMF_SETUP=/path/to/setup.bash` if your paths differ.

The helper script installs the extra system packages used by the bridge and
builds the required `rmf_demos` packages:

```bash
scripts/setup_rmf_demos_jazzy.sh
```

If you prefer manual setup, install these packages before building the native
overlay:

```bash
sudo apt update
sudo apt install -y \
  python3-venv python3-pip python3-colcon-common-extensions python3-rosdep python3-tk \
  ros-jazzy-nav2-msgs \
  ros-jazzy-control-msgs \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-trajectory-msgs \
  ros-jazzy-diagnostic-msgs \
  ros-jazzy-simulation-interfaces \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-xacro \
  ros-jazzy-turtlebot3-description \
  ros-jazzy-ur-description \
  ros-jazzy-rmw-cyclonedds-cpp
```

Use Cyclone DDS for the RMF scale checks:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## Build The Native Overlay

From the repository root:

```bash
scripts/setup_native_ros2_jazzy.sh
source .ros2_native_env
```

Equivalent Make target:

```bash
make ros-native-setup
source .ros2_native_env
```

The script:

- installs `pybullet_fleet` into `.venv` with `[sdf,dev]` extras;
- pins `numpy<2` in the venv to match ROS 2 Jazzy/RMF C-extension expectations;
- creates `.ros2_ws/src` symlinks to the three ROS packages under
  `ros2_bridge/`;
- runs `colcon build --symlink-install`;
- writes `.ros2_native_env` for later shells.

If your RMF workspace is elsewhere:

```bash
RMF_WS=/path/to/rmf_demos_ws scripts/setup_native_ros2_jazzy.sh
source .ros2_native_env
```

## Unit Tests

After sourcing `.ros2_native_env`, ROS/RMF unit tests can be run directly:

```bash
pytest -q \
  ros2_bridge/pybullet_fleet_ros/test \
  ros2_bridge/pybullet_fleet_rmf/test
```

Targeted RMF tests:

```bash
pytest -q \
  ros2_bridge/pybullet_fleet_rmf/test/test_fleet_clients.py \
  ros2_bridge/pybullet_fleet_rmf/test/test_per_robot_ros_client.py \
  ros2_bridge/pybullet_fleet_rmf/test/test_rmf_adapter_actions.py \
  ros2_bridge/pybullet_fleet_rmf/test/test_rmf_adapter_plugin.py \
  ros2_bridge/pybullet_fleet_rmf/test/test_rmf_launch_config.py
```

## RMF Demo Checks

The Docker test wrappers also work natively after `.ros2_native_env` is sourced:

```bash
source .ros2_native_env

bash docker/test_rmf_stack.sh
bash docker/test_rmf_client_modes.sh
bash docker/test_rmf_client_modes.sh --full
```

Equivalent default matrix target:

```bash
make rmf-matrix
```

Individual checks:

```bash
bash docker/test_rmf_dispatch.sh --client-mode python_fleet office_pybullet \
  "patrol:lounge,coe" \
  "delivery:pantry,coke_dispenser,hardware_2,coke_ingestor"

bash docker/test_rmf_dispatch.sh --client-mode fleet_ros hotel_pybullet \
  "patrol:lobby,L2_room1;zrise=2.0" \
  "clean:clean_lobby"

bash docker/test_rmf_dispatch.sh --client-mode python_fleet --ready-only clinic_pybullet
```

## Launch Manually

Non-RMF bridge demos only need `pybullet_fleet_msgs` and `pybullet_fleet_ros`
from the native overlay:

```bash
source .ros2_native_env

ros2 launch pybullet_fleet_ros tb3_demo.launch.py gui:=false rviz:=false
ros2 launch pybullet_fleet_ros attach_demo.launch.py gui:=false
ros2 launch pybullet_fleet_ros ur5e_demo.launch.py gui:=false rviz:=false
```

The bridge API check also runs natively:

```bash
source .ros2_native_env
ROS_LOG_DIR=/tmp/pbf_ros_log bash docker/test_bridge_api.sh
```

Use `gui:=true` when running from a desktop/WSLg session.

```bash
source .ros2_native_env

ros2 launch pybullet_fleet_rmf office_pybullet.launch.py \
  gui:=false headless:=true target_rtf:=1.0
```

RMF demo launch files default to `client_mode:=python_fleet`. Compatibility
paths remain available:

```bash
ros2 launch pybullet_fleet_rmf office_pybullet.launch.py client_mode:=per_robot_ros
ros2 launch pybullet_fleet_rmf office_pybullet.launch.py client_mode:=fleet_ros
```

## Notes

- `python_fleet` runs RMF adapters as in-process bridge plugins sharing the same
  `sim_core`.
- `per_robot_ros` and `fleet_ros` keep the standalone `fleet_adapter` process
  path.
- Multi-fleet demos register one RMF adapter plugin per fleet in `python_fleet`
  mode.
- Docker is still useful for CI parity and clean-room verification.
