# Run Your First ROS 2 Bridge

This guide uses the GitHub-hosted Jazzy APT preview on Ubuntu 24.04 or WSL.
It first installs and runs the Python simulation core, then adds the ROS 2
bridge. For Open-RMF, continue with [Run Your First RMF Demo](rmf-quickstart).

<div align="center">
<video controls preload="metadata" width="720">
  <source src="../fleet_interface_demo.mp4" type="video/mp4">
  <a href="../fleet_interface_demo.mp4">Download the fleet interface demo video</a>
</video>
</div>

This is the fleet-level ROS 2 demo described in step 4 below.

Before starting, install the base ROS 2 Jazzy distribution (at least
`ros-jazzy-ros-base`) using the [official Ubuntu binary installation
guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
The preview repository supplies PyBulletFleet packages; it does not replace
the ROS 2 repository.

The preview is for short-lived demo environments: its APT metadata is not yet
signed. Do not use `[trusted=yes]` for a general-purpose or long-lived system.

## 1. Install PyBulletFleet

Install the simulation core into the system Python user site. ROS console
scripts use `/usr/bin/python3`, so a package installed only in a virtual
environment will not be visible to `ros2 run`.

```bash
sudo apt update
sudo apt install -y curl python3-pip python3-dev build-essential
python3 -m pip install --user --break-system-packages pybullet-fleet==0.7.4
export PATH="$HOME/.local/bin:$PATH"
pybullet-fleet examples --list
```

For the optional DataMonitor tkinter window, install its system dependency:

```bash
sudo apt install -y python3-tk
```

With WSLg or another desktop display, run the bundled grid example:

```bash
pybullet-fleet examples --run 100robots_grid_demo.py
```

## 2. Add the Jazzy APT preview

Add the preview repository and install the bridge package. Its message package
is installed automatically.

```bash
echo 'deb [trusted=yes] https://yuokamoto.github.io/pybullet-fleet-apt/ ./' \
  | sudo tee /etc/apt/sources.list.d/pybulletfleet-preview.list
sudo apt update
sudo apt install -y ros-jazzy-pybullet-fleet-ros
source /opt/ros/jazzy/setup.bash
```

## 3. Run the TurtleBot3 demo

Install the TurtleBot3 description and start a Burger in the PyBullet GUI. This
is the first visible ROS 2 demo; it requires WSLg or another desktop display.

```bash
sudo apt install -y ros-jazzy-turtlebot3-description
ros2 launch pybullet_fleet_ros tb3_demo.launch.py \
  model:=burger gui:=true rviz:=false
```

Stop the demo with `Ctrl-C`. On a headless system, use `gui:=false rviz:=false`
and inspect ROS topics instead.

## 4. Run a Fleet ROS demo

Start a 100-robot scene using only the fleet-level ROS API. It is the native
equivalent of the Docker fleet demo and demonstrates one state stream and one
batched navigation request for the entire fleet.

```bash
ros2 launch pybullet_fleet_ros fleet_demo.launch.py \
  robots:=100 robot_model:=simple_cube gui:=true target_rtf:=1.0
```

On WSLg, RViz is often lighter than the PyBullet GUI. Install it once and use
the fleet marker view instead:

```bash
sudo apt install -y ros-jazzy-rviz2
ros2 launch pybullet_fleet_ros fleet_demo.launch.py \
  robots:=100 robot_model:=simple_cube gui:=false rviz:=true target_rtf:=1.0
```

The RViz view subscribes to the single `/fleet/states` stream and renders a
lightweight cube for each robot on `/fleet/markers`.

From a second terminal, source Jazzy and send one request for 10 robots. The
client reads their current poses, then applies the same offset to every goal,
so the grid spacing is preserved:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pybullet_fleet_ros.fleet_nav_demo \
  --robots 10 --dx 0.5 --dy 0.5 --transport service
```

The installed package also exposes the same client as
`ros2 run pybullet_fleet_ros fleet_nav_demo`.

For a presentation, this equivalent direct service call makes the complete
10-robot batch request visible in one command. It assumes the
`robot_model:=simple_cube` launch above: the goals shift the first ten robots
by `(+0.5, +0.5)` while preserving the 2 m grid spacing.

```bash
ros2 service call /fleet/navigate pybullet_fleet_msgs/srv/FleetNavigate \
  "{command_id: presentation_grid_shift, source: quickstart, goals_2d: [
    {name: robot_0, position: [0.5, 0.5], yaw: 0.0, z: 0.05},
    {name: robot_1, position: [2.5, 0.5], yaw: 0.0, z: 0.05},
    {name: robot_2, position: [4.5, 0.5], yaw: 0.0, z: 0.05},
    {name: robot_3, position: [6.5, 0.5], yaw: 0.0, z: 0.05},
    {name: robot_4, position: [0.5, 2.5], yaw: 0.0, z: 0.05},
    {name: robot_5, position: [2.5, 2.5], yaw: 0.0, z: 0.05},
    {name: robot_6, position: [4.5, 2.5], yaw: 0.0, z: 0.05},
    {name: robot_7, position: [6.5, 2.5], yaw: 0.0, z: 0.05},
    {name: robot_8, position: [0.5, 4.5], yaw: 0.0, z: 0.05},
    {name: robot_9, position: [2.5, 4.5], yaw: 0.0, z: 0.05}
  ], goals_3d: []}"
```

Use `gui:=false rviz:=false` on a headless system. Set `robot_model:=tb3_burger` or
`tb3_waffle` after installing `ros-jazzy-turtlebot3-description`.

## Optional: fleet smoke test

The installed three-robot configuration exposes the fleet API without a GUI.
It should remain running after reporting that it started with three robots.

```bash
ros2 run pybullet_fleet_ros bridge_node --ros-args \
  -p config_yaml:="$(ros2 pkg prefix pybullet_fleet_ros)/share/pybullet_fleet_ros/config/bridge_test.yaml"
```

See the [ROS 2 demo catalog](demos) for navigation, arm, attach, and fleet
examples, [Bridge Interfaces](overview) for the ROS topics, actions, and
services, or continue to [Run Your First RMF Demo](rmf-quickstart).

## Docker alternative

Docker provides a reproducible bridge and RMF environment, including RMF demo
assets. Use it when you do not want to configure a native Jazzy installation,
or when validating changes to bridge/RMF integration. Clone the repository and
build the image once before running a Docker demo:

```bash
git clone https://github.com/yuokamoto/PyBulletFleet.git
cd PyBulletFleet/docker
docker compose build bridge
```

Then run the bridge:

```bash
docker compose run --rm --name pbf_bridge bridge \
  ros2 launch pybullet_fleet_ros tb3_demo.launch.py gui:=false rviz:=false
```

In a second terminal, send a goal through the per-robot ROS API:

```bash
cd docker
docker exec pbf_bridge bash -c 'source /rmf_demos_ws/install/setup.bash && \
  ros2 topic pub --once /tb3_0/goal_pose geometry_msgs/PoseStamped \
    "{header: {frame_id: odom}, pose: {position: {x: 2.0, y: 1.0, z: 0.01}, orientation: {w: 1.0}}}"'
```

For GUI forwarding, fleet-scale commands, and Docker operations, see the
[Docker bridge guide](https://github.com/yuokamoto/PyBulletFleet/blob/main/docker/README.md).
