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

Start a scene with two independently exposed Fleet managers (`delivery_fleet`
and `inspection_fleet`) plus an unexposed `npc` manager. This fixed,
copyable example is the simplest way to verify selective Fleet ROS endpoints.
The inspection fleet uses TurtleBot3 Burger, so install its description first:

```bash
sudo apt install -y ros-jazzy-turtlebot3-description
```

```bash
ros2 launch pybullet_fleet_ros multi_manager_fleet_demo.launch.py gui:=true
```

On WSLg, RViz is often lighter than the PyBullet GUI. Install it once and use
the fleet marker view instead:

```bash
sudo apt install -y ros-jazzy-rviz2
ros2 launch pybullet_fleet_ros multi_manager_fleet_demo.launch.py \
  gui:=false rviz:=true target_rtf:=1.0
```

The RViz view subscribes to `/fleet/delivery_fleet/states` by default and
renders lightweight cubes on `/fleet/markers`. Set
`view_manager:=inspection_fleet` to inspect the other exposed fleet. The five
NPCs are deliberately absent from every continuous FleetState stream.

The bundled example also retains a filtered compatibility `/fleet/states`
snapshot for clients that need both external fleets. Remove `states: true` and
`state_scope` / `state_include_managers` from its `fleet_api` section when
deploying only the two manager-scoped streams.

From a second terminal, source Jazzy and send one request for five delivery robots. The
client reads their current poses, then applies the same offset to every goal,
so the grid spacing is preserved:

```bash
source /opt/ros/jazzy/setup.bash
ros2 run pybullet_fleet_ros fleet_nav_demo -- \
  --fleet-namespace /fleet/delivery_fleet \
  --robots 5 --dx 0.5 --dy 0.5 --transport service
```

For a ROS-level view of the complete request, this equivalent direct service
call shows every delivery goal in one command. It shifts the five delivery
robots by `(+0.5, +0.5)` while preserving their grid layout.

```bash
ros2 service call /fleet/delivery_fleet/navigate pybullet_fleet_msgs/srv/FleetNavigate \
  "{command_id: presentation_grid_shift, source: quickstart, goals_2d: [
    {name: delivery_robot_0, position: [0.5, 0.5], yaw: 0.0, z: 0.05},
    {name: delivery_robot_1, position: [2.5, 0.5], yaw: 0.0, z: 0.05},
    {name: delivery_robot_2, position: [4.5, 0.5], yaw: 0.0, z: 0.05},
    {name: delivery_robot_3, position: [6.5, 0.5], yaw: 0.0, z: 0.05},
    {name: delivery_robot_4, position: [8.5, 0.5], yaw: 0.0, z: 0.05}
  ], goals_3d: []}"
```

Use `gui:=false` on a headless system. The multi-manager launch's robot models
are fixed by its copyable YAML configuration.

### Alternative: configurable single fleet

After stopping the multi-manager launch, this separate demo exposes the global
`/fleet/*` endpoints and lets you choose a fleet size and robot model. The
default cube model needs no TurtleBot3 package:

```bash
ros2 launch pybullet_fleet_ros fleet_demo.launch.py \
  robots:=10 robot_model:=simple_cube gui:=true
```

From another terminal, verify the global state stream and command five robots:

```bash
ros2 topic echo /fleet/states --once
ros2 run pybullet_fleet_ros fleet_nav_demo -- \
  --robots 5 --dx 0.5 --dy 0.5 --transport service
```

`fleet_demo.launch.py` accepts `robots`, `robot_model`, `gui`, `rviz`, and
`target_rtf`. Its supported models are `simple_cube`, `mobile_robot`,
`tb3_burger`, and `tb3_waffle`; install `ros-jazzy-turtlebot3-description`
before selecting a TurtleBot3 model.

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
