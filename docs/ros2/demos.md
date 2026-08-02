# ROS 2 Demo Catalog

Complete [Run Your First ROS 2 Bridge](quickstart) before using these commands.
All native examples assume that `/opt/ros/jazzy/setup.bash` has been sourced.
Use `gui:=true` only with WSLg or another desktop display; otherwise use the
headless form shown where available.

| Demo | Purpose | Extra package | Launch command |
| --- | --- | --- | --- |
| TurtleBot3 | First visible mobile-robot bridge demo | `ros-jazzy-turtlebot3-description` | `tb3_demo.launch.py` |
| Navigation | Mobile robot navigation topics and actions | None | `nav_demo.launch.py` |
| Panda arm | Seven-joint trajectory control | None | `arm_demo.launch.py` |
| Attach | Attach/detach a nearby box through ROS services | `ros-jazzy-turtlebot3-description` | `attach_demo.launch.py` |
| Fleet ROS GUI | 100 robots with batched Fleet ROS commands | None | `fleet_demo.launch.py` |
| RMF office | RMF patrol and delivery integration | `ros-jazzy-pybullet-fleet-rmf` plus `rmf_demos` overlay | [RMF quickstart](rmf-quickstart) |

## TurtleBot3

```bash
sudo apt install -y ros-jazzy-turtlebot3-description
ros2 launch pybullet_fleet_ros tb3_demo.launch.py \
  model:=burger gui:=true rviz:=false
```

For headless verification, replace the final arguments with
`gui:=false rviz:=false`. The launch starts one robot named `tb3_0`.

## Navigation

Start a differential-drive robot named `robot0`:

```bash
ros2 launch pybullet_fleet_ros nav_demo.launch.py gui:=true rviz:=false
```

From a second terminal, send a goal:

```bash
ros2 topic pub --once /robot0/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: odom}, pose: {position: {x: 2.0, y: 1.0, z: 0.01}, orientation: {w: 1.0}}}"
```

## Panda arm

Start the fixed-base Franka Panda demo. It exposes `joint_states` and the
`follow_joint_trajectory` action under `panda0`:

```bash
ros2 launch pybullet_fleet_ros arm_demo.launch.py gui:=true rviz:=false
```

For RViz visualization, install `ros-jazzy-rviz2` and omit `rviz:=false`.

From another terminal, send a seven-joint trajectory and keep the feedback on
screen until the action completes:

```bash
ros2 action send_goal --feedback \
  /panda0/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7], points: [{positions: [0.2, -0.5, 0.2, -1.8, 0.0, 1.3, 0.6], time_from_start: {sec: 3}}]}}"
```

## Attach and detach

This demo starts `tb3_0` beside three pickable boxes. Attach `box_A`, move with
it attached, then detach it at the destination:

```bash
sudo apt install -y ros-jazzy-turtlebot3-description
ros2 launch pybullet_fleet_ros attach_demo.launch.py gui:=true
```

```bash
ros2 service call /tb3_0/toggle_attach std_srvs/srv/SetBool "{data: true}"
```

```bash
ros2 topic pub --once /tb3_0/goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: odom}, pose: {position: {x: 2.0, y: -1.0, z: 0.01}, orientation: {w: 1.0}}}"
```

After the robot reaches the goal, detach the box:

```bash
ros2 service call /tb3_0/toggle_attach std_srvs/srv/SetBool "{data: false}"
```

## Fleet ROS GUI demo

This native demo creates a fleet scene with the Fleet ROS API enabled, then
accepts one navigation request for every robot. It is the representative GUI
demo for the fleet ROS path:

```bash
ros2 launch pybullet_fleet_ros fleet_demo.launch.py \
  robots:=10 robot_model:=simple_cube gui:=true target_rtf:=1.0
```

For a lower-overhead WSLg view, install `ros-jazzy-rviz2` and launch with
`gui:=false rviz:=true`. It renders lightweight cubes from the single
`/fleet/states` stream rather than opening the PyBullet GUI.

From another terminal, inspect the batched state and move all ten robots while
preserving their grid spacing:

```bash
ros2 topic echo /fleet/states --once
python3 -m pybullet_fleet_ros.fleet_nav_demo \
  --robots 10 --dx 0.5 --dy 0.5 --transport service
```

The [bridge quickstart](quickstart) also includes a literal 10-robot
`ros2 service call` suited to a live API demonstration. Set
`robot_model:=tb3_burger` or `tb3_waffle` after installing
`ros-jazzy-turtlebot3-description`. The same launch works in Docker; see the
[Docker bridge guide](https://github.com/yuokamoto/PyBulletFleet/blob/main/docker/README.md).

## Docker-only model catalog

The model catalog launches every model available in the fully provisioned
Docker image, including optional ROS descriptions. It is not an APT preview
demo:

```bash
cd docker
docker compose run --rm bridge \
  ros2 launch pybullet_fleet_ros model_catalog_demo.launch.py gui:=true
```
