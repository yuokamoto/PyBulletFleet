# Run Your First ROS 2 Demo

This page starts a small ROS 2 bridge demo, sends one command, and then points
to the fleet and RMF paths.  Use Docker first unless you already maintain a
native ROS 2 Jazzy overlay.

## Docker: TurtleBot3 navigation

The first terminal runs the bridge and visualizers.  From the repository root:

```bash
cd docker
docker compose build bridge
xhost +local:docker  # desktop Linux/WSLg only; omit for headless use
docker compose run --rm --name pbf_bridge bridge \
  ros2 launch pybullet_fleet_ros tb3_demo.launch.py gui:=true
```

In a second terminal, send a goal through the per-robot ROS API:

```bash
cd docker
docker exec pbf_bridge bash -c 'source /rmf_demos_ws/install/setup.bash && \
  ros2 topic pub --once /tb3_0/goal_pose geometry_msgs/PoseStamped \
    "{header: {frame_id: odom}, pose: {position: {x: 2.0, y: 1.0, z: 0.01}, orientation: {w: 1.0}}}"'
```

The robot rotates and drives to the goal.  Stop the first terminal with
`Ctrl-C`.  For a server without graphics, use `gui:=false rviz:=false` and
inspect `/tb3_0/odom` or `/tb3_0/diagnostics` instead.

## Next: fleet-scale commands

For one state stream and batched commands, start the fleet API demo instead of
per-robot endpoints:

```bash
cd docker
docker compose run --rm --name pbf_bridge -v "$(pwd):/docker:ro" \
  bridge bash /docker/run_fleet_api_demo.sh \
  --robots 100 --robot-model simple_cube --gui false --target-rtf 1.0
```

Then, in another terminal:

```bash
docker exec pbf_bridge bash -c 'source /rmf_demos_ws/install/setup.bash && \
  python3 /opt/pybullet_fleet/scripts/send_fleet_nav_goals.py \
  --robots 50 --dx 0.5 --transport service'
```

See [Bridge Interfaces](overview) for endpoint semantics and
[Bridge Configuration](configuration) for selecting the exported API groups.

## Next: Open-RMF

Docker includes the RMF demo assets.  Start the office demo and dispatch a
patrol from another terminal:

```bash
cd docker
docker compose run --rm bridge \
  ros2 launch pybullet_fleet_rmf office_pybullet.launch.py
```

```bash
cd docker
docker compose run --rm bridge \
  ros2 run rmf_demos_tasks dispatch_patrol -p pantry lounge -n 3
```

The RMF launch defaults to the in-process `python_fleet` path.  See
[Open-RMF](rmf) before choosing `fleet_ros` or `per_robot_ros`.

## Native Jazzy and apt installs

Native overlays and apt-installed bridge packages require the `pybullet-fleet`
Python package in the same Python environment used by ROS nodes.  Follow the
[native Jazzy setup](https://github.com/yuokamoto/PyBulletFleet/blob/main/ros2_bridge/NATIVE_ROS2.md)
or the [bridge installation reference](https://github.com/yuokamoto/PyBulletFleet/blob/main/ros2_bridge/README.md#jazzy-apt-installation),
then run the same launch files shown above.

For Docker GUI forwarding, image maintenance, automated checks, and the full
set of Docker-only commands, see the
[Docker bridge guide](https://github.com/yuokamoto/PyBulletFleet/blob/main/docker/README.md).
