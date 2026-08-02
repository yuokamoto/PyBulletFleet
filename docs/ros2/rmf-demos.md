# RMF Demo Catalog

Complete [Run Your First RMF Demo](rmf-quickstart) before using these scenarios.
All entries require the `rmf_demos` overlay because it supplies the building
maps, navigation graphs, fleet configurations, and task tools. The examples
default to `client_mode:=python_fleet`; see [Open-RMF Integration](rmf) before
selecting a ROS-backed mode.

## Scenarios

| Scenario | Launch file | Fleets | Focus |
| --- | --- | ---: | --- |
| Office | `office_pybullet.launch.py` | 1 | First patrol, delivery, and basic RMF dispatch |
| Campus | `campus_pybullet.launch.py` | 1 | Outdoor single-fleet navigation without doors or lifts |
| Battle royale | `battle_royale_pybullet.launch.py` | 1 | Compact single-fleet map and task dispatch |
| Clinic | `clinic_pybullet.launch.py` | 2 | Multiple fleets, doors, and elevators |
| Hotel | `hotel_pybullet.launch.py` | 3 | Multiple fleets with door and lift coordination |
| Airport terminal | `airport_terminal_pybullet.launch.py` | 4 | Largest multi-fleet configuration and cleaning tasks |

Start with Office. The other scenarios reuse the same RMF infrastructure while
changing the building map, bridge world, navigation graphs, and fleet adapters.

## Native Jazzy

Source the ROS and `rmf_demos` overlays in every terminal. Replace
`office_pybullet.launch.py` with any catalog launch file:

```bash
source /opt/ros/jazzy/setup.bash
source ~/rmf_demos_ws/install/setup.bash
ros2 launch pybullet_fleet_rmf office_pybullet.launch.py
```

The command assumes the dedicated `~/rmf_demos_ws` overlay from the quickstart.
When `rmf_demos` was added to an existing workspace, source that workspace's
`install/setup.bash` instead.

For a headless simulator and RMF visualization, use `gui:=false`. To omit RViz
as well, add `headless:=true`:

```bash
ros2 launch pybullet_fleet_rmf hotel_pybullet.launch.py \
  gui:=false headless:=true
```

### Native simulator with RMF Web containers

To add the RMF Web dashboard without building a PyBulletFleet Docker image,
follow the optional Web stack step in [Run Your First RMF Demo](rmf-quickstart).
It pulls only the upstream RMF Web API and dashboard images. Set the same
`ROS_DOMAIN_ID` in every native terminal and pass
`server_uri:=ws://localhost:8000/_internal` to the scenario launch:

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source ~/rmf_demos_ws/install/setup.bash
ros2 launch pybullet_fleet_rmf office_pybullet.launch.py \
  gui:=false headless:=true target_rtf:=1.0 \
  server_uri:=ws://localhost:8000/_internal
```

The browser dashboard is then available at [http://localhost:3000](http://localhost:3000).

## Docker

Docker Compose bind-mounts the checkout for the bridge and RMF packages, so
clone the repository and build the image before running a scenario. The image
already contains the required `rmf_demos` overlay:

```bash
git clone https://github.com/yuokamoto/PyBulletFleet.git
cd PyBulletFleet/docker
docker compose build bridge
```

Then run a scenario by replacing the launch filename:

```bash
docker compose run --rm bridge \
  ros2 launch pybullet_fleet_rmf clinic_pybullet.launch.py \
  gui:=false headless:=true
```

### Full RMF Web stack

The Compose stack also starts the RMF Web API and dashboard. It is the most
direct Docker demonstration of a fleet that is both dispatched through RMF and
visible in a browser. Start the Office scenario with a headless simulator:

```bash
GUI=false RVIZ=false docker compose up --build
```

Open [http://localhost:3000](http://localhost:3000) for the RMF dashboard. The
API is available at [http://localhost:8000](http://localhost:8000). The bridge
automatically connects its fleet adapter to the API at
`ws://localhost:8000/_internal` when it is launched through Compose.

In a second terminal, dispatch a task through the already-running bridge:

```bash
cd docker
docker compose exec bridge \
  ros2 run rmf_demos_tasks dispatch_patrol \
  -p pantry lounge -n 3 --use_sim_time
```

The task and robot state should appear in the dashboard. Stop the full stack
with `docker compose down`. Set `GUI=true` or `RVIZ=true` when a local display
server is available. To start another catalog scenario, set `DEMO_WORLD` before
the command, for example `DEMO_WORLD=clinic GUI=false RVIZ=false docker compose up`.

## Dispatch Examples

Run task commands in a second terminal after sourcing the same native overlays,
or through another `docker compose run --rm bridge` command.

```bash
# Office patrol
ros2 run rmf_demos_tasks dispatch_patrol \
  -p pantry lounge -n 3 --use_sim_time

# Office delivery destination
ros2 run rmf_demos_tasks dispatch_go_to_place -- -p pantry

# Airport cleaning task
ros2 run rmf_demos_tasks dispatch_clean -cs zone_1 --use_sim_time
```

For multi-fleet scenarios, dispatch to the fleet expected by the map and task
configuration. For example, the airport terminal launch accepts a patrol for
the `tinyRobot` fleet:

```bash
ros2 run rmf_demos_tasks dispatch_patrol \
  -p A -n 1 -F tinyRobot --use_sim_time
```

The available waypoints, task types, and fleet names come from the pinned
`rmf_demos` version. Inspect its map and fleet configuration when adapting a
scenario rather than assuming Office waypoint names apply elsewhere.
