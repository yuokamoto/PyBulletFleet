# Run Your First RMF Demo

This guide extends the [ROS 2 bridge quickstart](quickstart). Complete its
core and bridge installation first, then add the RMF adapter and the
source-built `rmf_demos` overlay required by the demo maps and tasks.

## 1. Install the RMF adapter package

The RMF adapter is distributed from the same Jazzy APT preview repository as
the bridge. In a terminal where that repository has already been configured:

```bash
sudo apt update
sudo apt install -y ros-jazzy-pybullet-fleet-rmf
```

## 2. Add `rmf_demos` to a workspace

The Jazzy binary packages do not provide the `rmf_demos` and
`rmf_demos_maps` assets used by the office, hotel, airport, clinic, campus,
and battle-royale examples. Download the version-pinned setup script and build
the required packages. By default, it creates a dedicated
`~/rmf_demos_ws` overlay, which keeps the example dependencies isolated:

```bash
curl -fsSL \
  https://raw.githubusercontent.com/yuokamoto/PyBulletFleet/main/scripts/setup_rmf_demos_jazzy.sh \
  -o /tmp/setup_rmf_demos_jazzy.sh
bash /tmp/setup_rmf_demos_jazzy.sh --prefetch-rmf-assets
```

The script installs required Jazzy dependencies, clones `rmf_demos` 2.3.0, and
builds only the packages needed by these examples. It may take several minutes.

To use an existing colcon workspace instead, pass it with `--workspace`. The
script clones the sources under `<workspace>/src/rmf_demos` and preserves the
workspace's existing `build`, `install`, and `log` directories:

```bash
bash /tmp/setup_rmf_demos_jazzy.sh \
  --workspace ~/ros2_ws --prefetch-rmf-assets
```

Use the matching overlay path in later terminals. For the dedicated default,
source `~/rmf_demos_ws/install/setup.bash`; for the example above, source
`~/ros2_ws/install/setup.bash`.

## 3. Optional: start RMF Web without building a PyBulletFleet image

The RMF Web API and dashboard can run in Docker while the PyBulletFleet
simulation and RMF adapter run natively. This Compose file pulls only the
upstream RMF Web images; it does not build or distribute a PyBulletFleet image,
`rmf_demos`, or Gazebo Fuel assets.

Choose a ROS domain ID and use it for both the native terminals and the
containers. Download the Compose file, then start the Web stack:

```bash
export ROS_DOMAIN_ID=42
curl -fsSL \
  https://raw.githubusercontent.com/yuokamoto/PyBulletFleet/main/docker/docker-compose.rmf-web.yaml \
  -o /tmp/pybullet-fleet-rmf-web.yaml
docker compose -f /tmp/pybullet-fleet-rmf-web.yaml up -d
```

Open [http://localhost:3000](http://localhost:3000) for the dashboard. Stop it
after the demo with:

```bash
docker compose -f /tmp/pybullet-fleet-rmf-web.yaml down
```

## 4. Run an office patrol

In the first terminal, source both overlays and start the office scenario:

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source ~/rmf_demos_ws/install/setup.bash
ros2 launch pybullet_fleet_rmf office_pybullet.launch.py \
  gui:=true target_rtf:=1.0 \
  server_uri:=ws://localhost:8000/_internal
```

In a second terminal, source the same overlays and dispatch a patrol:

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash
source ~/rmf_demos_ws/install/setup.bash
ros2 run rmf_demos_tasks dispatch_patrol \
  -p pantry lounge -n 3 --use_sim_time
```

The adapter defaults to `python_fleet`, which keeps RMF command and state
exchange in-process. See [Open-RMF](rmf) for client modes, task behavior, and
scenario details. See the [RMF demo catalog](rmf-demos) after completing this
first patrol.

RMF world furniture is stored on Gazebo Fuel rather than in `rmf_demos`. The
setup command above prefetches assets for every supported RMF demo. To download
only Office assets, use `--prefetch-office-assets` instead. The full cache is
currently about 80 models and can take several minutes on the first run.

## Full Docker alternative

Docker already contains the compatible RMF demo overlay. Use it instead of the
native workspace when you want the most reproducible environment:

```bash
cd docker
docker compose run --rm bridge \
  ros2 launch pybullet_fleet_rmf office_pybullet.launch.py target_rtf:=1.0
```

In another terminal:

```bash
cd docker
docker compose run --rm bridge \
  ros2 run rmf_demos_tasks dispatch_patrol \
  -p pantry lounge -n 3 --use_sim_time
```
