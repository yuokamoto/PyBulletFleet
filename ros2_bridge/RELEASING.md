# ROS 2 Apt Release Guide

This guide covers the release of the ROS 2 packages in this repository through
rosdistro and Bloom. It is independent from the PyPI release of the simulation
core.

## Package And Tag Model

| Item | Value |
|---|---|
| rosdistro repository key | `pybulletfleet` |
| ROS packages | `pybullet_fleet_msgs`, `pybullet_fleet_ros`, `pybullet_fleet_rmf` |
| Core/PyPI tags | `vX.Y.Z` |
| ROS source tags | `vX.Y.Z-ros2` |
| Jazzy release repository | `yuokamoto/pybullet_fleet-release` |

The source branch is the mutable reference used by rosdistro indexing. The ROS
source tag is the immutable revision consumed by Bloom. Keep the ROS and core
tags separate while their versions and release schedules are independent.

## Before An Initial Release

1. Set the same ROS version in the three `package.xml` files.
2. Add a matching top-level release entry to each `CHANGELOG.rst`.
3. Confirm package metadata and dependencies:

   ```bash
   source /opt/ros/jazzy/setup.bash
   rosdep check --from-paths ros2_bridge --ignore-src --rosdistro jazzy
   colcon build --base-paths ros2_bridge
   ```

4. Run the bridge/RMF smoke test appropriate to the changes and run
   `make verify`. The CI bridge workflow also builds a local APT repository and
   runs a clean `apt install` bridge/RMF runtime gate; it must be green before
   publishing a preview repository or Bloom release.
5. Create the ROS release branch and immutable `*-ros2` source tag. Do not
   include Bloom-generated `debian/` directories in this repository.

The ROS packages intentionally do not declare `pybullet` or `pybullet-fleet`
in `package.xml`. The core is installed separately from PyPI at runtime.

## Index The Source Repository

Fork `ros/rosdistro` and add a source/doc entry to
`jazzy/distribution.yaml`. The entry key must be `pybulletfleet`: for GitHub
repositories, rosdistro uses the lower-case repository name without the owner.

Open a PR to `ros/rosdistro` using the documentation-index template. Wait for
that PR to merge before running Bloom. This source-entry PR only indexes the
repository; it does not make apt packages available.

## Generate The Bloom Release

The first run needs GitHub credentials that can push to the release repository
and open a rosdistro PR. Configure the Jazzy Bloom track as follows:

| Setting | Value |
|---|---|
| Repository name | `pybulletfleet` |
| Source URL | `https://github.com/yuokamoto/PyBulletFleet.git` |
| Source version | ROS `*-ros2` tag |
| Release URL | `https://github.com/yuokamoto/pybullet_fleet-release.git` |
| Release tag | `release/jazzy/{package}/{version}` |

Run the release from a clean checkout:

```bash
bloom-release pybulletfleet --ros-distro jazzy --track jazzy --new-track
```

Bloom creates release branches in the release repository and opens or prepares
a separate rosdistro release PR. Review the generated release entry before
submitting it. It must include all three sibling ROS packages from the same
source tag.

## Validate ros-testing

After the release PR merges and packages reach ros-testing, test the actual
Debian packages in a clean Jazzy environment:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-pybullet-fleet-ros \
  ros-jazzy-pybullet-fleet-msgs \
  python3-pip python3-dev build-essential
python3 -m pip install --user --break-system-packages pybullet-fleet

source /opt/ros/jazzy/setup.bash
python3 -c 'import pybullet, pybullet_fleet; print(pybullet.__file__)'
ros2 run pybullet_fleet_ros bridge_node --ros-args \
  -p config_yaml:="$(ros2 pkg prefix pybullet_fleet_ros)/share/pybullet_fleet_ros/config/bridge_test.yaml"
```

The bridge should start with three robots. Stop the smoke test with `Ctrl-C`.
The `--user --break-system-packages` installation is intentional: ROS apt
console scripts use `/usr/bin/python3`, so a package installed only in an
activated virtual environment is not visible to `ros2 run`.

Install `ros-jazzy-pybullet-fleet-rmf` for the RMF adapter. RMF demo launch
files also require a source-built `rmf_demos` overlay that provides
`rmf_demos` and `rmf_demos_maps`.
