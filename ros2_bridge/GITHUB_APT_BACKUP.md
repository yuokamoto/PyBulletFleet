# GitHub APT Supplement Repository

This repository supplements the official rosdistro/Bloom release described in
[RELEASING.md](RELEASING.md). Official ROS packages are the default path. The
GitHub APT repository remains useful for preview builds, fixed demo versions,
and releases that have not yet reached the ROS APT repository.

## Scope

The fallback publishes amd64 packages for:

- `ros-jazzy-pybullet-fleet-msgs`
- `ros-jazzy-pybullet-fleet-ros`
- `ros-jazzy-pybullet-fleet-rmf` when built with `--include-rmf`

`pybullet_fleet_rmf` is an integration package, not an APT-only RMF demo. The
office, hotel, airport, clinic, campus, and battle-royale launch files need
`rmf_demos` and `rmf_demos_maps` from a source overlay. Although
`rmf_fleet_adapter_python` is available from the official Jazzy APT repository,
the office/hotel demo launch files still require those missing demo packages.
Use the source overlay or the tested Docker workflow for an RMF dispatch demo.

The PyBulletFleet core also remains a separate PyPI dependency. Install it for
the system interpreter used by `ros2 run`. ROS APT console scripts use
`/usr/bin/python3`, so they cannot import a core package installed only in an
activated virtual environment.

## Coexistence With Official ROS Packages

Do not enable this repository and the official ROS repository for the same
PyBulletFleet package version unless an APT pinning policy is configured. Both
repositories use the same Debian package names, so APT may otherwise select an
unexpected source. Publish a distinct Debian revision for preview builds and
document its pin priority before recommending a long-lived installation.

The public repository must publish its supported ROS/Ubuntu/architecture
matrix, source tag and commit, checksums, and update or withdrawal policy. Use
HTTPS and signed APT metadata for general-purpose or long-lived use.

## Build The Flat Repository

From a checkout containing the immutable ROS source tag:

```bash
scripts/build_ros2_jazzy_demo_apt.sh --output /tmp/pybulletfleet-apt-repo
```

The script creates a detached clean worktree from `v0.1.1-ros2`, builds in the
existing `pybullet-fleet-rmf:jazzy` image, and writes `.deb` files, `Packages`,
`Packages.gz`, `SHA256SUMS`, and `BUILD_INFO.txt` to the specified empty output
directory. It does not modify the active worktree or commit Bloom-generated
`debian/` files. Each generated `.deb` includes the Apache-2.0 license text in
its Debian copyright documentation.

`--include-rmf` additionally builds `ros-jazzy-pybullet-fleet-rmf`. Publish it
only with source-overlay or Docker instructions that supply a compatible
`rmf_demos` and `rmf_demos_maps` environment.

Publish that directory as the root of a GitHub Pages repository. Verify the
published `SHA256SUMS` against the local output before configuring a demo
machine.

## Demo-Machine Install

For a time-limited demo, HTTPS plus the published checksum are mandatory
compensating controls. `[trusted=yes]` is not suitable for a general-purpose or
long-lived package repository; sign its APT metadata before using it that way.

```bash
echo 'deb [trusted=yes] https://OWNER.github.io/REPOSITORY/ ./' \
  | sudo tee /etc/apt/sources.list.d/pybulletfleet-demo.list
sudo apt update
sudo apt install -y ros-jazzy-pybullet-fleet-ros

python3 -m pip install --user --break-system-packages pybullet-fleet==0.7.0
source /opt/ros/jazzy/setup.bash
ros2 run pybullet_fleet_ros bridge_node --ros-args \
  -p config_yaml:="$(ros2 pkg prefix pybullet_fleet_ros)/share/pybullet_fleet_ros/config/bridge_test.yaml"
```

`ros-jazzy-pybullet-fleet-msgs` is installed automatically by the bridge
package dependency. Add `ros-jazzy-pybullet-fleet-rmf` for the adapter, then
follow the source-overlay or Docker RMF demo instructions. Stop the bridge
smoke test with `Ctrl-C`.

## Verify Before The Demo

Use a clean Jazzy container or VM. Confirm all of the following:

```bash
ros2 pkg prefix pybullet_fleet_msgs
ros2 pkg prefix pybullet_fleet_ros
python3 -c 'import pybullet, pybullet_fleet; print(pybullet.__file__)'
```

Start the bridge smoke command above with `timeout`; exit status `124` is
expected when the healthy node remains running until the timeout.
