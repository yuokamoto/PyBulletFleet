---
name: ros2-releasing
description: Use when indexing, Bloom-releasing, or validating PyBulletFleet ROS 2 packages for apt on a supported ROS distribution.
---

# ROS 2 Apt Release

Use this skill for `pybullet_fleet_msgs`, `pybullet_fleet_ros`, and
`pybullet_fleet_rmf`. Read `ros2_bridge/RELEASING.md` for the maintainer-facing
workflow and exact commands.

## Release Model

- rosdistro repository key: `pybulletfleet`.
- ROS package names remain `pybullet_fleet_*`.
- Core/PyPI tags (`vX.Y.Z`) and ROS source tags (`vX.Y.Z-ros2`) are separate.
- The release branch is the mutable rosdistro source reference; the ROS tag is
  the immutable Bloom source revision.
- Bloom releases all three packages from one source revision. Do not release
  sibling packages one at a time.

## Preconditions

1. Confirm every package has the intended version, maintainer, Apache-2.0
   license, dependencies, and `CHANGELOG.rst` entry.
2. Run `rosdep check --from-paths ros2_bridge --ignore-src --rosdistro <distro>`
   and the relevant native or Docker bridge/RMF smoke test.
3. Run `make verify` before pushing source, test, packaging, or documentation
   changes. If the environment prevents completion, report the exact stage.
4. Never declare `pybullet` or `pybullet-fleet` as a ROS dependency: the core
   is a separately installed PyPI runtime prerequisite.
5. Do not commit Bloom-generated `ros2_bridge/*/debian/` directories to the
   source repository.

## Initial Indexing

1. Create a source/doc entry in `<distro>/distribution.yaml` using the
   `pybulletfleet` key and an HTTPS source URL ending in `.git`.
2. Point rosdistro at the ROS release branch, open the source-entry PR, and
   wait for it to merge.
3. Do not run `bloom-release` until the source entry is merged. A source-entry
   approval is not an apt release.

## Bloom Release

After the source entry merges, configure the `jazzy` track with:

- source repository: `https://github.com/yuokamoto/PyBulletFleet.git`
- source version: the immutable `*-ros2` tag
- release repository: `https://github.com/yuokamoto/pybullet_fleet-release.git`
- release tag template: `release/jazzy/{package}/{version}`

Run:

```bash
bloom-release pybulletfleet --ros-distro jazzy --track jazzy --new-track
```

Use the generated rosdistro release PR. Verify that it lists all three ROS
packages and references only the intended ROS source tag. Never hand-author a
Bloom release entry unless recovering from a reviewed Bloom failure.

## Post-Release Validation

1. Wait for the release PR merge and ros-testing buildfarm jobs.
2. In a clean Jazzy environment, install the actual apt packages, then install
   `pybullet-fleet` for the system interpreter's user site with
   `python3 -m pip install --user --break-system-packages pybullet-fleet`.
3. Source `/opt/ros/jazzy/setup.bash` and start the installed bridge smoke
   configuration. Confirm the bridge node starts.
4. Treat RMF demos separately: they require a source-built `rmf_demos` overlay
   providing `rmf_demos` and `rmf_demos_maps`.
