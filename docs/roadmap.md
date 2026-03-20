# Roadmap

Planned additions and improvements for PyBulletFleet.
Items are grouped by category; ordering within a group does not imply priority.

## Components

New simulation building blocks:

- ~~**Linear Joint**~~ ✅ Implemented in v0.1.0+ — `rail_arm_robot.urdf`, `JointAction` with per-joint tolerance, prismatic kinematic fallback

## Assets

New robot and infrastructure models:

- **Physics Mobile Robot** — Wheeled robot driven by PyBullet physics (motor torques, friction, contact forces)
- ~~**Kinematic Mobile Manipulator**~~ ✅ Implemented — `mobile_manipulator.urdf` works in kinematic mode (`mass=0.0`), IK auto-detects and locks wheel joints, `ik_joint_names` for explicit control
- **Physics Mobile Manipulator** — Physics-mode mobile manipulator with motor-driven base and arm
- **Conveyor / Elevator / Mobile Rack** — Warehouse infrastructure entities for material handling scenarios

## Features

- **Snapshot & Replay** — Full and delta snapshot serialization for logging, replay, and external synchronization ([USO](https://github.com/yuokamoto/Unified-Simulation-Orchestrator) integration)
- **Behavior tree integration** - Create agent behavior from behavior tree.

## Interfaces

External communication layers:

- **ROS 2** — Topic / service / action bridge for ROS 2 ecosystem integration
- **gRPC** — Language-agnostic RPC interface for orchestrators, WMS, and fleet managers

## Refactoring

- **Remove scipy dependency** — Currently only `scipy.spatial.transform.Rotation` is used (9 call sites for quat↔euler, quat↔matrix, relative rotation). Replace with PyBullet utilities + lightweight helpers in `geometry.py` to eliminate the ~150 MB transitive dependency. Low priority: no runtime performance impact, only install size.

## CI / DevOps

- **GitHub Actions refactoring** — Streamlined CI pipeline
- **Automated performance tracking** — Run time / memory benchmarks in CI, auto-update results in documentation, and alert on significant performance regressions
