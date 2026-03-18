# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/),
and this project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

### Added

- **Kinematic robot arm control** — Fixed-base URDF arm with joint targets. Kinematic mode (`mass=0.0`) uses per-step interpolation at URDF velocity limits; physics mode uses PyBullet motor control. Joint position cache eliminates per-step `getJointState()` calls.
  - `Agent.from_urdf(use_fixed_base=True)` — fixed-base arm spawning
  - `set_joint_target()` / `set_all_joints_targets()` / `set_joint_target_by_name()` — transparent kinematic/physics joint control
  - `are_all_joints_at_targets()` / `are_joints_at_targets_by_name()` — joint convergence checking
  - Link-level object attachment via `attach_object(parent_link_index=...)`
- **`JointAction`** — Action that moves all joints to target positions. Completes when all joints are within `tolerance`. Works in both physics and kinematic modes.
- **Arm demos** — `pick_drop_arm_demo.py` (low-level callback), `pick_drop_arm_action_demo.py` (action queue), `pick_drop_arm_100robots_demo.py` (100 arms fleet)
- **Tutorial 4** — Arm Joint Control & Pick/Drop documentation (`docs/examples/arm-pick-drop.md`)
- **Inverse Kinematics (IK)** — Multi-seed iterative IK solver with current-joint and quartile seed strategies. Configurable via `IKParams` dataclass.
  - `Agent.move_end_effector(target_position, target_orientation)` — high-level EE position command
  - `Agent.are_ee_at_target()` — check whether EE has reached a Cartesian target
  - `Agent._get_end_effector_link_index()` — auto-detect or resolve EE link
  - `Agent.are_joints_at_targets()` — unified joint convergence check using `_last_joint_targets`
- **`PoseAction`** — Action that moves the end-effector to a Cartesian target via IK. Supports position-only and position+orientation modes.
- **`ee_target_position` parameter** for `PickAction` and `DropAction` — IK-based EE positioning via `PoseAction` sub-action, as alternative to `JointAction` pre-positioning. `continue_on_ik_failure` flag controls behaviour on unreachable targets.
- **`IKParams` dataclass** — Tunable IK solver parameters: `max_outer_iterations`, `convergence_threshold`, `max_inner_iterations`, `residual_threshold`, `reachability_tolerance`, `seed_quartiles`. Passed to `Agent.from_urdf(ik_params=...)`.
- **`_last_joint_targets` unification** — All joint-setting methods (`set_joint_target`, `set_all_joints_targets`, `move_end_effector`) record targets in `_last_joint_targets`. `are_joints_at_targets()` with no arguments checks these last-commanded targets.
- **EE control demos** — `pick_drop_arm_ee_demo.py` (low-level callback) and `pick_drop_arm_ee_action_demo.py` (action queue)
- **Tutorial 5** — End-Effector Control & IK documentation (`docs/examples/arm-ee-control.md`)

### Changed

- `are_all_joints_at_targets()` now logs a warning when called with no targets ever set (vacuous `True`)
- Architecture overview updated with IK methods, `PoseAction`, and `IKParams`

## v0.1.0 (2026-03-14)

Initial public release of PyBulletFleet — a kinematics-first simulation framework for large-scale multi-robot fleets.

### Highlights

- **Kinematics-first simulation** — Teleport-based stepping enables N× real-time execution for fleet-scale evaluation
- **Scalable to 1000+ agents** — Spatial-hash collision detection and shared-shape caching keep step times low
- **Action system** — Built-in MoveTo, Pick, Drop, and Wait actions with queue-based execution
- **Flexible physics** — Full PyBullet physics available per-scenario when needed (grasping, conveyor dynamics)

### Features

- Multi-robot simulation core with configurable physics/kinematics modes
- Agent and AgentManager with grid spawning, YAML-driven configuration
- Spatial-hash collision detection with configurable cell sizes and modes
- Differential drive motion with smooth Slerp rotation
- Action queue system (MoveTo, Pick, Drop, Wait) with status tracking
- SimObject base class for robots, structures, and custom objects
- Callback-driven simulation loop with pause/resume support
- Time and memory profiling built in
- URDF and mesh object support with shared-shape caching

### CI & Release Infrastructure

- GitHub Actions CI: lint, test matrix (Python 3.10/3.11/3.12), docs build, security audit, license check
- Automated PyPI release via tag push (Trusted Publisher / OIDC)
- Pre-release and publish scripts with semver validation
- Releasing Copilot skill for guided release orchestration

### Documentation

- Complete ReadTheDocs documentation: tutorials, API reference, architecture guide, how-to guides
- Zero Sphinx warnings — builds cleanly with `-W` flag
- Benchmark results and profiling guides

### Performance

- Quaternion Slerp replaces scipy dependency (7× rotation speedup)
- 1000 agents at 3.2× real-time, 31 ms/step (Intel i7-1185G7)
