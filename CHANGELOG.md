# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/),
and this project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

## v0.2.0 (2026-03-20)

### Highlights

- **IK end-effector control** — `Agent.move_end_effector()` → `PoseAction` → `IKParams` for Cartesian arm control
- **Prismatic (linear) joints** — Rail arm support with mixed revolute+prismatic chains
- **Mobile manipulator** — Arm-on-mobile-base with IK auto-locking wheels and kinematic EE tracking
- **AI-native DX** — Repository-level AI instructions and `Makefile` entry points

### Features

- `PoseAction` — move end-effector to Cartesian target; uses `Agent.move_end_effector()` with IK solved via `IKParams`
- `IKParams` dataclass for IK solver configuration (attached to `AgentSpawnParams`)
- `PickAction` / `DropAction` EE extensions via `ee_target_position` parameter
- `DropAction.drop_relative_pose` — offset from current object pose instead of absolute world position
- `JointAction` — per-joint tolerance (scalar, list, or dict) and prismatic joint support
- `Agent.joint_tolerance` — agent-level default with fallback chain (Action → Agent → 0.01)
- Prismatic joint kinematic fallback (0.5 m/s) alongside revolute (2.0 rad/s)
- Kinematic arm joint position cache for velocity interpolation
- `.github/copilot-instructions.md` — AI-readable project context (architecture, guard rails, patterns)
- Root `Makefile` with 11 targets (`make verify`, `make test`, `make bench-smoke`, etc.)

### New URDFs & Demos

- `robots/rail_arm_robot.urdf` — 1 prismatic + 4 revolute joints (5-DOF)
- New demo scripts: `pick_drop_arm_ee_demo`, `pick_drop_arm_ee_action_demo`, `rail_arm_demo`

### Bug Fixes

- Fix link-level object attachment (`computeForwardKinematics` for correct link poses)
- Fix packaging configuration for pip install compatibility
- Standardize collision method names to lowercase
- Resolve CI and type-checking issues

### Documentation

- Tutorials: EE/IK control, prismatic joints, mobile manipulator
- Design specs for all new features
- Updated architecture overview with IK and action system extensions

### Testing

- 740 tests (up from ~560), coverage 80%
- Mobile manipulator E2E tests with shared assertion helpers
- Parametrized arm tests across physics/kinematic/physics_off modes

### Performance

- No regressions: 1000 agents at 3.2× real-time, 31 ms/step

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
