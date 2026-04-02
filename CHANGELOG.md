# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/),
and this project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

### Highlights

- **Robot model resolution** — `resolve_urdf("panda")` finds URDFs by name across local, pybullet_data, and robot_descriptions sources with auto-discovery fallback
- **Auto-detect robot profiles** — `auto_detect_profile()` introspects any URDF to determine robot type, joint layout, EE link, and arm reach
- **Controller refactor** — Extracted `Controller` class from `Agent`, reducing agent.py by ~500 lines
- **Entity registry** — `register_entity_class()` for custom spawn types via YAML config
- **Examples reorganization** — Demos organized into arm/, basics/, mobile/, scale/, models/ subdirectories

### Features

- `robot_models.py` — `resolve_urdf()` with 5-step resolution (user search paths → KNOWN_MODELS → pybullet_data scan → robot_descriptions scan → error)
- `auto_detect_profile()` — introspects URDF via PyBullet to produce `RobotProfile` (robot_type, joints, EE, arm_reach)
- `register_model()` / `unregister_model()` — runtime model registry management
- `add_search_path()` / `remove_search_path()` — custom URDF search directories
- `discover_models()` — enumerate all available models in pybullet_data or robot_descriptions
- `controller.py` — extracted `Controller` with `OmnidirectionalController`, `DifferentialController`
- `entity_registry.py` — `register_entity_class()` for extensible YAML-driven spawning
- `SimulationParams.enable_floor` — option to skip ground plane loading
- `config_utils.py` — improved YAML merge and validation
- `sim_object.py` / `tools.py` — enhanced pose utilities and mesh support
- `[models]` extra in pyproject.toml for `pip install pybullet-fleet[models]`

### New Examples

- `examples/models/` — `resolve_urdf_demo.py`, `model_catalog_demo.py`, `robot_descriptions_demo.py`
- All existing demos reorganized into `arm/`, `basics/`, `mobile/`, `scale/` subdirectories
- Updated camera setup and robot profile integration in arm demos

### Documentation

- Tutorial 6: Robot Models — resolution, auto-detect, search paths, auto-discovery
- Updated quickstart, architecture overview, spawning-objects docs
- Research report: OSS robot models survey (pybullet_data, robot_descriptions)
- Design specs: robot-models, controller-refactor, plugin-architecture, unified-spawn

### Testing

- 960 tests (up from 740), coverage 82%
- `test_robot_models.py` — 40+ tests covering resolution, auto-detect, search paths, register/unregister, auto-discovery
- `test_controller.py` — controller extraction tests
- `test_config_spawn.py` — YAML config and entity registry tests
- `test_entity_registry.py`, `test_sim_object.py`, `test_tools.py` — new test modules

### Performance

- No regressions from controller refactor

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
