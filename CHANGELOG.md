# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/),
and this project adheres to [Semantic Versioning](https://semver.org/).

## [Unreleased]

Packaging patch: the v0.4.0 wheel shipped **without** its bundled data, so
pip-installed users could not load the built-in robots, configs, or meshes.
This release makes that data ship in the wheel and resolve correctly.

### Bug Fixes

- Bundle the built-in `robots/`, `config/`, and `mesh/` data in the wheel —
  pip-installed users can now load the bundled URDFs, configs, and meshes that
  were missing from the v0.4.0 wheel.
- Resolve bundled assets from a relative path (e.g. `"config/config.yaml"`,
  `"robots/arm_robot.urdf"`) whether you run from the repo or a pip install.

### Internal

- Add a clean-install packaging smoke test (`make test-clean-install`, a
  standalone script, and a Docker image) plus a release-time wheel-verify gate,
  so missing bundled data is caught before publishing.
- Add CI coverage: path-filtered packaging (clean-venv install) and ROS 2
  bridge (Docker integration) workflows.
- Pin `numpy<2` in the bridge image to match the ROS 2 Jazzy runtime.

## v0.4.0 (2026-06-22)

First release with the **ROS 2 bridge and Open-RMF integration**, an event
system, SDF world loading, vectorized batch controllers, and a plugin/device
architecture.

### Highlights

- **ROS 2 bridge** (`ros2_bridge/`, ROS 2 Jazzy) — drive PyBulletFleet from ROS 2:
  per-robot odom / TF / cmd_vel / goal_pose / joint topics; NavigateToPose,
  FollowPath and FollowJointTrajectory actions; and `simulation_interfaces`
  services (spawn / delete / get entities, step simulation, simulator features).
  Dockerized with an integration smoke test.
- **Open-RMF integration** — fleet adapter plus handlers for doors, lifts,
  delivery, cleaning, and battery, with runnable demos: office, hotel, clinic,
  airport_terminal, battle_royale, and campus.
- **EventBus** — global and per-entity lifecycle events (object/agent spawn &
  remove, pre/post step & update, action start/complete, collision enter/exit)
  for plugins and external integrations.
- **SDF loader** — load multi-model SDF worlds, with `<submesh>` extraction from
  multi-part meshes, `model_yaw_offset`, and `force_color` flat-shading for
  meshes PyBullet cannot texture.
- **Batch controllers** — vectorized omnidirectional and differential controllers
  (two-phase step) for efficiently simulating large fleets.
- **Plugin & device architecture** — per-agent plugins (e.g. battery) and
  simulation plugins (e.g. workcell delivery); `Door` and `Elevator` agent
  subclasses with joint control and automatic passenger attachment.

### Features

- Unified `controller=` API: registry name, dict, list, `ControllerParams`, or a
  prebuilt controller; `type:` (registry) / `class:` (dotted import path)
  selection; patrol and random-walk high-level controllers; per-axis kinematic
  limits and `navigation_2d`.
- `resolve_model()` resolves Tier 1/2/3 model names (bundled, ROS description
  packages, `robot_descriptions`) to URDF paths.
- `CameraController` — interactive right-drag pan, zoom, and top-down view.
- `pybullet_fleet_msgs` package (ExecuteAction action).

### Bug Fixes

_(Fixes to behaviour from v0.3.0; issues introduced and resolved within this
release's new features are omitted.)_

- Collision detection was silently disabled when the simulation was loaded from
  YAML — the `collision_detection_method` string is now coerced to its enum.
- Per-axis velocity/acceleration limits with a zero-capped axis (e.g.
  `max_linear_vel: [0.3, 3.0, 0.0]`) no longer produce a NaN limit.

### Documentation

- ROS 2 bridge / Docker guide; corrected headless, env-var, and config-file docs;
  2026-06-20 implementation audit; a single shared agent-instructions source for
  GitHub Copilot and Claude Code.

### Performance

- Kinematics mode (headless, AMD Ryzen AI 7 PRO 350): ~64× real-time at 100
  agents, ~10× at 500, ~4.4× at 1000. See `docs/benchmarking/results.md`.

## v0.3.0 (2026-04-08)

### Breaking Changes

- `DifferentialPhase` removed — motion phase logic is now owned by `DifferentialController` / `OmnidirectionalController` in `controller.py`

### Highlights

- **SimulationRecorder** — Headless and GUI video capture (GIF/MP4) via `start_recording()` API or `RECORD` environment variable
- **Centralized defaults** — All parameter defaults in `_defaults.py` with `PBF_*` env-var overrides and `.env` file support
- **Visual documentation** — Embedded demo videos across all doc pages, YAML-driven batch capture scripts
- **Robot model resolution** — `resolve_model("panda")` finds URDFs/SDFs by name across local, pybullet_data, and robot_descriptions sources (`resolve_urdf` kept as deprecated alias)
- **Controller refactor** — `Controller` extracted from `Agent` into `controller.py`, reducing `agent.py` by ~500 lines
- **Entity registry** — `register_entity_class()` for custom spawn types via YAML config

### Features

- `SimulationRecorder` — camera modes (auto, gui, orbit, manual), time bases (sim, real), GIF/MP4 output
- `start_recording()` / `stop_recording()` on `MultiRobotSimulationCore`; `RECORD` env-var enables auto-recording in `run_simulation()`
- `_defaults.py` — single source of truth for all defaults; `PBF_{SECTION}_{KEY}` env-var overrides; `.env` auto-loading via python-dotenv
- `robot_models.py` — `resolve_model()`, `auto_detect_profile()`, `register_model()`, `discover_models()`, `add_search_path()` (`resolve_urdf()` kept as deprecated alias)
- `controller.py` — `OmnidirectionalController`, `DifferentialController` extracted from Agent
- `entity_registry.py` — `register_entity_class()` for extensible YAML-driven spawning
- `compute_scene_bounds()` — scene AABB for auto-framing cameras
- `unregister_callback()` — remove callbacks by function identity
- `SimulationParams.enable_floor` — option to skip ground plane loading
- `[models]` extra in `pyproject.toml` — `pip install pybullet-fleet[models]` pulls `robot_descriptions`
- New demo scripts: `capture_demo.py` (headless), `capture_screen_demo.py` (GUI), `capture_model_catalog.py`
- Examples reorganized into `arm/`, `basics/`, `mobile/`, `scale/`, `models/` subdirectories

### Documentation

- Embedded demo videos on all example pages; Quickstart page with single demo video
- How-To: Capturing Demo Videos (Python API, `RECORD` env var, batch scripts)
- Configuration Reference — centralized defaults, override priority, `.env` usage
- Tutorial 6: Robot Models — resolution, auto-detect, search paths, auto-discovery
- Roadmap page with performance profiling plan

### Testing

- 960 tests (up from 740), coverage 82%
- New modules: `test_recorder.py`, `test_defaults.py`, `test_robot_models.py`, `test_controller.py`, `test_entity_registry.py`

### Performance

- 1000-agent RTF improved **+38%** (2.4× → 3.3×, 40.9 ms → 30.0 ms/step)
- 500-agent RTF improved **+12%** (6.8× → 7.6×, 14.7 ms → 13.2 ms/step)

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
