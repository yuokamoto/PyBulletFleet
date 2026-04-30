# Roadmap

Planned additions and improvements for PyBulletFleet.
Items are grouped by category; ordering within a group does not imply priority.


## Assets

New robot and infrastructure models:

- **Physics Mobile Robot** — Wheeled robot driven by PyBullet physics (motor torques, friction, contact forces)
- **Physics Mobile Manipulator** — Physics-mode mobile manipulator with motor-driven base and arm
- **Conveyor / Elevator / Mobile Rack** — Warehouse infrastructure entities for material handling scenarios

## Features

- **Snapshot & Replay** — Full and delta snapshot serialization for logging, replay, and external synchronization ([USO](https://github.com/yuokamoto/Unified-Simulation-Orchestrator) integration)
- **Behavior tree integration** — Create agent behavior from behavior trees
- **`SimObject.from_sdf()` → `List[SimObject]`** — Factory method that loads an SDF file via `p.loadSDF()` and wraps each returned body_id in a `SimObject`. Collision detection and lifecycle management via `add_object()` are applied automatically. Required for Open-RMF SDF environment loading and official support for pybullet_data SDF models (kiva_shelf, wsg50_gripper, etc.). Currently the catalog demo calls raw `p.loadSDF()` directly.

### Devices

See [ros2_bridge/README.md](../ros2_bridge/README.md) for device enhancements (elevator doors, double-hinge doors, xacro-parameterised URDFs).

## Interfaces

External communication layers:

- **ROS 2** — Topic / service / action bridge for ROS 2 ecosystem integration (see [ros2_bridge/README.md](../ros2_bridge/README.md))
- **gRPC** — Language-agnostic RPC interface for orchestrators, WMS, and fleet managers

## Refactoring

- **`MultiRobotSimulationCore` responsibility decomposition** — The core class has grown to ~3200 lines / 66 methods. Extract self-contained subsystems into dedicated classes composed into the core:

  | Subsystem | Target class | Methods to extract | Est. lines |
  |---|---|---|---|
  | Collision detection (spatial hash, AABB, broad/narrow phase) | `CollisionSystem` | `check_collisions`, `filter_aabb_pairs`, `_update_object_spatial_grid`, … (~20 methods) | ~1000 |
  | Visualizer (camera, transparency, keyboard) | `VisualizerController` | `configure_visualizer`, `setup_camera`, `_handle_keyboard_events`, … (~6 methods) | ~250 |
  | Profiling & memory tracking | `SimProfiler` | `record_profiling`, `_print_profiling_summary`, `_print_memory_profiling_summary`, … (~5 methods) | ~200 |

  Each subsystem is held by composition (`self._collision = CollisionSystem(self)`).
  The core class delegates; public API does not change.
  Natural stepping stone toward the SimBackend ABC (Long-Term Phase 1).

- **Move `controller.py` into `controllers/` package** — Base controller classes (`Controller`, `KinematicController`, `OmniController`, `DifferentialController`, `create_controller`, `register_controller`) live in `pybullet_fleet/controller.py` while higher-level controllers (`PatrolController`, `RandomWalkController`) are already in `pybullet_fleet/controllers/`. Consolidate by moving the base module into the package as `controllers/base.py` (or splitting omni/differential into separate files) and re-exporting from `controllers/__init__.py`. Preserve the `from pybullet_fleet.controller import ...` path via a compatibility shim or `__init__.py` re-export.

- **Remove scipy dependency** — Currently only `scipy.spatial.transform.Rotation` is used (9 call sites for quat↔euler, quat↔matrix, relative rotation). Replace with PyBullet utilities + lightweight helpers in `geometry.py` to eliminate the ~150 MB transitive dependency. Low priority: no runtime performance impact, only install size.

- **Manual quaternion helpers in `geometry.py`** — Extend the pattern established by `SlerpPrecomp` / `quat_slerp` (avoiding scipy, hand-written scalar math) by adding the following helpers to `geometry.py`. The goal is to eliminate scipy `Rotation` object creation overhead on hot paths.

  Required helper functions:
  - `quat_rotate_vector(q, v)` — Rotate vector `v` by quaternion `q` (body→world transform). Replaces `Rotation.from_quat(q).apply(v)`
  - `quat_multiply(q1, q2)` — Quaternion product. Replaces `(Rotation.from_quat(q1) * Rotation.from_quat(q2)).as_quat()`
  - `quat_from_rotvec(rotvec)` — Quaternion from rotation vector. Replaces `Rotation.from_rotvec(rotvec).as_quat()` (used in angular velocity → orientation update)

  Primary application sites:
  - `OmniVelocityController._apply_velocity()` — body→world velocity transform + quaternion update from angular velocity
  - `tools.body_to_world_velocity_3d()` — same as above
  - `Path._calculate_orientation_for_plane()` — rotation matrix → quaternion conversion
  - `Path.visualize_waypoints()` — quaternion → rotation matrix conversion

  Design policy:
  - Pure Python scalar math (`math.sin`/`math.cos`) or small numpy array ops
  - Same file and style as `SlerpPrecomp`
  - Apply incrementally after profiling confirms bottleneck (YAGNI)
  - Full scipy removal in a separate PR after all call sites are replaced

## Performance

Near-term optimizations within the current PyBullet-backed architecture.
Goal: reduce per-step cost at 100–1000 agents without a full backend swap (see Long-Term section for that).

### Profiling Baseline (measured 2026-04-03)

Benchmark: 500 agents, omnidirectional MoveAction, `collision_check_frequency=0` (disabled), `physics=False`, `simple_cube.urdf`.

**Per-step breakdown (median):**

| Component | Time | % of total | What |
|---|---|---|---|
| Python overhead (TPI + slerp + action queue + set_pose logic) | 11.5 ms | 88% | CPython for-loop + object dispatch |
| `p.resetBasePositionAndOrientation` | 0.7 ms | 5% | PyBullet C API |
| `p.getAABB` | 0.5 ms | 4% | PyBullet C API |
| Movement detection | 0.3 ms | 3% | Pure Python arithmetic |
| **Total** | **13.0 ms** | 100% | **FPS 77** |

Key insight: **88% of step time is Pure Python overhead**, not C API calls.

Collision at 10 Hz adds only ~0.2 ms at 500 agents — negligible compared to agent_update.

**Vectorization micro-benchmark (500 agents):**

| Operation | Python for-loop | NumPy vectorized | Speedup |
|---|---|---|---|
| Position compute (`start + dir × ratio`) | 1,150 μs | 5.6 μs | **205×** |
| TPI-like trapezoidal profile | (per-agent) | 33 μs | — |
| Per-agent cost | 23 μs/agent | ~0.01 μs/agent | — |

### Two-Phase Step: Decouple Computation from PyBullet C API

Current `step_once()` iterates agents one-by-one, each calling `controller.compute()` (Python/NumPy) → `set_pose_raw()` (PyBullet C API + AABB update + spatial grid) interleaved. This prevents vectorization and adds per-agent Python↔C crossing overhead.

**Proposed split:**

| Phase | What | Hot path |
|-------|------|----------|
| **Phase 1 — Compute** | All controllers compute new poses; no side effects | Pure Python / NumPy |
| **Phase 2 — Apply** | Tight loop of `p.resetBasePositionAndOrientation()` only | PyBullet C calls |
| **Phase 3 — Bookkeep** | Batch AABB refresh (`p.performCollisionDetection()` + `p.getAABB()`) and spatial grid update | C calls + Python dict |

This requires **removing direct `pybullet` API calls from `sim_object.py`, `agent.py`, and `controller.py`**. Instead, these modules produce *pose intents* (position + orientation tuples), and `core_simulation.py` flushes them to PyBullet in bulk.

Key changes:
- `SimObject.set_pose()` / `set_pose_raw()` writes to an internal buffer (cached pose + dirty flag) without calling `p.resetBasePositionAndOrientation()`
- `Controller.compute()` returns `(new_pos, new_orn)` or writes to agent's pending pose buffer
- `core_simulation.step_once()` collects dirty poses → batch `resetBasePositionAndOrientation` → batch AABB update
- Movement detection stays pure Python (already cached-pose-based), unaffected
- Attached-object propagation runs after Phase 2 using the buffered parent poses

### Vectorized Agent Update (NumPy Batch)

For the common "N agents on straight-line TPI paths" case, Phase 1 can be further vectorized:

- Store all active agents' `forward_start_pos`, `forward_direction`, and TPI parameters in contiguous `(N, 3)` NumPy arrays
- Compute `new_positions = start_positions + directions * ratios[:, np.newaxis]` in one vectorized call
- Slerp batch: pre-compute all `(start_quat, target_quat, t_fraction)` and batch `quat_slerp`
- Fallback: agents with non-standard controllers (velocity mode, custom callbacks) use the existing per-agent path

This is a "BatchController" or "VectorizedOmniController" that sits alongside the existing `Controller` ABC.

### C++ Extensions for Hot-Path Functions

Profile-guided candidates for C++ (via pybind11) or Cython acceleration:

| Function | Current | Why C++ helps |
|----------|---------|---------------|
| **TwoPointInterpolation** | Pure Python `math` | Called N× per step; tight numerical loop ideal for native code |
| **`quat_slerp` / `quat_slerp_precompute`** | Python scalar math in `geometry.py` | N× per step; SIMD-friendly |
| **Spatial hash broad-phase** | Python dict + set ops in `check_collisions()` | Dict overhead at 1000+ objects; Rust/C++ hash map faster |
| **AABB overlap test** | Python comparisons in `_aabb_overlap_2d` | Tight inner loop; autovectorizable in C++ |
| **`getClosestPoints` narrow-phase** | PyBullet C API (already native) | Already fast; not a candidate |

Priority: TPI and slerp first (highest call frequency), then spatial hash (scales with agent count²).

### Deferred AABB Update

Currently `set_pose()` calls `p.getAABB()` and updates the spatial grid **per object, immediately**. For kinematic-only mode:

- Defer all AABB updates to a single `p.performCollisionDetection()` call after Phase 2
- Batch `p.getAABB()` for all moved objects at once
- Rebuild spatial grid once per step instead of incrementally per-object

This removes N `p.getAABB()` C-API round-trips from the set_pose hot path.

### Summary: Expected Impact (500 agents, measured baseline)

| Optimization | Estimated step time | Estimated FPS | Speedup vs current | Effort |
|---|---|---|---|---|
| **Current** | 13.0 ms | 77 | 1.0× | — |
| **NumPy vectorized controller** | ~1.2 ms | ~850 | **~11×** | Medium |
| **+ C++ TPI/slerp extensions** | ~1.0 ms | ~1,000 | **~13×** | Medium |
| **Theoretical floor** (C API only) | 0.7 ms | ~1,400 | ~18× | — |

Scaling by agent count (current → vectorized estimate):

| Agents | Current FPS | Est. vectorized FPS | Speedup |
|---|---|---|---|
| 100 | 225 | ~2,000+ | ~9× |
| 500 | 77 | ~850 | ~11× |
| 1,000 | 27 | ~450 | ~17× |

> **Note:** Collision detection (10 Hz spatial hash) adds <1% overhead at 500 agents. C++ spatial hash becomes relevant at 1,000+ agents with higher collision frequencies.

> **Relation to Long-Term Backend Abstraction:** The two-phase split and the "remove direct pybullet calls" refactoring are natural stepping stones toward the SimBackend ABC (Phase 1 of Long-Term). The buffered-pose pattern becomes the write side of `SimBackend.set_positions_batch()`.

## SDF & DAE Support Improvements

PyBulletFleet currently has two self-implemented workarounds for Gazebo ecosystem interop:

1. **`resolve_sdf_to_urdf`** — Hand-rolled SDF→URDF XML converter (PyBullet cannot load SDF directly)
2. **DAE defensive fallbacks** — Colour extraction, texture symlinks, collision try/except (PyBullet has poor DAE support)

Both are functional but hacky. This section tracks improvements to make them more robust or replace them entirely.

### SDF → URDF Conversion: `gz sdf -p` Replacement

`resolve_sdf_to_urdf()` in `sdf_loader.py` is a minimal self-implemented SDF→URDF converter using `xml.etree`. It covers only `<link>`, `<joint>`, `<mesh>`, and primitive geometries — enough for simple robots (DeliveryRobot) but fragile for complex SDF models.

**Why self-implemented?** No usable Python SDF→URDF tool exists:
- `sdformat_urdf` — C++ only, no Python bindings
- `pysdf` (PyPI) — fails to build on Python 3.12+
- `libsdformat14` — C++ template API, not callable via ctypes
- `gz sdf -p model.sdf` — converts SDF→URDF via CLI subprocess (see evaluation below)

**`gz sdf -p` evaluation (Gazebo Harmonic / sdformat 14, as of 2026-04):**

| | Self-implemented parser | `gz sdf -p` |
|---|---|---|
| Dependencies | None (`xml.etree` stdlib) | `sdformat14` + `gz-tools` (~200 MB) |
| Coverage | Minimal (link/joint/mesh only) | Broader (frames, nested models partial) |
| Edge cases | Fix ourselves immediately | Wait for upstream fix |
| DAE colour injection | `_extract_dae_diffuse_color` embeds into URDF | Not supported (no colour in output URDF) |
| Nested `<model>` | Not supported | Partial support |
| Known issues | Pose hierarchy may drift for deep nesting | Nested model bugs, some pose frame resolution errors |
| Stability | Tested for our models | "Best effort" — known regressions between versions |

**Verdict:** `gz sdf -p` is **usable but not fully stable**. For simple robots it works; for complex SDF it has regressions. The dependency cost (~200 MB) is also significant. **Current self-implemented parser is the pragmatic choice.**

**Future action:**
- Monitor `gz sdf -p` stability across Gazebo releases
- If it stabilises + we already have `gz-tools` in Docker for other reasons → switch to subprocess call and delete `resolve_sdf_to_urdf()`
- If we support complex SDF robots (nested models, sensors) → worth the dependency

### DAE → OBJ Automatic Conversion

PyBullet has poor DAE (COLLADA) support: textures are not loaded, diffuse colours are ignored, and some meshes fail `createCollisionShape`. Gazebo Fuel models ship exclusively as DAE.

**Proposal:** Use `assimp export` (from `assimp-utils`, ~2 MB) to convert DAE meshes to OBJ + MTL at download time. OBJ is PyBullet's best-supported mesh format — colours, textures via MTL, and collision all work reliably.

Changes needed:
1. **`docker/Dockerfile.rmf_demos`** — `apt install assimp-utils`
2. **`docker/download_fuel_models.py`** — Post-download `assimp export *.dae *.obj` conversion step
3. **`pybullet_fleet/sdf_loader.py`** — Prefer `.obj` over `.dae` when both exist (`mesh_path.replace(".dae", ".obj")`)

**Simplification benefit:** Once all DAE meshes have OBJ counterparts, three defensive workarounds in `sdf_loader.py` become unnecessary and can be removed:

| Current workaround | Why it exists | Removed after conversion |
|---|---|---|
| `is_fuel` flag on `_IncludeInfo` | Fuel DAE has embedded colours; local OBJ does not → different `rgba_color` handling | OBJ+MTL always carries material info → uniform `rgba_color=None` for all models |
| `_ensure_texture_symlinks()` | DAE references textures by bare filename; PyBullet resolves relative to `meshes/` not `materials/textures/` | OBJ+MTL uses relative paths that resolve correctly |
| `try/except` around `createCollisionShape` in `sim_object.py` | Some DAE meshes crash PyBullet's collision shape loader | OBJ collision shapes load reliably |

The `is_fuel` flag would be replaced by a mesh-format check (e.g. "does an `.obj` file exist alongside the `.dae`?"), making the loader source-agnostic — any DAE model from any provider would be handled the same way.

### Priority

Low — the current SDF parser + DAE defensive fallbacks are functional for demo purposes. These improvements primarily affect:
- **Visual fidelity** (DAE→OBJ: textures and materials render correctly)
- **Maintainability** (gz sdf -p: delete ~150 lines of hand-rolled XML conversion)
- **Robustness** (both: fewer edge-case failures with complex models)

## Environments

Simulation environment assets (warehouse floors, factory layouts, etc.):

- **`pybullet-fleet-environments` package** — Manage environment assets in a separate repository, installable via `pip install pybullet-fleet-environments` for on-demand retrieval. Keep PyBulletFleet core lightweight by not bundling meshes.
  - AWS RoboMaker Small Warehouse (MIT-0): DAE→OBJ converted meshes + URDF wrappers
  - Open-RMF rmf_demos maps (office, hotel, clinic, airport, campus): OBJ mesh export
  - pybullet_data bundled environments (kiva_shelf, samurai, stadium) wrappers
  - Original license clearly noted per environment
- **`resolve_environment()` API** — Name resolution similar to `resolve_model()` for loading environments. Shows install hints when not installed.

## CI / DevOps

- **GitHub Actions refactoring** — Streamlined CI pipeline
- **Automated performance tracking** — Run time / memory benchmarks in CI, auto-update results in documentation, and alert on significant performance regressions

## Long-Term: Backend Abstraction & Beyond PyBullet

Current architecture is tightly coupled to PyBullet (`body_id`, per-entity FFI calls). At 1000 agents PyBullet's per-call overhead dominates step time (88% of 40.9 ms). The following items explore decoupling from PyBullet to unlock 10–100× performance gains while keeping the Python user API unchanged.

### Phase 1: SimBackend ABC + Numpy Pure Kinematic Backend

- **SimBackend ABC** — Abstract interface (`set_positions_batch`, `detect_collisions`, `load_model`, `step_physics`) that `Agent` and `SimObject` program against instead of raw `pybullet` calls
- **NumpyBackend (default)** — Contiguous numpy arrays for positions/orientations, `scipy.spatial.cKDTree` for collision. No physics engine dependency. Expected: 1000 agents in ~2–5 ms (RTF 20–50×), 5000+ agents at RTF > 1.0
- **PyBulletBackend (compat)** — Wraps existing PyBullet calls behind SimBackend ABC for backward compatibility and physics-mode users
- **URDF parsing without PyBullet** — Use `yourdfpy` or similar to parse URDF into internal model data, removing the last hard dependency on PyBullet for kinematic-only mode
- **Visualization decoupling** — Replace `p.GUI` with Rerun, Open3D, or RViz for rendering. Backend-agnostic scene display

### Phase 2: Native Backend (Rust/C++ via PyO3/pybind11)

Only justified if Phase 1 numpy performance is insufficient (e.g., 5000+ agents at 240 Hz).

- **Rust kinematic core** — Position update, yaw integration, AABB collision in Rust. Exposed to Python via PyO3. Expected: further 3–10× over numpy (RTF 100–300× for 1000 agents)
- **Batch collision in Rust** — Sweep-and-prune or spatial hash for O(n log n) broad-phase, replacing Python KDTree
- **Zero-copy interop** — Share numpy arrays directly with Rust via buffer protocol (no serialization)

### Phase 3: GPU Backend (optional)

For 10,000+ agent scenarios or RL training workloads.

- **MuJoCo MJX backend** — JAX-accelerated batch physics on GPU. URDF via MJCF conversion
- **JAX pure kinematic** — `jax.numpy` drop-in for NumpyBackend, JIT-compiled, GPU-parallel

### Phase 4: ECS Architecture (v2.0 candidate)

Considered when entity diversity explodes beyond what Agent/SimObject OOP can express cleanly (e.g., drones + ground robots + conveyors + elevators + dynamic obstacles all in one scene with distinct component sets).

- **Component-based entity model** — Replace Agent/SimObject inheritance with composable components (Transform, Collision, Kinematics, JointState, ActionQueue, etc.)
- **System pipeline** — Each system (MovementSystem, CollisionSystem, ActionSystem) operates on component arrays. Maps naturally from current Controller ABC → System
- **Rust ECS runtime** — Leverage `bevy_ecs` or `hecs` for cache-friendly SoA memory layout. Python API via PyO3 for user-facing logic
- **Migration path** — Controller ABC → System, EventBus → ECS Events, Registry → ECS resource/archetype queries. Plugin Architecture phases are designed as stepping stones toward this

> **Note:** At this point the project may evolve beyond "PyBulletFleet" in name, as PyBullet would be just one optional backend among several, and the core would no longer depend on it. A rename (e.g., *FleetSim*, *KinematicFleet*) may be appropriate.
