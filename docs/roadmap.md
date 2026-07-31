# Roadmap

Planned additions and improvements for PyBulletFleet.
Items are grouped by category.  A roadmap item states the intended outcome and
its boundary; implementation detail belongs in the linked architecture or
design documentation.


## Assets

New robot and infrastructure models:

- **Physics Mobile Robot** — Wheeled robot driven by PyBullet physics (motor torques, friction, contact forces)
- **Physics Mobile Manipulator** — Physics-mode mobile manipulator with motor-driven base and arm
- **Conveyor / Mobile Rack** — Warehouse infrastructure entities for material
  handling scenarios.  Door and Elevator devices, and the WorkcellPlugin used
  for RMF dispenser/ingestor flows, are available today.

## Simulation Capabilities

- **Snapshot, Event log & Replay** *(event emission complete; recording and
  replay pending)* — Three-layer architecture sharing a single EventBus as a
  source. Enables replay, external synchronization ([USO](https://github.com/yuokamoto/Unified-Simulation-Orchestrator)
  integration), and downstream observability (see [Observability](#observability) below).

  **Layer overview:**

  | Layer | Purpose | Granularity | Lossless? | Replayable? |
  |---|---|---|---|---|
  | **Snapshot** | State checkpoint for seek / restore | Low frequency (e.g. 1 Hz) | Yes | Yes (state restore) |
  | **Event log** | Causal record of state transitions (input replay, audit) | Per state-change | Yes (must) | Yes (input replay) |
  | **Trace** | Operational observability (Grafana/Tempo) | Action-level spans | Sampling allowed | No (view only) |

  **Design principle: same source, separate sinks.** All three are derived from
  the EventBus ([eventbus/spec.md](https://github.com/yuokamoto/PyBulletFleet/blob/main/docs/design/eventbus/spec.md))
  — emit once, fan out to multiple subscribers. Snapshot ≠ Event (different
  schemas, separate files), but share common header keys (`sim_time`, `step`,
  `wall_time`, `run_id`) so they can be merged on the time axis.

  **Why trace alone cannot replay:** trace is sampled, coarse-grained (Action-level spans), drops non-deterministic inputs (RNG seed, callback returns), and has attribute-size limits unsuitable for `Path` waypoints / IK results. Replay needs Event log (input) + Snapshot (checkpoint).

  **Output layout:**

  ```
  run_<timestamp>/
    snapshots.jsonl   # low-frequency full+delta state
    events.jsonl      # lossless state-change events
    # OTel spans → Tempo (separate, optional, sampled)
  ```

  **Implementation order:**

  **Current state:** the EventBus and lifecycle, action, collision, pause, and
  per-agent update events are available.  Handlers receive the current
  event-name/keyword-argument API; there is no persisted, structured
  `SimEvent` record yet.

  **Remaining order:**

  1. Define a versioned event-record schema and JSONL writer subscriber
     (lossless input log).
  2. Add `SimulationSnapshot` plus
     `MultiRobotSimulationCore.snapshot()` / `restore()` (see
     [snapshot-replay/spec.md](https://github.com/yuokamoto/PyBulletFleet/blob/main/docs/design/snapshot-replay/spec.md)).
  3. Add `replay.py` for snapshot + event-log re-execution.
  4. Add an OTel exporter subscriber (independent of replay; see
     [Observability](#observability)).

- **Behavior tree integration** — Create agent behavior from behavior trees
### Crowd Simulation (lightweight)

Lightweight crowd simulation for NPC pedestrians in RMF demo environments. Provides more realistic pedestrian behavior than the current `RandomWalkController` (random point within radius) while staying far simpler than Gazebo's full Menge/ORCA `crowd_simulator` plugin.

**Motivation:** RMF demos (airport_terminal) use Gazebo's `crowd_simulator` with Menge BFSM + ORCA + nav mesh for 10 pedestrians. PyBulletFleet currently substitutes `RandomWalkController` which has no obstacle avoidance and no agent-agent interaction. A middle-ground is useful for visual fidelity in demo environments without the complexity of a full crowd simulation framework.

**Scope:**

| Feature | RandomWalkController (current) | Proposed | Gazebo crowd_simulator |
|---------|-------------------------------|----------|----------------------|
| Obstacle avoidance | None | Simple raycast or AABB check | ORCA + nav mesh |
| Agent-agent avoidance | None | Separation force (boids-style) | ORCA mutual |
| Goal selection | Uniform random in radius | Weighted goal sets (configurable) | BFSM state machine |
| Nav mesh | Not required | Not required | Required |
| Config complexity | 2 params | ~5 params (YAML) | 3 XML files |
| Computation | ~0 | O(n·k) neighbor lookup | O(n²) ORCA |

**Approach:** A new `CrowdController` (or enhancement of `RandomWalkController`) that adds:
1. **Goal sets** — Named groups of positions with transition weights (like Menge BFSM but in YAML)
2. **Simple separation** — Repulsive force from nearby agents (boids separation rule, not full ORCA)
3. **Static obstacle avoidance** — Optional raycast or AABB query to avoid walls (PyBullet `rayTest`)

**Priority:** Low — current `RandomWalkController` is functional for demo purposes. This is primarily for RMF demo visual fidelity (再現度).

#### Crowd-as-dynamic-obstacle throughput benchmark

The higher-value use of a crowd is not visual fidelity but **turning crowd density into a benchmark variable**: measure how much a fleet's *task* throughput degrades when robots must contend with pedestrians. Pure visual pedestrians (a human mesh wandering) have little functional value here, because the fleet is RMF-path-driven and does not avoid arbitrary dynamic obstacles — the NPCs are currently just eye-candy. Making them *dynamic obstacles the fleet reacts to* is what aligns with PyBulletFleet's strength (throughput/scale benchmarking at hundreds of robots, a regime too heavy for Gazebo).

**Three pieces:**
1. **Crowd as dynamic obstacles** — the lightweight `CrowdController` above, with pedestrian positions known to the sim.
2. **Robot reaction** —
   - *Minimal:* "pedestrian within X m ahead → slow/stop" via PyBullet `rayTest` / collision checks. No RMF changes; measures stop-and-wait delay.
   - *Full:* publish detected pedestrians as `rmf_obstacle_msgs/Obstacles` so RMF reroutes / closes lanes (heavier; integrates with `rmf_visualization_obstacles` and the obstacle pipeline).
3. **Task-throughput harness** — dispatch N patrol/delivery tasks and measure completion time / completions-per-hour. This is a **new metric** distinct from the existing RTF/step-time benchmarks (which measure sim performance, not fleet task throughput).

**Experiment:** sweep crowd density × avoidance{off, stop, reroute} → task throughput, at 100–1000 robots. Reuses existing collision detection, `rayTest`, EventBus, and the benchmark harness scaffolding.

**Priority:** Low / research — schedule after the ROSCon minimum (working demos + packaging). Start with the minimal reaction (stop-on-detect, no RMF changes).

### Devices

Door, Elevator, and workcell simulation are implemented in the core/bridge
integration.  ROS/RMF-specific gaps and device-model enhancements are tracked
in `ros2_bridge/README.md`; this roadmap retains only cross-project asset work.
See [device-external-agent/spec.md](https://github.com/yuokamoto/PyBulletFleet/blob/main/docs/design/2026-04-21-device-external-agent/spec.md)
for the implemented device/external-actor model.

## RMF Demo Feature Coverage

Gaps between the standard Gazebo `rmf_demos` and the PyBulletFleet bridge. Core
flows (navigation, doors, **lifts**, delivery pick/drop, battery/charging,
**cleaning coverage**, **georeferenced maps**) are implemented. Ported demos:
office, clinic, hotel, airport_terminal, battle_royale, **campus**. Outstanding:
See [open-rmf/spec.md](https://github.com/yuokamoto/PyBulletFleet/blob/main/docs/design/open-rmf/spec.md)
for the bridge design and the original integration boundary.

**Demos not ported — require an RMF mode the bridge doesn't implement:**
- `office_mock_traffic_light` and `triple_H` — both use the **`EasyTrafficLight`** adapter (the robot owns navigation; RMF only gates it with go/stop). This inverts the bridge's full-control model and is a poor fit for the kinematic, RMF-planned sim — see the [TrafficLight discussion]. Relevant only once a co-simulation/robot-proxy layer exists. Run them in Gazebo for now.

**RMF tasks/actions — partial** (`fleet_adapter.execute_action`):
- **dock** — treated as a plain navigate (no dedicated dock-path approach; endpoint is reached, only the precise approach trajectory is skipped — negligible in a kinematic sim).
- Other custom performable actions — logged and finished, not executed.

**Substituted** (works, but differs from Gazebo):
- Crowd — Menge/ORCA `crowd_simulator` → lightweight `RandomWalkController` (no avoidance). See [Crowd Simulation](#crowd-simulation-lightweight).
- Caddy — Gazebo keyboard teleop (`diff_drive` plugin) → autonomous `PatrolController`.

**Not modeled by design** (kinematic focus):
- Robot sensors (lidar/camera), physics dynamics (wheel torque/contact).
- `rmf_obstacle` detection pipeline (also unused in stock Gazebo demos; see the "Crowd-as-dynamic-obstacle throughput benchmark" section above).

Detailed ROS/RMF release and validation plans live in `ros2_bridge/README.md`;
keep this section focused on user-visible feature coverage.

## Interfaces

External communication layers:

- **Fleet API actor scopes** — Separate simulation actor ownership from public
  fleet visibility. Today `FleetStateProvider` publishes all `sim.agents` and
  `FleetCommandDispatcher` resolves every named Agent, while `AgentManager`
  groups execution only. Add a short-term filter for fleet-state publication
  so mock people, external robots, and devices do not inflate `/fleet/states`.
  Then add named API scopes that can be attached to one or more managers and
  independently select state publication and command ingress (with distinct
  endpoint namespaces when multiple fleets are exported). Keep the state and
  command scopes explicit rather than inferring ownership from an Agent type.
  Acceptance criteria: a state-only filter can be deployed first; a named
  scope can select one or more managers; and changing command ingress never
  implicitly changes state publication (or vice versa).
  See [fleet-control-api/spec.md](https://github.com/yuokamoto/PyBulletFleet/blob/main/docs/design/fleet-control-api/spec.md).

- **ROS 2** — Topic / service / action bridge for ROS 2 ecosystem integration
  (see `ros2_bridge/README.md` and
  [ros2-bridge/spec.md](https://github.com/yuokamoto/PyBulletFleet/blob/main/docs/design/ros2-bridge/spec.md))
- **gRPC** — Language-agnostic RPC interface for orchestrators, WMS, and fleet managers
- **Distributed co-simulation (Robot Proxy layer)** — Per-robot proxy processes that translate between simulated robots and real Robot Apps (Nav2, task assigners, BTs). Enables running 100+ unmodified single-robot software stacks against a centralized batched simulator. Sim Central stays single-process and batched; only a thin fixed-schema boundary (`StateSnapshot` + `CommandBuffer` + Events) crosses the IPC. **Shares schema with Snapshot/Replay** — same `StateSnapshot` dataclass feeds live IPC, replay log, and observability sinks (define schema once, fan out to multiple consumers). See [co-simulation/spec.md](https://github.com/yuokamoto/PyBulletFleet/blob/main/docs/design/co-simulation/spec.md) for layer separation, transport options (shared memory / gRPC / DDS), lockstep vs async sync modes, and the implementation phases.

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
  See [controller-refactor/spec.md](https://github.com/yuokamoto/PyBulletFleet/blob/main/docs/design/controller-refactor/spec.md).

- **Remove scipy runtime dependency** — Quaternion helpers now cover vector
  rotation, multiplication, rotation-vector conversion, and angle comparison;
  the batch omni hot path already uses them.  Audit and migrate the remaining
  `Rotation` callers in `geometry.py`, `_motion_planning.py`, and the batch
  differential controller, then remove the dependency only after equivalent
  numerical tests cover the replacements.  This is an install-size and
  dependency-simplification task, not a promised performance improvement.

## GUI / Interactive Tools

Improve `p.GUI` interactivity for development and debugging.
Long-term plan is to replace `p.GUI` with Rerun (see [Long-Term Phase 1](#long-term-backend-abstraction--beyond-pybullet)) where many of these capabilities come built-in. These items are the **interim layer** until that migration lands, and naturally collect under the planned `VisualizerController` (see Refactoring).
The existing camera interaction boundary is described in
[camera-control/spec.md](https://github.com/yuokamoto/PyBulletFleet/blob/main/docs/design/2026-04-18-camera-control/spec.md).

Current state:
- `sim.pause()` / `resume()` Python API ✅
- `SPACE` keybinding for pause toggle ✅
- `pause` / `resume` events on EventBus ✅
- `CameraController` with right-drag pan, zoom, top-down view ✅

Planned:

- **Pause / Step / Speed control panel** — In-window UI for pause/resume, single-step, and real-time multiplier. Short-term: `p.addUserDebugParameter` sliders/buttons. Mid-term: tkinter side panel composed with `DataMonitor`.
- **SimObject inspector panel** — Live list of all `_agents` / `_sim_objects` with `object_id`, `name`, current `Pose`, action queue length, and status. Click-to-select with viewport highlight.
- **Camera focus on selection** — Selected entity becomes camera target; auto-follow toggle; distance / yaw / pitch sliders. Replaces the current static `setup_camera`.
- **Object manipulation** — Drag selected `SimObject` in XY plane, edit pose via numeric input, delete via UI (calls `remove_object()` properly so collision/EventBus stay consistent). Respects `pickable` flag.
- **Action queue inspector** — Per-Agent visualization of pending and active actions (type, status, target). Right-click to cancel an action.
- **Keyboard shortcut overlay** — Help panel listing all bound keys (`SPACE`, future shortcuts), surfaced via a `?` key.

Implementation notes:
- All new GUI affordances must be **opt-out via `SimulationParams(gui=False)`** so headless / CI paths stay zero-cost.
- UI state changes must publish through the EventBus (`gui.select`, `gui.manipulate`, `gui.action_cancel`) so plugins / recordings see them.
- Avoid per-step UI redraws — throttle to ≤30 Hz independent of sim FPS.

## Performance

Near-term optimizations within the current PyBullet-backed architecture.
Goal: reduce per-step cost at 100–1000 agents without a full backend swap (see Long-Term section for that).

Measured throughput and profiling methodology are maintained in
[Benchmark Results](benchmarking/results) and the profiling guide.  They are
configuration- and hardware-dependent, so this roadmap intentionally does not
duplicate point-in-time timings or projected speedups.

### Two-Phase Pose Commit: Decouple Computation from PyBullet C API ✅ (batch controller path)

Base-pose updates are buffered during simulation work and committed together,
which lets manager-level batch controllers vectorize compatible agents.  The
public API remains pose/controller based; joint control and plugin code may
still make necessary PyBullet calls.  See [Two-Phase Simulation Step](architecture/two-phase-step)
for the operational model and extension boundaries, and
[two-phase-step/spec.md](https://github.com/yuokamoto/PyBulletFleet/blob/main/docs/design/two-phase-step/spec.md)
for the design record.

### Vectorized Agent Update (NumPy Batch) ✅

`fleet_controller.type` selects the batch omni or differential implementation
for a named manager.  Agents that are not compatible with that batch path keep
their normal per-agent controller behavior.  See the fleet tutorial and
controller configuration reference for user-facing configuration.

### C++ Extensions for Hot-Path Functions

Consider native extensions only after a reproducible profile identifies a
dominant Python hot path.  Likely candidates are interpolation/quaternion work
and broad-phase spatial hashing; PyBullet narrow-phase calls are already
native and are not an initial target.

### Deferred AABB Update

Audit whether collision-disabled or low-frequency configurations perform more
AABB/grid work than required.  Any change must preserve collision-event timing
and the current two-phase synchronization guarantees.

## SDF & DAE Support Improvements

PyBulletFleet currently has two self-implemented workarounds for Gazebo ecosystem interop:

See [sdf-loader/spec.md](https://github.com/yuokamoto/PyBulletFleet/blob/main/docs/design/sdf-loader/spec.md)
for the original SDF-loading scope and constraints.

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

### Multi-material mesh textures (per-material visuals)

`createVisualShape(GEOM_MESH)` applies a **single texture per mesh**, so a
multi-material mesh (one OBJ/DAE packing many materials) renders with just one
texture tiled over everything. The campus env is a single 50-material OBJ and
showed one (a person) texture across the whole scene. Current workaround: the
`world.force_color` config flat-shades such a mesh grey (strips its `mtllib`
so no texture loads — see `_obj_without_materials`). Proper fix: split the mesh
by material into per-material sub-visuals (reusing the trimesh `<submesh>`
extraction path), one texture each. Moderate effort; helps any multi-material
environment.

### Priority

Low — SDF parsing, DAE handling, and multi-material textures are all functional
enough for demos (with the flat-shade workaround for multi-material meshes).
These improvements primarily affect:
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
  See [benchmark-refactor/spec.md](https://github.com/yuokamoto/PyBulletFleet/blob/main/docs/design/benchmark-refactor/spec.md).

## Observability

Operational visibility for long benchmark runs and production deployments.
**Shares the EventBus with Snapshot/Replay** — same emission point, different sinks.
See [observability/spec.md](https://github.com/yuokamoto/PyBulletFleet/blob/main/docs/design/observability/spec.md)
for the design.

- **Structured logging (JSON)** — Replace prefix-string `NamedLazyLogger` with `extra={...}` dict fields (`agent_id`, `object_id`, `sim_time`, `step`). Enables Loki/Grafana label queries.
- **`sim_time` injection filter** — `logging.Filter` registered by `MultiRobotSimulationCore` auto-attaches `sim_time` and `step` to every record. Removes per-call boilerplate on hot paths.
- **Prometheus / OTLP metrics** — Promote `DataMonitor` internals (`pbf_step_duration_seconds`, `pbf_active_agents`, `pbf_actions_total{type,status}`, `pbf_collision_pairs`) to scrapable metrics. Opt-in to keep default cost zero.
- **OpenTelemetry trace exporter** — EventBus subscriber that converts action
  lifecycle events into spans.  Sampling is allowed; trace and operation IDs
  are copied into structured event records so Tempo "Trace to Logs" can locate
  the corresponding JSONL record in Loki.
- **End-to-end robot-operation tracing** — Make a single fleet operation
  debuggable across RMF task ingress, fleet/ROS command handling, the target
  agent action, and workcell/plugin completion.  Propagate a trace context and
  stable operation ID at each boundary; attach `fleet_id`, `robot_id`,
  `task_id`, and request/command IDs to spans and structured logs.  The same
  attribute vocabulary must work for a simulator-only run, ROS 2 deployment,
  and a future Robot Proxy, so traces can be compared with real-robot
  operations.  This is action/operation-level tracing, not a span per
  simulation step.
- **Hot-path safety** — All exporters use batch / async processors. Per-step spans are forbidden; Action-level granularity only. `LazyLogger.isEnabledFor()` semantics preserved.

## Long-Term: Backend Abstraction & Beyond PyBullet

Current architecture is tightly coupled to PyBullet (`body_id`, per-entity FFI
calls).  The following exploratory items investigate a backend boundary while
preserving the Python user API.  They require workload-specific benchmarks and
are not performance commitments.

### Short / Mid-Term Performance TODOs

- **Release-like profiler mode** — Add a fixed first-window profile mode (for example 1000 agents, 50% moving, first 100 measured steps after warmup) so profiler output matches release benchmark conditions. Long profiling runs can understate moving-agent cost after robots reach their goals.
- **Active moving set** — Keep an explicit set of moving agents so stationary agents skip more of the hot path.
- **AABB/grid flush reduction** — Audit whether low-frequency or disabled collision modes still refresh more AABB/spatial-grid state than needed.
- **Ground-robot 2D path** — Make the 2D collision/grid path easier to use for AGV-style fleets where Z-axis neighbor checks are unnecessary.

### Phase 1: SimBackend ABC + Numpy Pure Kinematic Backend

- **SimBackend ABC** — Abstract interface (`set_positions_batch`, `detect_collisions`, `load_model`, `step_physics`) that `Agent` and `SimObject` program against instead of raw `pybullet` calls
- **NumpyBackend (candidate default)** — Contiguous numpy arrays for
  positions/orientations and a spatial-index collision implementation.  It
  would have no physics-engine dependency in kinematic-only mode; validate
  correctness and throughput before choosing it as the default.
- **PyBulletBackend (compat)** — Wraps existing PyBullet calls behind SimBackend ABC for backward compatibility and physics-mode users
- **URDF parsing without PyBullet** — Use `yourdfpy` or similar to parse URDF into internal model data, removing the last hard dependency on PyBullet for kinematic-only mode
- **Visualization decoupling** — Replace `p.GUI` with Rerun, Open3D, or RViz for rendering. Backend-agnostic scene display

### Phase 2: Native Backend (Rust/C++ via PyO3/pybind11)

Only justified if Phase 1 numpy performance is insufficient (e.g., 5000+ agents at 240 Hz).

- **Rust kinematic core** — Position update, yaw integration, and AABB
  collision in Rust, exposed through PyO3.  Evaluate only against a measured
  NumpyBackend baseline.
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
