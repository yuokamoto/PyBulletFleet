# Implementation Audit — 2026-06-20

Hands-on reference. Reflects actual code state as of this date.
Companion to [docs/roadmap.md](../roadmap.md) (design intent) and
[ros2_bridge/README.md](../../ros2_bridge/README.md#roadmap). Supersedes the
[2026-05-31 audit](2026-05-31-implementation-audit.md) (kept as a point-in-time
record); changes since then are noted inline.

---

## What changed since 2026-05-31

Implemented since the last audit (verified in code):

- **Velocity API** — `Agent.velocity` / `Agent.angular_velocity` now exist (was the P0 Snapshot prerequisite). Snapshot/Replay is now unblocked.
- **`class:` / `type:` loading everywhere** — per-agent controllers, plugins, **and batch controllers** load by registry name *or* dotted import path (`resolve_class`). Lazy + validated.
- **Controller chain wiring** — high-level controllers (patrol, random_walk) auto-wire on top of the motion-mode base via the unified `controller=` API; list form supported.
- **`<submesh>` extraction** — `sdf_loader` splits multi-part meshes (e.g. the Caddy's `polaris.dae`) by named geometry into cached OBJs, with COLLADA `<unit>` → metre conversion and `<center>`. Optional `sdf` extra (trimesh + pycollada), lazy-imported with a full-mesh fallback.
- **`model_yaw_offset`** — per-entity mesh yaw correction plumbed config → `resolve_sdf_to_urdf`.
- **sim↔RMF frame offset** (`rmf_frame_offset`) — bridge shifts published poses (+) and incoming nav goals (−) so georeferenced maps (campus, nav graph ~22 km from world origin) work.
- **`world.force_color`** — flat-shades a multi-material mesh PyBullet can't texture (strips the OBJ `mtllib`); used to grey the campus env.
- **`clean` task** — follows the zone coverage path from `fleet_manager.action_paths` (was a no-op stub).
- **Cart delivery connection** — `execute_action("delivery_pickup"/"dropoff")` → `toggle_attach` is wired.
- **Demos ported** — added **campus** (now office, clinic, hotel, airport_terminal, battle_royale, campus).

---

## Confirmed Implemented

| Feature | Notes |
|---|---|
| Two-Phase Step (compute / flush) | `core_simulation.py`; `_batch_controllers` driven each step |
| BatchKinematic / BatchOmni / BatchDifferential controllers | NumPy buffers, multi-waypoint TPI + slerp, ~1e-6 equivalence with per-agent |
| `_tpi.py` TPI primitives | shared by all controllers |
| EventBus (global + per-entity) | 12+ `SimEvents`, priority ordering, COLLISION enter/exit |
| SimPlugin ABC + YAML `plugins:` loading | `on_init/on_step/on_reset/on_shutdown`, Hz throttle, dynamic import |
| AgentPlugin (battery, workcell) + registry | `type:`/`class:` entries |
| Controller registry + chain + `type:`/`class:` | omni/differential/patrol/random_walk/batch_* |
| Velocity API | `Agent.velocity` / `angular_velocity` |
| `SimObject.from_sdf()`, IKPoseAction, CameraController | — |
| navigation_2d flag, centralized defaults (`PBF_*`, `.env`) | — |
| SDF loader: `<submesh>`, `model_yaw_offset`, `world.force_color` | trimesh-backed (optional `sdf` extra) |
| Devices: Door, Elevator/Lift | multi-floor Z + passenger attach |
| Recorder (video/GIF) | pixel only (no state serialization) |

### ros2_bridge

| Feature | Notes |
|---|---|
| Per-robot topics (odom, joint_states, cmd_vel, plan, current_goal, **diagnostics**, battery_state) | working |
| Action servers (NavigateToPose, FollowPath, FollowJointTrajectory, ExecuteAction) | working |
| Simulation services (/sim/spawn, delete, step, reset, …) | working |
| toggle_attach / attach_object | working |
| RMF fleet adapter (EasyFullControl): patrol, delivery pickup/dropoff, **clean coverage**, teleop | working |
| DoorHandler / LiftHandler / WorkcellHandler | working |
| sim↔RMF `rmf_frame_offset` | georeferenced maps |
| Demos: office, clinic, hotel, airport_terminal, battle_royale, **campus** | via Docker / launch |

---

## Confirmed Unimplemented (design docs exist)

| # | Item | Spec | Missing | Priority |
|---|------|------|---------|----------|
| 1 | **Snapshot / Event log / Replay** | snapshot-replay, eventbus | `StateSnapshot`, `SnapshotLogger`, `ReplayController` (EventBus 1–2 done; velocity prereq now done) | P0/P1 |
| 2 | **Observability** | observability | structured JSON logging, `sim_time` filter, Prometheus/OTLP, OTel exporter | P1 |
| 3 | **Co-Simulation / Robot Proxy** | co-simulation | `RobotProxy`, `CommandBuffer`, IPC; shares `StateSnapshot` schema (do Snapshot first) | P2 |
| 4 | **Behavior Tree integration** | roadmap | BT → agent behavior | mid |
| 5 | **plugin entry_points discovery** | plugin-architecture | setuptools entry_points (actions/robots/worlds); `type:`/`class:` covers practical need | low |
| 6 | **SimBackend ABC + NumpyBackend** | roadmap (long-term) | no `SimBackend`; core imports `pybullet` directly | long-term |

---

## Code Gaps — No Design Doc

Quality issues found in code, not covered by any spec. (Verified open as of 2026-06-20.)

### Correctness
- **scipy in `batch_differential.py:31`** — `BatchOmni` is scipy-free; `BatchDifferential` still imports `scipy.spatial.transform.Rotation`. Blocks full scipy removal (also used in `_motion_planning.py`, `tools.py`, `geometry.py`). Add `quat_rotate_vector`/`quat_multiply`/`quat_from_rotvec` to `geometry.py` first.
- **Silent failures at boundaries** — `remove_object()` swallows `KeyError`; `set_poses()` doesn't validate `object_ids`. Should warn/raise to surface bugs at scale.
- **Collision enter/exit stale state** — with `collision_check_frequency > 1`, a contact that appears and vanishes between checks never emits `COLLISION_ENDED`. No test.

### API consistency
- `Agent.set_pose` (has `preserve_velocity`) vs `SimObject.set_pose` (doesn't) — unify or document.
- `agent.stop()` vs `agent.cancel_path()` — overlapping semantics, unclear distinction.

### Performance (not in roadmap)
- `_moved_this_step` rebuilt as a fresh `set()` each step — a per-agent dirty flag avoids the allocation at 1000+ agents.
- Action queue is a `list` (`pop(0)` is O(N)) — `collections.deque` makes it O(1).

### Type safety
- `# type: ignore` in `geometry.py`, `controller_params.py`, `action.py` — untracked; sweep to fix or document.

---

## ros2_bridge — Unimplemented (README roadmap + gaps)

### Distribution (ROSCon goal — highest priority)
- **apt / bloom** packaging (0%); **pip release** polish (`release.sh`, PyPI Trusted Publisher); **package split + naming** decided before publishing (core PyPI / ROS apt / future proxy) — sticky after rosdistro.

### Near / Mid-term
- **Batch API ROS Wrapper (Pattern 2)** — `/fleet/states` + `/fleet/navigate` (O(N)→O(1)); needs `FleetStates.msg`. Mid.
- **Direct Python Connection (Pattern 3/4)** — fleet logic ↔ sim via Python/EventBus, no ROS layer (CI/headless, low latency). Mid–high.
- **Handler Decomposition** — split `RobotHandler` into Odometry/JointState/CmdVel/Navigation/Diagnostics handlers. Low.
- **rmf_demos fleet_adapter reference** (`robot_handler_rmf.py` + fleet_manager FastAPI) — depends on Handler Decomposition. Low.
- **`cargo_relative_pose` on Agent** — move workcell attach offset to per-robot config. Low–mid.

### Low priority
- **TrafficLight (EasyTrafficLight)** → office_mock_traffic_light / triple_H. Poor fit for the kinematic full-control bridge; only meaningful with co-simulation. Gazebo-only for now.
- **dock path-following** — currently a plain navigate to the endpoint (negligible in a kinematic sim).
- **Device Enhancements** — elevator doors, double-hinge door, xacro-parameterised device URDFs.
- **Multi-Floor Visualization** — floor on/off (transparency + 1/2/3 keys); deferred with the Rerun GUI migration.
- **Multi-material OBJ textures** — PyBullet applies one texture per mesh; campus env flat-shaded as a workaround. Proper fix: per-material sub-visuals (reuse the trimesh submesh path). Low.

### ros2_bridge code gaps (no spec)
- **No BatchController-over-ROS example** — `RobotHandler` spawns a per-agent `OmniController`; the recommended 500-robot config (BatchOmni) is neither documented nor exemplified.
- **`charge`-task deadlock pitfall** — documented (README) but easy to miss; move to a prominent "Known Limitations" section.

### Documentation
- Standard / Cart Delivery flow diagrams; Pattern 1/2/3/4 communication comparison; a `class:`-dotted-path extension example (custom controller + plugin + batch).

---

## Priority Order (2026-06-20)

### ROSCon critical path
```
1. Package structure decision (split + naming; reserve a slot for the proxy)
2. Distribution: pip release polish → apt/bloom (Jazzy)
3. (functional demos already done: office/clinic/hotel/airport/battle_royale/campus)
```

### Core (pybullet_fleet)
```
P0/P1  Snapshot/Replay (velocity prereq done) → Observability
P1     scipy removal (batch_differential + geometry helpers); silent-failure fixes
P2     core subsystem extraction (CollisionSystem / VisualizerController / SimProfiler; core_simulation.py ~3700 lines)
       Co-Simulation / Robot Proxy (after Snapshot schema is stable)
P3     SimBackend ABC + NumpyBackend; C++ extensions; GUI inspector
low    Behavior trees; crowd sim + throughput benchmark; assets; environments package
```

### ros2_bridge
```
high   Distribution (apt/pip)
mid    Direct Python Connection; Batch API ROS Wrapper; flow-diagram docs
low    Handler Decomposition; cargo_relative_pose; Device Enhancements; dock
later  TrafficLight (with co-sim); Multi-Floor Visualization (with Rerun); per-material textures
```
