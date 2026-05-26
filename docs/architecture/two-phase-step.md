# Two-Phase Step

PyBulletFleet's `step_once()` is internally a **two-phase step**. This page
explains what each phase does and what it means for callback / plugin authors.

> If you only call high-level APIs (`Agent.add_action`, `controller.set_cmd_vel`,
> `agent.get_pose`), the two-phase split is fully transparent — you do not need
> to change any code. This page is for callback / plugin / ROS-bridge authors
> who interact with poses or PyBullet directly inside a step.

## Design rationale

Separating **pure-Python compute** from **PyBullet C-API writes** lets the
simulator:

- batch pose writes for N agents in one pass (`set_poses()`),
- vectorize per-agent compute with NumPy across N agents,
- guarantee that collision detection always sees a consistent world snapshot
  (all writes applied, all AABBs refreshed) within a single step.

## The three internal phases

```
step_once()
├── PHASE 1 — UPDATE         (pure Python / NumPy; pose writes are buffered)
│     ├── pre-step events
│     ├── for obj in sim_objects:  obj.update(dt)
│     │       └── controller.compute → agent.set_pose(...)   ← buffered
│     ├── user callbacks               (registered via register_callback)
│     └── plugin on_step hooks
│
├── PHASE 2 — POSE FLUSH     (tight C-API loop)
│     └── for obj_id in _pending_pose_ids:
│             p.resetBasePositionAndOrientation(...)
│
├── stepSimulation()         (only when physics is enabled)
│
├── PHASE 3 — AABB + GRID FLUSH
│     └── for obj_id in _pending_pose_ids:
│             refresh AABB (p.getAABB)
│             update spatial-hash grid
│
├── check_collisions         (frequency-gated; sees up-to-date AABBs/grid)
└── monitor / post-step events
```

The set of "objects that moved this step" (`_pending_pose_ids`) is built up
during Phase 1 and consumed (and cleared) by Phases 2 and 3.

## `set_pose()` behaviour: buffered vs immediate

Calls to `SimObject.set_pose()` (and `agent.set_pose_raw()`) behave
differently depending on **when** they are made:

| When | Behaviour |
|---|---|
| **Outside `step_once()`** (test setup, REPL, between steps) | Immediate — writes to PyBullet right away. |
| **Inside `step_once()`** (controllers, callbacks, plugin `on_step`) | **Buffered** — only the cache is updated; the actual PyBullet pose is written at Phase 2 (a few microseconds later in the same step). |

Buffering applies to **kinematic** pose writes via `set_pose()` only. Physics-driven
rigid bodies are advanced by `stepSimulation()` itself between Phase 2 and Phase 3.

The cache is the source of truth mid-step, so:

- ✅ `agent.get_pose()` — **always correct** (reads the cache).
- ⚠️ `p.getBasePositionAndOrientation(object_id)` — returns the **previous
  step's** pose during Phase 1, because PyBullet hasn't been written yet.
- ⚠️ `p.getAABB(object_id)` — returns the **previous step's** AABB during
  Phases 1 and 2. AABBs are refreshed in Phase 3.

### Rule of thumb

> Inside a callback or plugin `on_step`, **always use the framework getter**
> (`agent.get_pose()`, `sim_object.get_pose()`), never the raw PyBullet API.

If you genuinely need PyBullet to reflect the latest pose mid-step (e.g. for a
raycast against just-moved agents), do the work in a **post-step** callback or
let it run in the next step — by then Phase 2 has flushed.

## Profiling

The per-step profiling dict (returned by `MultiRobotSimulationCore.get_profiling_stats()`)
exposes the phase split:

| Key | What it measures |
|---|---|
| `phase1_update` | End-to-end Phase 1 wall time (events → `agent.update` loop → callbacks → plugin `on_step`). Buffered writes only — no PyBullet pose calls. |
| `phase2_pose_flush` | Time spent writing buffered poses to PyBullet (`resetBasePositionAndOrientation` loop). |
| `phase3_aabb_grid_flush` | Time spent refreshing AABBs and spatial-hash grid entries for the objects flushed in Phase 2. |
| `agent_update` | Time spent in the per-object `update(dt)` loop inside Phase 1 (subset of `phase1_update`). |

## See also

- {doc}`overview` — overall architecture
- {doc}`../how-to/custom-profiling` — adding your own profiling metrics
