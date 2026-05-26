# Two-Phase Step + Vectorized Agent Update — Design Spec

**Status:** Phase A Implemented (Tasks 0–6 complete on `feat/ros2-bridge-v2-core`); Phases A.5 / B / C in progress / planned.
**Date:** 2026-05-17 (updated 2026-05-21)
**Branch:** `feat/ros2-bridge-v2-core`
**Related roadmap items:**
- [Performance > Two-Phase Step](../../roadmap.md#performance)
- [Performance > Vectorized Agent Update](../../roadmap.md#performance)
- Stepping stone toward [Long-Term Phase 1: SimBackend ABC](../../roadmap.md#long-term-backend-abstraction--beyond-pybullet)

## Motivation

Profiling baseline (500 agents, omni MoveAction, kinematic, `simple_cube.urdf`):

| Component | Time | % |
|---|---|---|
| Python overhead (TPI + slerp + dispatch) | 11.5 ms | 88% |
| `p.resetBasePositionAndOrientation` | 0.7 ms | 5% |
| `p.getAABB` | 0.5 ms | 4% |
| **Total** | **13.0 ms** | 100% (FPS 77) |

The current `step_once()` interleaves per-agent **Python compute** with **PyBullet C-API calls** (`set_pose_raw` → `resetBasePositionAndOrientation` + `getAABB` + spatial-grid update). This:

1. Prevents NumPy batch computation across agents
2. Adds N × Python↔C crossings per step (~7 PyBullet calls per agent)
3. Forces per-object spatial-grid mutations that defeat batch APIs

Goal: split into **Phase 1 (Compute) / Phase 2 (Apply) / Phase 3 (Bookkeep)** to enable batching and pave the way for `SimBackend.set_positions_batch()`.

Expected impact: **13 ms → 1.2 ms (~11×)** at 500 agents per roadmap measurement.

## Current Flow (for reference)

```
core_simulation.step_once()
├── for obj in self._sim_objects:
│   └── obj.update(dt)                       # Agent.update
│       └── controller.compute(agent, dt)
│           └── agent.set_pose_raw(pos, orn) # SimObject._set_pose_internal
│               ├── compute moved flag
│               ├── p.resetBasePositionAndOrientation(...)   ← C call (per agent)
│               ├── sim_core._mark_object_moved()
│               ├── sim_core._update_object_aabb()           ← C call (per agent)
│               └── sim_core._update_object_spatial_grid()   ← per agent
├── callbacks
├── stepSimulation (if physics)
├── check_collisions (frequency-gated)
└── monitor
```

Per-agent C-API count: ~3 (reset, getAABB, optional getBaseVelocity). At 500 agents × 240 Hz = 360,000 calls/s.

## Proposed Flow

```
core_simulation.step_once()
├── PHASE 1 — COMPUTE (pure Python / NumPy)
│   └── for obj in self._sim_objects:
│       └── obj.update(dt)
│           └── controller.compute(agent, dt)
│               └── agent.set_pose_raw(pos, orn)   ← now WRITES TO BUFFER ONLY
│                   ├── compute moved flag (unchanged)
│                   ├── update self._cached_pose (unchanged → get_pose() consistent)
│                   ├── self._pending_pose = (pos, orn)  ← NEW
│                   └── sim_core._mark_object_pending(self.object_id)  ← NEW
│
├── PHASE 2 — APPLY (tight C-API loop)
│   └── sim_core._flush_pending_poses()
│       └── for obj_id in self._pending_pose_ids:
│           └── p.resetBasePositionAndOrientation(...)   ← batched
│
├── (optional) p.stepSimulation()  — physics only
│
├── PHASE 3 — BOOKKEEP (batched)
│   └── sim_core._flush_aabb_and_grid()
│       ├── for obj_id in self._pending_pose_ids: p.getAABB()   ← batched
│       └── batch spatial-grid update
│
├── callbacks (unchanged)
├── check_collisions (unchanged, now sees up-to-date AABBs)
└── monitor (unchanged)
```

## Design Decisions

### D1. Buffer location: `SimObject._pending_pose`

Per-object `Optional[Tuple[pos, orn, preserve_velocity]]` field. `None` when no pending write.

**Why per-object** (not central dict in `sim_core`):
- Zero lookup overhead in `set_pose_raw` hot path (direct attribute set)
- Natural for sub-objects (attached payloads) that propagate from parent
- Survives object lifecycle (no stale entries when remove_object called)

`sim_core` keeps a **set** `_pending_pose_ids: Set[int]` for iteration. Sets are O(1) insert/clear and avoid scanning all sim_objects each step.

### D2. `_cached_pose` is updated **in Phase 1**, PyBullet state in Phase 2

This preserves the existing contract that `get_pose()` after `set_pose()` returns the new value. External code (tests, plugins, ROS bridge) does not see buffering.

**Consequence:** `_cached_pose` is now the **source of truth** mid-step. PyBullet's internal pose is "behind" until flush. Any code that calls `p.getBasePositionAndOrientation()` directly during Phase 1 will see stale values — but `get_pose()` correctly returns the cached value.

Audit needed (Task 0): grep for direct `p.getBasePositionAndOrientation` calls outside `SimObject`.

### D3. Backwards compatibility for direct `set_pose()` callers

#### Strategy: `sim_core._in_step` flag

`sim_core` exposes an internal flag `_in_step: bool` (set by `step_once()`). `SimObject._set_pose_internal()` branches on it:

- `_in_step == True` → write to per-object buffer (Phase 1 path)
- `_in_step == False` → call PyBullet directly (current behaviour preserved)

```python
# core_simulation.py
def step_once(self):
    self._in_step = True
    try:
        # Phase 1: compute (set_pose calls buffer)
        # Phase 2: flush
        # Phase 3: bookkeep
        ...
    finally:
        self._in_step = False
```

Cost: one bool check per `set_pose` call. Negligible.

#### Audit of existing callers (2026-05-17)

All 70 `set_pose` / `set_pose_raw` call sites classified:

| Caller class | Count | `_in_step` | Behaviour |
|---|---|---|---|
| Core controller / Agent / SimObject internal | 11 | True | Buffered (intended) |
| Plugin `on_step` (e.g. `workcell_plugin._process_return_home`) | 1 | True | Buffered — no immediate PyBullet read after |
| Python callback registered via `register_callback` (examples) | 2 | True | Buffered — no immediate PyBullet read after |
| **ROS 2 service handler** (`/set_entity_state`) | 1 | True or False | **See note below** |
| Tests / benchmarks / example setup / `set_pose_all` | ~50 | False | Immediate flush, zero change |

#### ROS 2 service decision

`ros2_bridge/.../sim_services.py:230` runs in the ROS executor thread, asynchronously to `step_once()`. The `_in_step` flag value depends on the exact moment the service handler runs:

- Service arrives **during** `step_once()` → buffered → flushed at Phase 2 (up to **one step delay**, ~4 ms at 240 Hz)
- Service arrives **between** steps → immediate flush (current behaviour)

**Decision: accept the one-step delay.** Rationale:

1. External service invocations have inherently loose timing — the network / scheduler already introduces variability far larger than 4 ms.
2. Strict synchronization between an external client and the sim step is a *client-side* concern. If the client needs deterministic sequencing it must build it externally (request → wait for confirmation → next request).
3. PyBullet itself is not thread-safe; the existing direct cross-thread `resetBasePositionAndOrientation` is already a latent race. D3b does not worsen this.
4. No `flush_immediately` kwarg is added — the API surface stays minimal.

Document the one-step semantics in `ros2_bridge/README.md` and in `_set_entity_state`'s docstring as part of Task 0.

#### Risks specific to D3b

| Risk | Mitigation |
|---|---|
| Callback (b) directly calls `p.getBasePositionAndOrientation()` instead of `agent.get_pose()` and sees stale value | Audit (Task 0) — no current callsite does this. Spec mandates `agent.get_pose()` |
| Callback (b) calls `p.getAABB()` mid-step expecting freshly-updated AABB | Currently no such callsite. Document in plugin/callback authoring guide |
| `_in_step` not reset on exception path | `try/finally` block; covered by test |


### D4. AABB / spatial grid: deferred to Phase 3

Currently `set_pose_raw` immediately calls `_update_object_aabb()` and `_update_object_spatial_grid()` for kinematic objects. These move to Phase 3.

- For **kinematic mode** (default): AABB query is `p.getAABB()`, which only requires PyBullet's internal pose to be correct. After Phase 2 it is. Phase 3 batches `getAABB()` for all `_pending_pose_ids` and rebuilds spatial grid entries in one pass.
- For **physics mode**: physics objects are already conservatively marked in `_moved_this_step` and AABB updates run in `step_once()` after `p.stepSimulation()`. Unchanged.

### D5. Vectorized controller is **opt-in**, separate from two-phase split, with shared base

Two-phase split (D1–D4) is the **prerequisite refactor**. Vectorized controllers are a follow-up that use the new contract.

**Three-layer controller hierarchy (extends existing `KinematicController`):**

```
Controller (existing ABC)
└── KinematicController (existing, per-agent state)
    ├── OmniController (existing, per-agent)
    ├── DifferentialController (existing, per-agent)
    └── BatchKinematicController (NEW ABC — shared vectorized base)
          ├── Vectorized fields (one row per managed agent):
          │     _agents: List[Agent]
          │     _start_positions: (N, 3) ndarray
          │     _directions:      (N, 3) ndarray
          │     _tpi_forward_state: structured arrays (start/end/dur/...)
          │     _tpi_rotation_state: structured arrays
          │     _slerp_precomps: List[SlerpPrecomp]   # opaque per-agent
          │     _pose_phases: (N,) int8 (ROTATE=0, FORWARD=1)
          │     _current_waypoint_index: (N,) int
          │
          ├── batch_advance(dt) -> (positions, orientations, moved_mask)
          │       # Vectorized TPI advance + slerp + waypoint switching
          │
          ├── _apply_phase1(agent_idx, pos, orn) -> None
          │       # abstract — how to write back per-agent results
          │       # Default impl: agent.set_pose_raw(pos, orn)
          │
          ├── BatchOmniController (Task 8 — first concrete batch impl)
          └── BatchDifferentialController (Task 9 — follow-up)
```

**What goes in `BatchKinematicController` (shared base):**

The current `KinematicController` already shares ~70% of its state between Omni and Diff ([controller.py:175-205](../../../pybullet_fleet/controller.py#L175)):

- Path / waypoint indexing
- TPI rotation + TPI forward state
- `SlerpPrecomp` for orientation interpolation
- `_pose_phase` state machine (ROTATE → FORWARD)
- Goal reach detection, `_align_final_orientation`
- `cmd_vel` watchdog timeout

All of this is **batchable** — these are scalar arithmetic operations that NumPy handles trivially across N agents in one call.

**What differs between Omni and Diff (handled in subclasses):**

| | `BatchOmniController` | `BatchDifferentialController` |
|---|---|---|
| Translation directions | Any direction (lateral OK) | Forward only (no sideways) |
| FORWARD orientation | Path direction or commanded | Forward-aligned (no decoupling) |
| Velocity input | `(vx, vy, vz)` body-frame | `(v, ω)` only |
| Phase logic | Simpler (no strict ROTATE gate) | Strict ROTATE → FORWARD |

**Implementation order — Omni first, Diff second:**

| Task | Content | PR |
|---|---|---|
| 7 | `BatchKinematicController` ABC + vectorized state advance + tests | PR-B1 |
| 8 | `BatchOmniController` concrete impl + benchmark vs `OmniController` | PR-B1 |
| 9 | `BatchDifferentialController` concrete impl + benchmark | PR-B2 |

**Rationale for staging:**

1. **API stabilization** — Define `batch_advance()` interface against the Omni use case first (the simpler one), validate with tests, then apply to Diff. Diff alone risks designing an API that doesn't generalize.
2. **Omni vectorization is straightforward** — Lateral motion permitted means `new_pos = start + dir * ratio` works directly. Diff has the ROTATE/FORWARD branch that resists naïve NumPy without `np.where` masking design — needs more careful design.
3. **Staged perf validation** — Land Omni and measure the actual 11× FPS gain at 500 agents before sinking effort into Diff. If Phase A alone already hits target, Phase B may not be urgent.
4. **BatchKinematicController is designed for both from day one** — Even though Diff lands later, the base class API does **not** become a "BatchOmniBase" — it's a true shared base with both controllers in mind from the start. No hacky retrofit.

**Registry / opt-in:**

- Batch controllers register as `"batch_omni"`, `"batch_diff"` in the `CONTROLLER_REGISTRY` (existing pattern via `_registry_name`).
- Agents opt in via `AgentSpawnParams(controller_type="batch_omni")`.
- Existing `OmniController` / `DifferentialController` remain the default and the fallback for agents with custom controllers or hooks that batching cannot accommodate.

**Coexistence with per-agent controllers:**

Within one `sim_core`, some agents may use `BatchOmniController` and others may use `OmniController` (e.g. one agent with a custom on-collision hook). The batch controller manages only its registered agents; per-agent controllers run their original `compute()` path. Phase 1 in `core_simulation.step_once()` calls both: `batch_controller.batch_advance()` once for the batched agents, then the existing per-agent loop for the rest.


### D6. Attached objects (parent → child pose propagation)

`SimObject._update_attached_objects()` (or equivalent — to be located in Task 0) propagates parent's pose to attached payloads. This currently runs inside `set_pose_internal`.

**Decision:** moves to **end of Phase 1**, after all `obj.update()` calls. Iterate `_pending_pose_ids` and for each, propagate to attached children (which also write to buffers). This handles chains correctly because parents are guaranteed updated before children are iterated (within one step, the dependency is already resolved by `update()` ordering — attached objects don't have controllers).

### D7. Profiling fields

New per-step profiling entries (additive, do not break existing field names):

| Field | What |
|---|---|
| `phase1_update` | End-to-end Phase 1 wall time (events_pre_step → agent.update loop → callbacks → plugin on_step). Buffered writes only; no PyBullet pose API calls. |
| `phase2_pose_flush` | Time spent in `_flush_pending_poses()` writing buffered poses to PyBullet via `resetBasePositionAndOrientation`. |
| `phase3_aabb_grid_flush` | Time spent in `_flush_aabb_and_grid()` refreshing AABBs + spatial-grid for the kinematic objects whose poses were flushed in Phase 2. |

Existing `agent_update` field is retained for backward compatibility with benchmark scripts — it now covers only the per-object `update()` loop inside Phase 1 (its original meaning), not the full phase. Attached-object propagation is **not** measured separately: it is part of Phase 1, dominated by `phase1_update`, and adding per-call timing would touch a hot path with negligible diagnostic value (most agents have no attached objects).

### D8. Public batch API: `set_poses()` / `get_poses()`

Expose the buffered write/read mechanism as **public API on `MultiRobotSimulationCore`** for callers that already have N agents in array form (ROS bridge, AgentManager, BatchKinematicController).

```python
def set_poses(
    self,
    object_ids: Sequence[int],
    positions: np.ndarray,         # shape (N, 3)
    orientations: np.ndarray,      # shape (N, 4) quaternions
    *,
    preserve_velocity: bool = True,
) -> None:
    """Set poses for N objects in one call.

    Internally honours the same _in_step flag as set_pose():
    during a step → buffered; outside a step → immediate per-object flush.
    """

def get_poses(
    self,
    object_ids: Sequence[int],
) -> Tuple[np.ndarray, np.ndarray]:
    """Return (positions (N,3), orientations (N,4)) for N objects.

    Reads from each SimObject's _cached_pose (no PyBullet call).
    """
```

**Benefits:**

| Caller | Current | With set_poses/get_poses |
|---|---|---|
| ROS 2 `/fleet/states` & `/fleet/navigate` (roadmap Pattern 2) | N × set_pose calls | 1 call, no Pose object allocation |
| `AgentManager.set_pose_all()` | Python `for` loop with Pose factory | NumPy slice assignment to buffer |
| `BatchKinematicController._apply_phase1` (Phase B) | per-agent `set_pose_raw` | 1 call with N×3, N×4 arrays |

**Implementation:**

- `set_poses()` writes directly into each `SimObject._pending_pose` via index lookup, adds all IDs to `_pending_pose_ids` in one `set.update()`.
- `get_poses()` reads `_cached_pose` of each ID and stacks into ndarray.
- No new buffering layer — reuses Phase 1/2/3 machinery.
- Outside `_in_step`, `set_poses()` iterates and calls per-object `_set_pose_internal()` for compat.

**Synergy with ROS bridge Batch API (roadmap [ros2_bridge/README.md](../../../ros2_bridge/README.md)):**

End-to-end batch becomes:
```
ROS PoseArray msg (1 message)
  → bridge handler
    → sim_core.set_poses(ids, pos_array, orn_array)  ← D8
      → 1 set update + N buffer writes (no Python dispatch loop)
        → Phase 2 flush (N × resetBasePositionAndOrientation — PyBullet limit)
```

Compared to current: 100-robot pose update reduces from **100 ROS service calls + 100 Python dispatches** to **1 ROS msg + 1 Python batch call**.

## Non-Goals

- **No SimBackend ABC yet** — that is Long-Term Phase 1. This refactor only removes the assumption that pose writes hit PyBullet immediately. The SimBackend extraction is a clean follow-up.
- **No URDF/visualizer changes** — physics setup, mesh loading, GUI are untouched.
- **No collision refactor** — `check_collisions()` is unchanged; it consumes the spatial grid which is now correct at flush time.
- **No physics mode behaviour change** — physics objects still go through `stepSimulation()`. Two-phase split affects kinematic writes only; physics writes (rare in core flow) keep current path.

## Risks

| Risk | Mitigation |
|---|---|
| Hidden caller relies on PyBullet pose being current mid-step | Audit (Task 0) + D3b immediate-flush flag |
| Attached object timing breaks (parent buffer not propagated) | Task 4 explicit attached propagation in Phase 1 end |
| Collision detection sees stale AABBs | Phase 3 runs before `check_collisions()` |
| Profiling regression in benchmarks (field rename) | Kept legacy `agent_update` populated; new fields are additive (`phase1_update`, `phase2_pose_flush`, `phase3_aabb_grid_flush`) |
| Test breakage from `_pending_pose` autouse leak between tests | Existing `_clear_shared_shapes` autouse fixture extended to reset `_pending_pose_ids` |

## Success Criteria

1. `make verify` passes (no test regressions, 75% coverage threshold maintained)
2. Benchmark `make bench-smoke` shows no regression in physics mode
3. Kinematic benchmark (500 agents, omni, simple_cube) shows **measurable** improvement (target: ≥30% reduction in step time as Phase A only; Phase B target: ≥80% reduction)
4. `agent_update` profiling field continues to populate (backward-compat)
5. No new public API surface (only `flush_immediately` kwarg added, default `False`)

## Implementation Phases

See [plan.md](plan.md) for task breakdown.

- **Phase A** (Tasks 0–6): Two-phase split, behaviour-preserving — **Implemented** (2026-05-21). 1470 tests pass; benchmark (1000 agents, kinematic, `collision_physics_off.yaml`, 10s) shows step time within noise of baseline (Phase A defers AABB/grid but PyBullet lacks a batched `getAABB` API, so the gain is realised only once Phase B vectorises the pose-write loop).
- **Phase A.5** (Task 6.5): Public `set_poses()` / `get_poses()` batch API (D8)
- **Phase B** (Tasks 7–9): `BatchKinematicController` ABC + `BatchOmni` + `BatchDiff` vectorized fast path
- **Phase C** (Tasks 10–13): Always-buffered semantics + public `flush()` (see [D9](#d9-phase-c--always-buffered-semantics-axiom-shift))
- **Phase D** (deferred to follow-up PR): SimBackend ABC extraction

---

## D9. Phase C — Always-buffered semantics (axiom shift)

**Status:** Proposed (design accepted, scheduled after Phase B lands)
**Goal:** Eliminate the dual immediate/buffered code paths in `_set_pose_internal` by adopting a single, consistent model. Make the implementation match the user's mental model rather than the legacy "write = visible" contract.

### Axiom (to be documented prominently)

> **The simulation world only advances when `step_once()` (or `run_simulation()`) is called.**
>
> Between `step_once()` calls, the simulator is *paused*. Calls to `set_pose()` (and any other mutating API) update the **logical world state** (cache + pending buffer) but **do not** propagate to PyBullet until the next `step_once()` runs Phase 2 flush, or until the caller explicitly invokes `sim.flush()`.

This axiom makes the immediate code path eliminable: all writes are buffered uniformly regardless of `_in_step`, and observers must either step the world or explicitly flush before querying PyBullet directly.

### Why this is coherent

- The current `_in_step` gate exists only to preserve the legacy "set_pose → PyBullet visible immediately" contract for code that queries PyBullet between `set_pose` and `step_once` (e.g. `p.rayTest`, `p.getClosestPoints`, `p.saveState`).
- Under the new axiom, such queries are **expected** to see stale state — it's the caller's responsibility to call `sim.flush()` first. This is consistent with "world hasn't advanced".
- Library-internal callers (e.g. `attach_object`) already follow a **read-PyBullet → compute → write** ordering, so they are not affected.

### What gets removed / added

**Removed:**
- The immediate-path branch in `SimObject._set_pose_internal()` (the legacy `p.resetBasePositionAndOrientation` block).
- The `_in_step` gating logic — `set_pose` always buffers when `sim_core is not None`.

**Added:**
- **`MultiRobotSimulationCore.flush() -> None`**: Public method that synchronously runs Phase 2 (pose flush) + Phase 3 (AABB/grid). Idempotent (no-op if no pending writes). Safe to call at any time outside `step_once()`.
- **`SimObject.set_pose_now(pose, preserve_velocity=True) -> bool`** (and `set_pose_raw_now(...)`): Convenience wrappers that call `set_pose()` followed by `self.sim_core.flush()` for callers who genuinely want immediate visibility in a single API. Equivalent to `set_pose(...); sim.flush()`.
- **Documented escape hatches** in the public API reference: list of PyBullet calls that require an explicit `flush()` first (raycast, getClosestPoints, saveState, GUI capture, etc.).

### Migration

1. Add `sim.flush()` and `set_pose_now()` APIs (additive, non-breaking).
2. Audit existing tests for "set_pose immediately followed by direct PyBullet query". Replace with either:
   - `sim.flush()` before the query (preferred — explicit), or
   - `set_pose_now()` (shorter for single-object cases).
3. Remove the immediate code path from `_set_pose_internal`. Behaviour change: any unaudited caller doing `set_pose` → direct PyBullet read will now see stale state.
4. Bump minor version + add CHANGELOG entry documenting the axiom shift.
5. Document the axiom in `docs/index.md`, `docs/architecture/`, and the `MultiRobotSimulationCore` class docstring.

### Trade-offs

| Aspect | Phase A (current) | Phase C (proposed) |
|---|---|---|
| `_set_pose_internal` paths | 2 (immediate + buffered) | 1 (buffered always) |
| `set_pose` outside step → PyBullet read consistency | ✅ visible immediately | ⚠️ requires explicit `flush()` |
| Mental model | "write = visible, unless in step" | "write = pending, until step or flush" |
| Hot-path perf (in_step) | already buffered — no change | no change |
| Non-hot-path perf (outside step) | per-call PyBullet write | per-call buffer add (~10–30 ns faster) |
| Public API surface | unchanged | +`sim.flush()`, +`set_pose_now()` |
| Breaking change for external users | none | yes — needs version bump + docs |

### Success criteria

1. `_set_pose_internal` has a single code path (no `_in_step` branch).
2. `sim.flush()` and `set_pose_now()` documented and tested.
3. All 1461+ existing tests updated and passing.
4. Benchmark shows no regression vs. Phase B baseline (or marginal improvement from one-less-branch).
5. Axiom prominently documented in at least 3 places (index.md, architecture, class docstring).

### Tasks (placeholder — to be expanded in plan.md when Phase C is scheduled)

- **Task 10**: Add `sim.flush()` (Phase 2 + 3 reuse), tests.
- **Task 11**: Add `SimObject.set_pose_now()` / `set_pose_raw_now()` convenience wrappers, tests.
- **Task 12**: Audit + migrate existing tests that depend on immediate PyBullet visibility.
- **Task 13**: Remove immediate path from `_set_pose_internal`. Document axiom. CHANGELOG + version bump.
