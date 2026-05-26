# Two-Phase Step + Vectorized Agent Update — Implementation Plan

**Spec:** [spec.md](spec.md)
**Branch:** `feat/ros2-bridge-v2-core`
**Date:** 2026-05-17

## Strategy

- **Phase A** (Tasks 0–6): Two-phase split. Behaviour-preserving. Ship as PR-A. **Done.**
- **Phase A.5** (Task 6.5): Public `set_poses` / `get_poses` API. Ship in PR-A. **Done.**
- **Phase B** (Tasks 7–9): Batch controllers. **Done** — shipped as a single combined PR-B1 (Omni + Diff together).
- **Phase B follow-ups** (deferred to separate PRs): analytical AABB, NumPy spatial hash, C++ extension scaffold. See "Phase B follow-ups" section below.
- **Phase C** (Tasks 10–13): Always-buffered semantics. Scheduled after Phase B follow-ups, behind a minor version bump.
- After each task: `make verify` MUST pass. After each phase: rerun the baseline benchmark and diff.

## Task 0 — Baseline measurement & audit (SERIAL, BLOCKS ALL)

Goal: capture current performance numbers on the current PC before any change, and verify D3 audit assumptions on the latest code.

### 0.1 Performance baseline

Hardware/env may differ from the roadmap's 2026-04-03 baseline (500 agents → 13.0 ms / 77 FPS). Re-measure.

Commands:
```bash
# Smoke (sanity)
make bench-smoke | tee /tmp/baseline-smoke.txt

# Sweep at the comparison scale (kinematic, simple_cube, omni MoveAction)
python benchmark/run_benchmark.py --sweep 100 500 1000 --duration 10 \
    --config benchmark/configs/collision_physics_off.yaml \
  | tee /tmp/baseline-sweep.txt

# Detailed per-phase profiling at 500 agents
python benchmark/mobile_benchmark.py --agents 500 --duration 10 --profile \
  | tee /tmp/baseline-profile-500.txt
```

Record (in `/tmp/two-phase-baseline.md`, NOT committed):
- Step time median, p95
- FPS
- `agent_update` / `step_simulation` / `collision_check` breakdowns
- CPU info: `lscpu | grep "Model name"`, `nproc`
- Python / PyBullet / NumPy versions: `python -c "import sys, pybullet, numpy; print(sys.version, pybullet.__version__, numpy.__version__)"`

**Output:** local note `/tmp/two-phase-baseline.md` (used as comparison reference; not added to repo unless we decide to publish official PC info later).

### 0.2 Code audit (verify spec assumptions)

```bash
# Verify no callsite reads PyBullet pose state mid-step (must use agent.get_pose)
grep -rn "p\.getBasePositionAndOrientation\|p\.getAABB\|p\.getBaseVelocity" \
    pybullet_fleet/ examples/ ros2_bridge/ --include='*.py'
```

For each hit confirm it is either (a) inside `SimObject` (allowed) or (b) outside `step_once()` (allowed). Document exceptions in `spec.md` Risks if any.

### 0.3 Test fixture audit

```bash
grep -n "_clear_shared_shapes\|_disable_monitor_gui" tests/conftest.py
```

Confirm autouse fixtures and identify the right place to add `_pending_pose_ids` reset.

**Acceptance:** baseline numbers captured, audit confirms no blocking callsite, fixture extension point identified.

---

## Task 1 — Add buffer field & `_in_step` flag (SERIAL, depends on Task 0)

Goal: introduce the new state without changing behaviour. After this task, set_pose still goes to PyBullet immediately.

### Edits

**`pybullet_fleet/sim_object.py`:**
- Add `self._pending_pose: Optional[Tuple[Sequence, Sequence, bool]] = None` in `__init__`
- No behaviour change yet

**`pybullet_fleet/core_simulation.py`:**
- Add `self._in_step: bool = False` in `__init__`
- Add `self._pending_pose_ids: Set[int] = set()` in `__init__`
- Wrap `step_once()` body in `try: self._in_step = True ... finally: self._in_step = False`
- Do NOT yet flush in step_once (next task)

### Tests

**`tests/test_sim_object.py`:** add minimal smoke test that `obj._pending_pose is None` after construction.

**`tests/conftest.py`:** extend autouse fixture to reset `_pending_pose_ids` on any sim_core created during test (defensive).

**Acceptance:** `make verify` passes. No behaviour change observable.

---

## Task 2 — Implement buffer write path in `_set_pose_internal` (SERIAL, depends on Task 1)

Goal: when `_in_step=True`, write to buffer instead of PyBullet. Production controllers immediately benefit; external callers unchanged.

### Edits

**`pybullet_fleet/sim_object.py:_set_pose_internal`:**
- After computing `moved` and updating `_cached_pose`, branch:
  - If `self.sim_core is not None and self.sim_core._in_step`:
    - `self._pending_pose = (position, orientation, preserve_velocity)`
    - `self.sim_core._pending_pose_ids.add(self.object_id)`
    - Skip the `p.resetBasePositionAndOrientation` + AABB + grid blocks
  - Else: existing immediate path (unchanged)

**`pybullet_fleet/core_simulation.py`:**
- Add `_flush_pending_poses()` method that iterates `_pending_pose_ids`, dispatches each buffered pose to `p.resetBasePositionAndOrientation` (preserve_velocity-aware), clears buffer + set
- Call it at end of Phase 1 inside `step_once()` (location: just before `t_sim0`, before `p.stepSimulation`)

NOTE: Phase 3 AABB batching is deferred to Task 3 to keep this task small.

### Tests

**New `tests/test_two_phase_step.py`:**
- During step (via real `MultiRobotSimulationCore.step_once()`), confirm:
  - Inside controller callback, `obj._pending_pose is not None` mid-Phase 1
  - After `step_once()` returns, `obj._pending_pose is None` and PyBullet pose matches `_cached_pose`
- Outside step, `agent.set_pose()` → PyBullet pose updates immediately (existing contract preserved)

**Acceptance:** `make verify` passes. Sanity: `make bench-smoke` runs without crash.

---

## Task 3 — Defer AABB / spatial-grid updates to Phase 3 (SERIAL, depends on Task 2)

Goal: actually realize the perf gain by batching `p.getAABB()` and spatial-grid updates after Phase 2.

### Edits

**`pybullet_fleet/sim_object.py:_set_pose_internal`:**
- Remove the per-call `_update_object_aabb()` + `_update_object_spatial_grid()` calls when `_in_step=True` (they happen in Phase 3 instead)
- Keep `_mark_object_moved()` for collision frequency accounting

**`pybullet_fleet/core_simulation.py`:**
- Add `_flush_aabb_and_grid()` method that iterates `_pending_pose_ids` (snapshot before clear, OR collect IDs during flush)
- Call it AFTER `_flush_pending_poses()`, BEFORE `check_collisions()`
- For physics objects (already in `_moved_this_step`), unchanged

### Tests

**`tests/test_two_phase_step.py`:**
- Test that collision detection sees up-to-date AABBs after step
- Test that spatial grid contains buffered objects after step

### Benchmark

Re-run baseline-sweep. Expected: meaningful improvement on kinematic 500-agent case.

**Acceptance:** `make verify` passes. Benchmark shows ≥20% step-time reduction at 500 agents kinematic (Phase A target: ≥30% by end of Task 6).

---

## Task 4 — Attached object propagation in Phase 1 end (SERIAL, depends on Task 3)

Goal: ensure parent → child pose propagation runs after all parent updates but before flush.

### Audit

Locate the current attached-object propagation site. Likely in `SimObject._set_pose_internal()` or `_update_attached_objects()`. With Task 2/3, recursive set_pose calls already buffer correctly, but propagation timing must be deterministic.

### Edits

**`pybullet_fleet/core_simulation.py`:**
- Add `_propagate_attached_objects()` method that walks `_pending_pose_ids` and for each, applies child offset → writes child buffers
- Call between Phase 1 end and Phase 2 start

### Tests

**`tests/test_two_phase_step.py`:**
- Multi-level attachment (parent → child → grandchild) all see correct poses after step

**Acceptance:** existing attached-object tests in [test_sim_object.py:588+](../../../tests/test_sim_object.py) still pass.

---

## Task 5 — Profiling fields & `agent_update` alias (SERIAL, depends on Task 4)

Goal: surface the new phases in profiling output without breaking benchmark scripts.

### Edits

**`pybullet_fleet/core_simulation.py:step_once`:**
- Add timing instrumentation around Phase 1, Phase 2, Phase 3, attached propagation
- New `_profiling_stats` keys: `phase1_compute`, `phase2_apply`, `phase3_bookkeep`, `attached_propagation`
- Keep `agent_update` populated (alias to `phase1_compute`)

### Tests

**`tests/test_two_phase_step.py`:** check return_profiling=True returns all new keys.

**Acceptance:** `make verify` + `make bench-smoke` produce sensible per-phase numbers.

---

## Task 6 — Documentation + spec update (SERIAL, depends on Task 5)

### Edits

**`docs/architecture/`:** new short doc `two-phase-step.md` explaining the contract (or extend existing architecture doc — pick the lighter-touch option after a 30s scan).

**`docs/design/two-phase-step/spec.md`:** mark Phase A as **Implemented**, record measured Phase A perf delta vs Task 0 baseline.

**`pybullet_fleet/sim_object.py:set_pose` docstring:** add note about `_in_step` semantics.

**`ros2_bridge/README.md` `_set_entity_state` description:** add note about up-to-one-step delay.

**Acceptance:** `make docs` (Sphinx) passes.

---

## Task 6.5 — Public `set_poses()` / `get_poses()` batch API (SERIAL, depends on Task 6)

Goal: D8 public API enabling ROS bridge / AgentManager / future BatchController to skip the per-agent Python dispatch.

### Edits

**`pybullet_fleet/core_simulation.py`:**
- Implement `set_poses(object_ids, positions, orientations, *, preserve_velocity=True) -> None`:
  - When `_in_step=True`: NumPy slice assigns into each `SimObject._pending_pose`, single `_pending_pose_ids.update(object_ids)`
  - When `_in_step=False`: loop calls per-object `_set_pose_internal`
- Implement `get_poses(object_ids) -> Tuple[ndarray, ndarray]`:
  - Stack each `_cached_pose` into (N,3), (N,4) arrays

**`pybullet_fleet/agent_manager.py:set_pose_all`:**
- Migrate to use `set_poses()` internally where possible (preserve `pose_factory` API)

### Tests

**`tests/test_batch_pose_api.py`:** new
- Round-trip set_poses → get_poses
- Inside step → buffered; outside step → immediate
- Mismatched array shapes raise clear error

### Benchmark

Micro-bench: `set_poses` 500 agents vs `for: set_pose()` 500 agents — expect ≥3× speedup on the dispatch portion.

**Acceptance:** `make verify` passes. `set_pose_all` benchmark shows improvement.

### PR-A boundary

PR-A = Tasks 0–6.5. Ship, get review, measure end-to-end vs baseline. Decide whether to start PR-B1 immediately or defer based on actual numbers.

---

## Task 7 — `BatchKinematicController` ABC (Phase B start, PR-B1)

Goal: shared vectorized base, no concrete subclass yet (or trivial test subclass).

### Edits

**`pybullet_fleet/controllers/batch_base.py`:** new
- `BatchKinematicController(KinematicController)` ABC
- Vectorized fields per spec D5
- `batch_advance(dt) -> Tuple[positions, orientations, moved_mask]` (abstract or default)
- `_apply_phase1(pos_array, orn_array)` (calls `sim_core.set_poses()`)
- Lifecycle: `register_agent`, `unregister_agent`, `reset`

### Tests

**`tests/test_batch_controller.py`:** new
- Construct with N=10 dummy agents
- Verify state arrays sized correctly
- Test register/unregister

**Acceptance:** `make verify` passes. No production code uses it yet.

---

## Task 8 — `BatchOmniController` concrete impl (depends on Task 7, PR-B1)

### Edits

**`pybullet_fleet/controllers/batch_omni.py`:** new
- `_registry_name = "batch_omni"`
- Concrete `batch_advance()` with NumPy TPI + slerp
- Equivalence with `OmniController` for single-agent path

### Tests

**`tests/test_batch_controller.py`:**
- For identical input, `BatchOmniController.batch_advance()` and `OmniController.compute()` produce trajectories within `1e-6` per step
- Trajectory completion conditions match

### Benchmark

500 agents, omni, MoveAction, kinematic:
- Compare `OmniController` (Phase A baseline) vs `BatchOmniController`
- Target: step time ≤2 ms (vs Phase A ≤9 ms target)

**Acceptance:** `make verify` passes. Benchmark hits target. PR-B1 ships (Tasks 7+8).

---

## Task 9 — `BatchDifferentialController` (shipped in PR-B1 alongside Tasks 7+8)

### Edits

**`pybullet_fleet/controllers/batch_differential.py`:** **Done.**
- ROTATE → FORWARD phase per waypoint with `np.where` masking on a `_phase` int8 array.
- Vectorised slerp (helper `_batch_slerp`) for the rotation phase, trapezoidal-velocity TPI shared with `BatchOmniController` (`_trapezoid_params`, `_trapezoid_distance`).
- Scope (v1): pose mode only, `direction=FORWARD`, no final-orientation alignment. Agents needing AUTO/BACKWARD or final-align keep using `DifferentialController`.

### Tests

**`tests/test_batch_differential_controller.py`:** lifecycle + equivalence (< 5e–3 m / 5e–3 rad max divergence over 400 steps) + buffered-pose-write integration. 11 tests, 93% coverage on the new module.

### Benchmark

`benchmark/batch_diff_perf.py` — n=500, collision on (default `--collision-freq 60`).

Result on the dev machine (2026-05-24, n=500, 600 steps, collision every step):

| | step time (ms) | phase1_update (ms) |
|---|---:|---:|
| per-agent `DifferentialController` | 8.38 | 4.54 |
| `BatchDifferentialController` | **4.92** | 1.70 |
| speedup | **1.70×** | −6.3% |

Diff sees a larger speedup than Omni's 1.36× because each waypoint runs two TPI phases per step, so the per-agent Python dispatch is roughly twice as costly.

**Acceptance:** `make verify` passes. Shipped in PR-B1 together with Tasks 7+8.

---

## Phase B follow-ups (deferred, separate PRs)

At the end of PR-B1 the dev-machine baseline (n=500, collision every step, simple_cube) is:

| stage | step (ms) | vs this PC pre-two-phase (8.06 ms, see `/tmp/two-phase-baseline.md`) |
|---|---:|---:|
| per-agent Omni (Phase A only) | 3.28 | 2.46× |
| Batch Omni (Phase A + B) | 2.41 | 3.34× |
| per-agent Diff (Phase A only) | 8.38 | 0.96× (similar) |
| Batch Diff (Phase A + B) | 4.92 | 1.64× |

The spec's 11× target (vs the original 13 ms / FPS 77 reference) is therefore **not reached by Phase B alone**. The remaining ceiling is Phase 3 + collision check (`phase3_aabb_grid_flush` 0.74 ms + `collision_check` 0.62 ms ≈ 57% of the batched step).

The following items are the bridge from ∼3× to the original 11× envelope and are tracked as separate roadmap entries:

### Phase B-3 — Analytical AABB (replaces `p.getAABB`)

- Cache each `SimObject`'s local extent at shape-load time (primitive: closed form; mesh: vertex bbox via `p.getCollisionShapeData`).
- Compute world-frame AABB for N objects in NumPy from `_cached_pose` + cached extent (corner-rotation or rotation-matrix-absolute-value upper bound).
- Drop `p.getAABB` from the kinematic hot path entirely.
- Feature parity is straightforward: today's `getAABB(bodyId)` already returns only the base link, so jointed agents (arms) are already excluded from broad-phase.
- Expected impact (per profiling, n=500): `phase3_aabb_grid_flush` 0.74 → ∼0.05 ms; total step ∼2.4 → ∼1.7 ms (≊ 1.4× on top of PR-B1, ≊ 4.7× vs this-PC pre-two-phase).
- Scope: ∼400 lines + ∼150 lines of tests.

### Phase B-4 — NumPy spatial hash (replaces dict-of-sets grid)

- Replace `_cached_spatial_grid: Dict[(int,int,int), Set[int]]` with SoA NumPy arrays: a `(N, 6)` AABB ndarray plus a cell-sorted index built once per collision-check step via `np.lexsort`.
- Broad-phase: intra-cell pair generation with `np.triu_indices`, neighbour-cell pairs from a 27-offset table, dedupe via `np.unique`, AABB overlap with a single vectorised comparison.
- Combine with Phase B-3's AABB SoA representation so the two land in the same data-layout change.
- Expected impact: `collision_check` 0.62 → ∼0.30 ms; combined with Phase B-3, total step ≈ 1.2 ms (≊ 6.7× vs this-PC pre-two-phase).
- Scope: ∼600 lines + test churn (∼10–15 existing tests reference `_cached_spatial_grid` internals).

### Phase B-5 — C++ extension scaffold (`_fleet_ext`)

- Triggered by Phase B-3 + B-4 landing; at that point the four hot functions are `_trapezoid_distance`, `_batch_slerp`, analytical-AABB, and spatial-hash bucketise.
- Add a minimal `pybind11` C++ extension module with CMake + CI build hooks. No behaviour change; existing NumPy implementations remain as the fallback.
- Move the four functions above to C++ with NumPy buffer protocol bindings.
- Position: `_fleet_ext` is the replacement for the *Python* hot path, not for PyBullet. PyBullet remains the URDF/IK/GUI/physics backend.
- Tracked as PR-C* (post Phase B follow-ups).

### Phase B follow-ups skipped intentionally

- **Lazy grid update / collision-frequency tuning**: already exposed via `collision_check_frequency` and `spatial_hash_cell_size_mode`. No code change planned — leave to user.
- **Batched `p.resetBasePositionAndOrientation` via C ext**: ROI too small to justify a C extension on its own (−0.2 ms). Folded into Phase B-5 as a secondary item.

---

## Phase C — Always-buffered semantics (axiom shift)

See [spec.md §D9](spec.md#d9-phase-c--always-buffered-semantics-axiom-shift) for full design.

Scheduled after Phase B lands and stabilises. Goal: collapse the dual immediate/buffered code path into a single buffered model + public `flush()` API. This is a behaviour change (minor version bump) but eliminates the `_in_step` branch and aligns the implementation with the "world only advances on step_once" axiom.

### Task 10 — Add `sim.flush()` public API

**Edits:**
- `pybullet_fleet/core_simulation.py`: new `flush()` method that runs `_flush_pending_poses()` + (Task 3) AABB/grid flush. Idempotent. Returns `None`.

**Tests:**
- `tests/test_flush_api.py`: `flush()` after `set_pose` makes PyBullet read see the new pose; `flush()` with no pending writes is a no-op; `flush()` is safe to call from a callback.

**Acceptance:** `make verify` passes.

### Task 11 — Add `set_pose_now()` / `set_pose_raw_now()` convenience wrappers

**Edits:**
- `pybullet_fleet/sim_object.py`: add `set_pose_now(pose, preserve_velocity=True)` and `set_pose_raw_now(position, orientation, preserve_velocity=True)`. Implementation: call buffered path, then `self.sim_core.flush()`.

**Tests:**
- `tests/test_sim_object.py::TestSetPoseNow`: equivalence with `set_pose` + manual `flush`; PyBullet visible immediately after return.

**Acceptance:** `make verify` passes.

### Task 12 — Audit and migrate existing tests

**Steps:**
- `grep` for tests that call `set_pose` / `set_pose_raw` immediately followed by `p.getBasePositionAndOrientation`, `p.rayTest`, `p.getClosestPoints`, `p.saveState`, etc.
- Replace each with either `set_pose_now()` (single object) or `set_pose(...); sim.flush()` (multi-object).
- Confirm no library-internal code depends on legacy immediate behaviour.

**Acceptance:** all existing tests still pass *before* Task 13 removes the immediate path.

### Task 13 — Remove immediate path + document axiom

**Edits:**
- `pybullet_fleet/sim_object.py::_set_pose_internal`: remove the `else` branch (immediate path). `set_pose` always buffers when `sim_core is not None`. No-sim-core case retains direct `p.resetBase...` (for unit-test usage).
- `docs/index.md`, `docs/architecture/`, `MultiRobotSimulationCore` class docstring: prominently document the axiom.
- `CHANGELOG.md`: behaviour change entry.
- Version bump (minor).

**Tests:**
- Re-run full suite + benchmark sweep. No regression vs Phase B baseline.

**Acceptance:** `make verify` passes. CHANGELOG and axiom docs reviewed.

---

## Out of Plan (separate follow-up PRs)

- Phase D: `SimBackend` ABC extraction (per [Long-Term Phase 1](../../roadmap.md#long-term-backend-abstraction--beyond-pybullet))
- ROS 2 bridge Pattern 2 (`/fleet/states`, `/fleet/navigate`) consuming `set_poses`/`get_poses` — covered by separate ros2_bridge roadmap item
- `BatchController` for IK arms — not in scope, future

## Verification gates

After **each task**: `make verify`
After **Phase A** (Task 6.5): `make verify` + benchmark diff + manual smoke test of [examples/scale/100robots_grid_demo.py](../../../examples/scale/100robots_grid_demo.py)
After **Phase B**: benchmark diff vs Phase A baseline at 100/500/1000 agents
After **Phase C** (Task 13): full benchmark sweep + manual axiom-doc review

## Risk register (live)

| Risk | Severity | Status |
|---|---|---|
| Hidden mid-step PyBullet pose read | Medium | To be verified in Task 0.2 |
| Attached-object timing change | Medium | Covered by Task 4 |
| Benchmark script field rename | Low | Covered by Task 5 alias |
| Performance regression on physics-mode | Low | Physics path unchanged |
| Test brittleness on `_pending_pose` leak | Low | Covered by Task 1 fixture |
