# Kinematic Robot Arm Specification

**Date:** 2025-03-14
**Status:** Draft

## Context

The existing arm joint control (`set_joint_target()`, `JointAction`) relies on PyBullet motor control (`setJointMotorControl2`), which only works when physics is enabled and `stepSimulation()` is called. When a robot is spawned with `mass=0` (kinematic mode), arm joints cannot be controlled, even though the mobile robot base already supports kinematic movement in this mode.

## Decision

Extend the existing joint control API to support kinematic robots (`mass=0` / `is_kinematic=True`) by using `resetJointState()` with per-step interpolation based on URDF velocity limits. The detection uses the existing `self.is_kinematic` flag (consistent with mobile robot pattern), not the global `physics` setting. No new public API or actions are added — the existing `set_joint_target()` / `JointAction` transparently work in both modes.

## Requirements

- `set_joint_target()` and all derived methods work identically for both kinematic (`mass=0`) and physics (`mass>0`) robots
- For kinematic robots, joints interpolate smoothly toward targets using URDF `<limit velocity="...">` values
- `JointAction` completes over multiple steps (not instantaneously) in kinematic mode
- All `getLinkState()` calls use `computeForwardKinematics=1` to ensure correct link poses after `resetJointState()`
- Attached objects follow link movement correctly in kinematic mode

## Constraints

- No new public API (minimum scope — internal changes only)
- Detection uses `self.is_kinematic` (= `mass == 0.0`), consistent with mobile robot kinematic pattern
- URDF velocity limits drive interpolation speed; fallback default if unspecified
- `resetJointState()` resets joint velocity to 0 — acceptable for kinematic mode

## Performance Optimizations

### Kinematic Joint Position Cache (`_kinematic_joint_positions`)

**Problem:** cProfile showed `get_joint_state()` consuming ~36% of total time in kinematic mode via repeated `p.getJointState()` calls (4700 calls for 50 arms × 100 steps).

**Solution:** Cache joint positions in `_kinematic_joint_positions: Dict[int, float]`:
- **Init:** Read from PyBullet once via batch `p.getJointStates(body_id, indices)` at `__init__`
- **Write:** `_update_kinematic_joints()` updates cache after `resetJointState()` each step
- **Read:** `get_joint_state()` returns cached `(pos, 0.0)` for kinematic robots — zero PyBullet calls

**Result:** 1.6× speedup at 50 arms (0.826 ms → 0.523 ms per step).

### Benchmark Suite Refactoring

The benchmark suite was refactored to a Worker + Orchestrator pattern:

- **`arm_benchmark.py`** — Pure worker process (no standalone mode), outputs JSON to stdout
- **`mobile_benchmark.py`** — Pure worker process, same pattern as arm
- **`run_benchmark.py`** — Orchestrator dispatching workers via subprocess, unified `--agents` arg for both types
- **`tools.py`** — Shared helpers including `load_config()` (YAML config/scenario support) and `suppress_stdout()` (C-level warning suppression)
- **`configs/arm.yaml`** — YAML config with `physics`, `kinematic`, and `baseline` scenarios
- **`--scenario`** and **`--compare`** now work for both arm and mobile types

### PyBullet C Warning Suppression

`suppress_stdout()` context manager in `tools.py` redirects OS-level fd 1 to `/dev/null`, suppressing PyBullet C library inertia warnings that cluttered benchmark and profiling output.

## Out of Scope

- Inverse Kinematics (IK) solver / `IKAction`
- New URDF models (6-DOF arm etc.)
- Runtime `mass` / kinematic mode switching
- Motion planning / trajectory interpolation
- Gripper simulation

## Open Questions

- [ ] None

## Success Criteria

- [x] `JointAction` completes over multiple steps for kinematic (`mass=0`) robots
- [x] `set_joint_target()` works for kinematic robots without code changes for callers
- [x] Existing arm demos run with `mass=0.0` and arm moves smoothly (no demo logic changes needed)
- [x] All existing tests continue to pass (zero regressions)
- [x] Existing joint control tests parametrized with `mass=[None, 0.0]` to cover both modes
- [x] Kinematic joint position cache eliminates per-step `p.getJointState()` calls (verified by mock patch test)
- [x] Benchmark suite refactored: Worker + Orchestrator, unified CLI, YAML configs
- [x] 584 tests pass, 3 skipped, 2 xfailed
