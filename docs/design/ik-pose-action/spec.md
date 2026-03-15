# IK Pose Action Specification

**Date:** 2026-03-15
**Status:** Draft

## Context

The kinematic arm feature (v0.1.0) provides joint-level control via `set_joint_target()` and `JointAction`. Users must manually calculate joint angles to position the end-effector, which is tedious and URDF-specific. The roadmap calls for an IK action that accepts end-effector target position and uses PyBullet's `calculateInverseKinematics()` to solve joint angles automatically.

Additionally, the existing `PickAction` / `DropAction` accept `joint_targets` (explicit angles) but offer no way to specify the arm pose in Cartesian space. Users working with arms must pre-compute joint angles themselves.

## Decision

Add four capabilities:

1. **`PoseAction`** — A new Action that accepts an end-effector target pose (position + optional orientation) and internally solves IK → delegates to `JointAction` for execution.
2. **`Agent._solve_ik()`** — Internal method wrapping `p.calculateInverseKinematics()` with URDF joint limits.
3. **`Agent.move_end_effector()`** — Public method for direct EE position commands without the Action system (mirrors `set_joint_target()` for joint-level control). Calls `_solve_ik()` → `set_joints_targets()` internally.
4. **PickAction / DropAction EE pose parameters** — New optional `ee_target_position` / `ee_target_orientation` fields that internally convert to `joint_targets` via IK, enabling position-based pick/drop.

## Requirements

- `PoseAction` accepts `target_position` (XYZ relative to base) and optional `target_orientation` (quaternion)
- End-effector link auto-detected as the last link in the URDF chain; overridable via `end_effector_link` parameter
- `PoseAction` completes over multiple steps (same as `JointAction` — smooth interpolation in kinematic mode)
- Works transparently in both physics and kinematic modes (inherits from existing joint control path)
- `PickAction` / `DropAction` accept `ee_target_position` as alternative to `joint_targets` (mutually exclusive)
- `Agent.move_end_effector(target_position, target_orientation, end_effector_link)` provides direct EE control (analogous to `set_joint_target()` for joints)

## Constraints

- IK solver: PyBullet's built-in `calculateInverseKinematics()` only (no external IK libraries)
- Joint limits (lower/upper bounds, ranges, rest poses) extracted from `self.joint_info` and passed to the solver
- `_solve_ik()` returns a list of joint angles; no reachability validation beyond what PyBullet provides
- End-effector link detection is simple: last link index = `len(self.joint_info) - 1`
- Target position is relative to world frame (same as PyBullet IK convention)

## Out of Scope

- Public `solve_ik()` API on Agent (IK solver stays internal)
- Reachability checking / workspace analysis
- Motion planning / collision-aware trajectories
- New URDF models
- Gripper simulation
- Continuous IK tracking (servo mode)

## Open Questions

- [ ] None

## Success Criteria

- [ ] `PoseAction` moves end-effector to specified XYZ position within tolerance
- [ ] `PoseAction` with orientation constrains end-effector orientation
- [ ] `PoseAction` works for both kinematic (mass=0) and physics (mass>0) robots
- [ ] `PickAction(ee_target_position=...)` moves arm to position before picking
- [ ] `DropAction(ee_target_position=...)` moves arm to position before dropping
- [ ] `Agent.move_end_effector()` moves EE to target without Action system
- [ ] `joint_targets` and `ee_target_position` are mutually exclusive (error if both specified)
- [ ] Existing `JointAction`, `PickAction`, `DropAction` tests continue to pass (zero regressions)
- [ ] Demo script demonstrates IK-based pick/drop cycle
