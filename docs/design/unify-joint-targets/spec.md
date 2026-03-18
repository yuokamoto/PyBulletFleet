# Unify Joint Target Storage Specification

**Date:** 2026-03-16
**Status:** Validated

## Context

The current Agent implementation has `_kinematic_joint_targets` — a dict that stores pending kinematic joint targets and **deletes entries when joints arrive**. This means:

1. `are_joints_at_targets()` always requires explicit target arguments — there is no way to ask "have the joints reached wherever they were told to go?" without re-supplying the targets.
2. `PoseAction` cannot simply call `are_joints_at_targets()` and must instead check EE Euclidean distance each step (which ignores orientation) plus a separate "no pending kinematic targets" fallback.
3. Physics-mode motors (`setJointMotorControl2`) don't record their targets at all, so there's no unified way to query convergence.

## Decision

Replace `_kinematic_joint_targets` with a unified `_last_joint_targets: Dict[int, float]` that **always** records the most recent target for each joint (both kinematic and physics mode) and **never deletes** entries upon arrival.

This also simplifies `PoseAction` to use `are_joints_at_targets()` (no args) instead of the error-prone EE distance + kinematic fallback checks.

## Requirements

- `_last_joint_targets` stores targets set via `set_joint_target()` in **both** kinematic and physics modes
- Entries persist after joints arrive (never deleted)
- `are_joints_at_targets()` accepts optional `targets=None`; when None, uses `_last_joint_targets`
- `are_all_joints_at_targets()` accepts optional `target_positions=None`; when None, uses `_last_joint_targets`
- `_update_kinematic_joints()` iterates `_last_joint_targets`, skips joints where `abs(diff) < threshold`
- `PoseAction` completion uses `agent.are_joints_at_targets()` (no args) instead of EE distance check
- Read-only property `last_joint_targets` exposes `_last_joint_targets` publicly

## Constraints

- `_kinematic_joint_targets` is removed entirely (no two-dict redundancy)
- `_kinematic_joint_positions` cache is preserved (separate concern: avoids per-step `getJointState` calls)
- Backward compatibility is not required — API signatures and existing callers may be updated freely
- No changes to `move_end_effector()` (it calls `set_all_joints_targets()` which calls `set_joint_target()`)

## Out of Scope

- Clearing / resetting `_last_joint_targets` (not needed for current use cases)
- Per-joint tolerance in `_last_joint_targets` mode
- Continuous IK tracking

## Open Questions

- None

## Success Criteria

- [ ] `_kinematic_joint_targets` removed from codebase
- [ ] `_last_joint_targets` persists after joints arrive
- [ ] `are_joints_at_targets()` with no args returns True when all recorded targets reached
- [ ] `PoseAction` uses `are_joints_at_targets()` instead of EE distance
- [ ] All 616+ existing tests pass
- [ ] New tests cover `are_joints_at_targets()` without arguments
- [ ] New tests cover `last_joint_targets` property
