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

## Out of Scope

- Inverse Kinematics (IK) solver / `IKAction`
- New URDF models (6-DOF arm etc.)
- Runtime `mass` / kinematic mode switching
- Motion planning / trajectory interpolation
- Gripper simulation

## Open Questions

- [ ] None

## Success Criteria

- [ ] `JointAction` completes over multiple steps for kinematic (`mass=0`) robots
- [ ] `set_joint_target()` works for kinematic robots without code changes for callers
- [ ] Existing arm demos run with `mass=0.0` and arm moves smoothly (no demo logic changes needed)
- [ ] All existing tests continue to pass (zero regressions)
- [ ] Existing joint control tests parametrized with `mass=[1.0, 0.0]` to cover both modes
