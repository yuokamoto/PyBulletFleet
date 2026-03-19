# Linear Joint (Prismatic) Support

**Date:** 2026-03-19
**Status:** Implemented

## Context

PyBulletFleet's joint pipeline (`set_joint_target`, kinematic interpolation, `JointAction`, IK/`PoseAction`) treats all joints by index and float value — it is already joint-type-agnostic at the API level. However, no prismatic joint has ever been loaded or tested. There is no prismatic URDF, the kinematic fallback velocity assumes revolute (`2.0 rad/s`), and docstrings say "radians" in several places.

The roadmap lists "Linear Joint — Prismatic (sliding) joint entity for gates, sliding doors, and linear actuators" as a planned component.

## Decision

Add a **rail arm** URDF (Z-axis prismatic rail + existing 4-DOF revolute arm = 5 DOF) and make the minimal code changes required so that `JointAction` and `PoseAction`/IK work correctly with mixed revolute+prismatic chains. Validate with tests and a demo script.

This is **not** a new entity class — the rail arm is a standard `Agent.from_urdf()` using existing `JointAction` and `PoseAction`.

## Requirements

- New URDF: `robots/rail_arm_robot.urdf` — fixed base, 1 prismatic (Z, 0–1 m) + 4 revolute (identical to `arm_robot.urdf`)
- Kinematic interpolation: use a per-joint-type fallback velocity (prismatic → m/s, revolute → rad/s) when URDF `<limit velocity>` is 0 or missing
- IK pipeline: verify `_solve_ik()` and `PoseAction` work with a prismatic joint in the chain (no code change expected — PyBullet IK handles prismatic natively)
- Docstring/docs: update "radians" references to "radians for revolute, metres for prismatic"
- Tests: `JointAction` and `PoseAction` integration tests with the rail arm (physics, kinematic, physics_off)
- Demo: `examples/rail_arm_demo.py`

## Constraints

- No new Action or Entity class — reuse existing `JointAction` / `PoseAction`
- Must pass all existing tests unchanged (no regressions)
- URDF only uses PyBullet-compatible features (no SDF, no custom plugins)

## Out of Scope

- Elevator / conveyor / mobile rack entities (separate PR)
- `resolve_link_index` physicsClientId fix (separate PR)
- Per-joint-type tolerance (single float is sufficient for now)

## Open Questions

- [ ] Should the prismatic fallback velocity be a class constant or configurable via `IKParams`?

## Success Criteria

- [ ] `JointAction` with prismatic targets works in physics, kinematic, and physics_off modes
- [ ] `PoseAction` / IK works with the rail arm (EE reaches target at different rail heights)
- [ ] Kinematic interpolation respects prismatic velocity limits (multi-step, no teleport)
- [ ] All existing tests still pass
- [ ] Demo script runs without errors
