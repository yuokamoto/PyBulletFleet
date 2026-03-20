# Kinematic Mobile Manipulator Specification

**Date:** 2026-03-20
**Status:** Validated

## Context

The existing `mobile_manipulator_demo.py` does not function correctly even in physics mode (`mass=1.0`): (1) arm mount is positioned at the rear of the base so the arm cannot reach targets in front, (2) the pallet target object is too large for the EE to grasp, and (3) EE-attached objects don't follow arm motion. The goal is to make a working demo in kinematic mode (`mass=0.0`), where `update_attached_objects_kinematics()` tracks link-attached objects via forward kinematics. Physics mode is out of scope.

## Decision

Modify the URDF and demo to produce a working kinematic mobile manipulator demonstration. No core library changes are expected — the existing kinematic joint interpolation and link-attachment tracking should work as-is.

## Requirements

- Mobile manipulator operates in kinematic mode (`mass=0.0`)
- Arm mount repositioned to front of base so EE can reach forward targets
- Target object changed from pallet to small box (graspable by EE)
- Full action sequence works: home → move → pick → carry → move → drop → home → return
- Attached box follows EE during arm motion and base movement

## Constraints

- Minimal core library changes (agent.py, action.py — bug fixes only)
- Kinematic base movement via `resetBasePositionAndOrientation` (existing mechanism)
- Kinematic joint interpolation via `_update_kinematic_joints` (existing mechanism)
- If core bugs are discovered during testing, fix minimally

## Out of Scope

- Simultaneous base + arm control
- Base-relative EE targeting API
- New test files (follow-up)
- Physics mode fixes
- Wheel joint filtering

## Open Questions

- [ ] Exact arm joint poses for pick/carry/home need tuning after URDF change

## Success Criteria

- [ ] Demo runs with `mass=0.0` and completes the full action sequence
- [ ] Box attaches to EE and follows arm motion correctly
- [ ] Box follows base movement while attached
- [ ] Box is placed at drop position correctly
- [ ] No core library changes required (or minimal if bug found)
