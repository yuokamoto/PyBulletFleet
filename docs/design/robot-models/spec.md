# Robot Models — OSS Model Integration

**Date:** 2026-03-28
**Status:** Validated

## Context

PyBulletFleet demos use only 5 hand-crafted primitive-geometry URDFs (box/cylinder bodies).
For demo credibility and user adoption, we need real-world robot models (Panda, KUKA, Husky, etc.)
accessible with zero manual setup.

## Decision

Tiered URDF resolution with auto-detection of robot parameters:

- **Tier 1 (pybullet_data):** Panda, KUKA, Husky, Racecar, xArm — bundled with `pip install pybullet`, zero extra deps.
- **Tier 2 (ROS apt):** UR5e, TurtleBot3, Fetch, OpenManipulator — via `ros-{distro}-*-description`, declared in `package.xml`.
- **Tier 3 (robot_descriptions):** TIAGo, PR2, etc. — via `pip install robot_descriptions` optional dep.

A `resolve_model()` function maps short names to absolute paths. A `RobotProfile` auto-detects
joint info, EE link, and velocity limits from the loaded URDF via PyBullet API, with
optional manual overrides.

Existing demos gain `--robot` CLI arg to switch between built-in and real-world models.

## Requirements

- `resolve_model("panda")` returns absolute path with no extra install
- Auto-detect EE link name, joint limits, velocity limits from URDF
- Existing demos work unmodified (default = current self-made URDFs)
- `--robot panda` or `--robot husky` switches model in demos
- Arm ↔ mobile switching limited to same-category demos
- No new required dependencies; `robot_descriptions` is optional

## Constraints

- PyBulletFleet is Apache-2.0; only permissive-licensed models (BSD, MIT, Apache, Zlib)
- No URDF/mesh files committed to repo (all resolved at runtime)
- Tier 2/3 require user to install external packages; clear error messages on missing deps
- ROS Tier 2 uses Xacro in some packages (UR); may need pre-rendered URDF fallback

## Out of Scope

- Embedded mesh files in the PyBulletFleet repo
- Custom URDF conversion tooling
- Quadruped / humanoid robot support
- Physics-accurate mass/inertia tuning per model

## Open Questions

- [ ] xArm URDFs in pybullet_data are per-link, not single URDF — usable as arm?
- [ ] TurtleBot3 Xacro needs `xacro` tool — acceptable runtime dep for Tier 2?

## Success Criteria

- [ ] `resolve_model("panda")` works out-of-box after `pip install pybullet-fleet`
- [ ] `python examples/arm/pick_drop_arm_demo.py --robot panda` runs pick & place with Panda
- [ ] `python examples/scale/100robots_grid_demo.py --robot husky` runs 100 Huskys
- [ ] Auto-detected EE link matches expected link for Panda and KUKA
- [ ] Clear `FileNotFoundError` with install instructions when Tier 2/3 package missing
- [ ] All 866+ existing tests pass (no regressions)
