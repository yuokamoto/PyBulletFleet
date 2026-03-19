# Linear Joint (Prismatic) Support — Agent Specification

## Requirements

- Add `robots/rail_arm_robot.urdf`: fixed base → prismatic joint (Z-axis, 0–1 m) → 4 revolute joints (same as `arm_robot.urdf`)
- `_update_kinematic_joints()` must use joint-type-aware fallback velocity when URDF `<limit velocity>` is 0 or missing
- `_solve_ik()` and `move_end_effector()` must work with prismatic joints in the chain (validated by tests, no code change expected)
- Docstrings referencing "radians" must acknowledge prismatic ("radians for revolute, metres for prismatic")
- Integration tests for `JointAction` and `PoseAction` with the rail arm in all 3 modes (physics, kinematic, physics_off)
- Demo script: `examples/rail_arm_demo.py`

## Constraints

- No new Action or entity class — reuse `Agent.from_urdf()`, `JointAction`, `PoseAction`
- All existing 685+ tests must still pass
- Follow existing code patterns (TDD, pre-commit, black, pyright, flake8)

## Approach

The existing joint pipeline is already type-agnostic — the only code change is the kinematic fallback velocity. Everything else is URDF creation, testing, documentation, and a demo.

## Design

### 1. URDF: `robots/rail_arm_robot.urdf`

Structure:
```
world_link (visual: vertical rail box)
  └─ rail_joint (prismatic, axis Z, limit 0–1 m, velocity 0.5 m/s)
       └─ rail_platform_link (visual: platform cylinder)
            └─ base_to_shoulder (revolute) → shoulder_link
                 └─ shoulder_to_elbow → elbow_link
                      └─ elbow_to_wrist → wrist_link
                           └─ wrist_to_end → end_effector
```

Rail joint: `<joint type="prismatic"><axis xyz="0 0 1"/><limit lower="0" upper="1.0" effort="200" velocity="0.5"/>`

The arm links/joints are identical to `arm_robot.urdf`. Joint indices:
- 0: `rail_joint` (prismatic)
- 1: `base_to_shoulder` (revolute)
- 2: `shoulder_to_elbow` (revolute)
- 3: `elbow_to_wrist` (revolute)
- 4: `wrist_to_end` (revolute)

### 2. Agent Code Change: `pybullet_fleet/agent.py`

**Single change: `_update_kinematic_joints()`** — replace the fixed fallback with a joint-type-aware fallback.

Current code (line ~1595):
```python
max_vel = self.joint_info[joint_index][11]
if max_vel <= 0:
    max_vel = self._KINEMATIC_JOINT_FALLBACK_VELOCITY
```

New code:
```python
max_vel = self.joint_info[joint_index][11]
if max_vel <= 0:
    joint_type = self.joint_info[joint_index][2]
    if joint_type == p.JOINT_PRISMATIC:
        max_vel = self._KINEMATIC_PRISMATIC_FALLBACK_VELOCITY
    else:
        max_vel = self._KINEMATIC_JOINT_FALLBACK_VELOCITY
```

**New constant:**
```python
_KINEMATIC_PRISMATIC_FALLBACK_VELOCITY: float = 0.5  # m/s (for prismatic joints without URDF velocity limit)
```

Place it next to the existing `_KINEMATIC_JOINT_FALLBACK_VELOCITY` constant.

**Docstring updates:**
- `_update_kinematic_joints()` docstring: "Falls back to 2.0 rad/s" → "Falls back to 2.0 rad/s for revolute or 0.5 m/s for prismatic if the URDF limit is 0 or missing."
- `_KINEMATIC_JOINT_FALLBACK_VELOCITY` comment: keep `# rad/s (revolute)`

### 3. Action Docstring: `pybullet_fleet/action.py`

`JointAction` docstring (line ~301):
- "List of target joint positions (radians)" → "List of target joint positions (radians for revolute, metres for prismatic)"

### 4. Tests

#### `tests/test_action_integration.py`

Add a `rail_arm_sim` fixture (or reuse `arm_sim` pattern) parametrized over (physics, kinematic, physics_off). Use `RAIL_ARM_URDF = "robots/rail_arm_robot.urdf"`.

Helper:
```python
def create_rail_arm_agent(sim_core, mass=None):
    return Agent.from_urdf(
        urdf_path=RAIL_ARM_URDF,
        pose=Pose.from_xyz(0, 0, 0),
        mass=mass,
        use_fixed_base=True,
        sim_core=sim_core,
    )
```

**New test class: `TestRailArmJointActionIntegration`**
- `test_prismatic_joint_reaches_target`: Set rail to 0.5 m via JointAction → verify joint 0 reaches target
- `test_mixed_prismatic_revolute_targets`: Set all 5 joints → verify all reached
- `test_prismatic_kinematic_multi_step`: Kinematic mode, large target (1.0 m) → verify takes multiple steps (not teleport)

**New test class: `TestRailArmPoseActionIntegration`**
- `test_ee_at_target_with_rail`: PoseAction to position above base → verify EE reaches target (rail + arm IK)
- `test_ee_at_different_rail_heights`: Two sequential PoseActions at different heights → verify both complete

#### `tests/test_ik.py`

(Optional) Add a test verifying `_solve_ik` with the rail arm URDF produces valid joint angles including a non-zero prismatic value.

### 5. Demo: `examples/rail_arm_demo.py`

Pattern: follow `pick_drop_arm_demo.py` structure.

Sequence:
1. Spawn rail arm agent (fixed base) with GUI
2. JointAction: raise rail to 0.5 m
3. PoseAction: move EE to a target position
4. JointAction: lower rail to 0.0 m + arm joints to home
5. Print joint states at each step

### 6. Documentation

- `docs/architecture/overview.md`: If JointAction parameter table says "radians", update to "radians for revolute, metres for prismatic"
- CHANGELOG: Add entry under next version

## File References

Files the plan agent MUST read before planning:

- `robots/arm_robot.urdf` — template for the arm portion of the new URDF
- `pybullet_fleet/agent.py:221-222` — fallback velocity constants
- `pybullet_fleet/agent.py:1571-1606` — `_update_kinematic_joints()` implementation
- `pybullet_fleet/agent.py:2027-2130` — `_solve_ik()` implementation
- `pybullet_fleet/action.py:293-340` — `JointAction` class
- `tests/test_action_integration.py:940-1010` — `arm_sim` fixture and `TestJointActionIntegration` (pattern to follow)
- `tests/test_action_integration.py:1015-1090` — `TestPoseActionIntegration` (pattern to follow)
- `examples/pick_drop_arm_demo.py` — demo script pattern to follow
- `docs/architecture/overview.md:260-280` — JointAction/PoseAction parameter docs

## Success Criteria

- [ ] `robots/rail_arm_robot.urdf` loads in PyBullet with 5 joints (1 prismatic + 4 revolute)
- [ ] `JointAction` controls prismatic joint in physics, kinematic, and physics_off modes
- [ ] `PoseAction` / IK completes with the rail arm (EE within tolerance)
- [ ] Kinematic interpolation: prismatic joint takes multiple steps (uses velocity limit, not teleport)
- [ ] Kinematic fallback velocity: prismatic uses `0.5 m/s`, revolute uses `2.0 rad/s`
- [ ] All existing tests pass unchanged
- [ ] `examples/rail_arm_demo.py` runs without errors
- [ ] Pre-commit passes (black, pyright, flake8)
