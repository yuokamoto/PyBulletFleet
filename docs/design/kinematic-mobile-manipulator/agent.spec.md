# Kinematic Mobile Manipulator — Agent Specification

## Requirements

- Modify `robots/mobile_manipulator.urdf` arm mount position
- Rewrite `examples/arm/mobile_manipulator_demo.py` for kinematic mode
- Verify link-attachment tracking works in kinematic mode
- No changes to core library files unless a bug is discovered

## Constraints

- Python 3.8+ compatibility
- pre-commit hooks: black, flake8, pyright
- All 723 existing tests must continue to pass

## Approach

Initial plan was to change only two files (URDF + demo) with no new files created, testing after each change. During implementation, additional core, test, and documentation updates were made to fix kinematic-mode bugs while keeping all existing tests passing.

## Design

### 1. URDF Change: `robots/mobile_manipulator.urdf`

Move the `base_to_mount` fixed joint origin from rear to front of base:

```xml
<!-- BEFORE -->
<origin xyz="-0.1 0.0 0.15" rpy="0 0 0"/>

<!-- AFTER -->
<origin xyz="0.25 0.0 0.15" rpy="0 0 0"/>
```

This positions the arm mount 25cm forward of base center (base is 0.8m long, so front edge is +0.4). Combined with arm reach ≈ 0.7m, the EE can reach ≈ 0.95m ahead of base center.

### 2. Demo Rewrite: `examples/arm/mobile_manipulator_demo.py`

#### Key changes from current demo:

| Parameter | Before | After |
|-----------|--------|-------|
| `mass` | `1.0` | `0.0` |
| Target object | pallet (0.5×0.4×0.1 mesh) | box (0.05×0.05×0.05) |
| Target mass | `0.0` | `0.0` |
| Box position | `[2.0, 0, 0.1]` | `[0.8, 0, 0.35]` (reachable) |
| Arm poses | Current values | Re-tuned for forward mount |

#### Spawn params:

```python
spawn_params = AgentSpawnParams(
    urdf_path=mobile_manipulator_urdf,
    initial_pose=Pose.from_euler(0, 0, 0.3, yaw=0),
    motion_mode=MotionMode.DIFFERENTIAL,
    max_linear_vel=2.0,
    max_linear_accel=3.0,
    max_angular_vel=1.5,
    max_angular_accel=3.0,
    mass=0.0,           # ← kinematic mode
    use_fixed_base=False,
)
```

#### Target box:

```python
box_params = SimObjectSpawnParams(
    visual_shape=ShapeParams(
        shape_type="box", half_extents=[0.05, 0.05, 0.05],
        rgba_color=[0.8, 0.2, 0.2, 1.0],
    ),
    collision_shape=ShapeParams(
        shape_type="box", half_extents=[0.05, 0.05, 0.05],
    ),
    initial_pose=Pose.from_xyz(0.8, 0.0, 0.35),
    mass=0.0,
    pickable=True,
)
```

#### Arm poses (to be tuned after URDF change):

- **home**: arm retracted upward/back (safe transport)
- **pick**: arm extended forward to reach box
- **carry**: arm lifted with box clear of ground

These need to be determined experimentally after the mount position changes. Start with:

```python
arm_home_pose = {
    "mount_to_shoulder": 0.0,
    "shoulder_to_elbow": 0.0,
    "elbow_to_wrist": 0.0,
    "wrist_to_end": 0.0,
}

arm_pick_pose = {
    "mount_to_shoulder": 0.0,        # facing forward
    "shoulder_to_elbow": 0.5,        # tilt forward
    "elbow_to_wrist": 0.3,           # extend
    "wrist_to_end": 0.0,
}

arm_carry_pose = {
    "mount_to_shoulder": 0.0,
    "shoulder_to_elbow": -0.8,
    "elbow_to_wrist": 1.2,
    "wrist_to_end": 0.0,
}
```

#### Action sequence (same structure as before):

1. `JointAction` → arm to home
2. `MoveAction` → navigate near box
3. `PickAction` → pick box at EE
4. `JointAction` → retract arm (carry)
5. `MoveAction` → navigate to drop position
6. `DropAction` → drop box
7. `JointAction` → arm to home
8. `MoveAction` → return to start

#### PickAction config:

```python
PickAction(
    target_object_id=box.body_id,
    use_approach=False,
    pick_offset=0.15,
    attach_link=end_effector_link_index,
    attach_relative_pose=Pose.from_xyz(0, 0, 0),
    joint_targets=arm_pick_pose,
    joint_tolerance=0.05,
    joint_max_force=100.0,
)
```

### 3. Kinematic Attachment Tracking

**Expected to work without code changes:**

In `Agent.update(dt)`:
1. `_update_actions(dt)` — processes actions (sets joint targets, base goals)
2. `_update_kinematic_joints(dt)` — interpolates joints via `resetJointState`
3. Base movement — `resetBasePositionAndOrientation`
4. `update_attached_objects_kinematics()` — `getLinkState(computeForwardKinematics=1)` + `multiplyTransforms` → `set_pose_raw`

Since `box.mass == 0`, no constraint is created. The box is tracked purely by step 4, which reads the freshly-updated joint positions from step 2. This should work correctly.

**If it doesn't work:** check whether `update_attached_objects_kinematics` is called when `use_fixed_base=False` — it should be, since it's guarded only by `is_urdf_robot()`.

**Note:** The previous physics-mode demo (`mass=1.0`) was never functioning correctly. Physics mode fixes are out of scope.

## File References

- `robots/mobile_manipulator.urdf` — URDF to modify (mount joint at line 119)
- `examples/arm/mobile_manipulator_demo.py` — demo to rewrite
- `pybullet_fleet/agent.py:1684-1743` — update() call order
- `pybullet_fleet/agent.py:1645-1685` — _update_kinematic_joints
- `pybullet_fleet/agent.py:2438-2458` — update_attached_objects_kinematics
- `pybullet_fleet/sim_object.py:890-989` — attach_object
- `pybullet_fleet/action.py:511-878` — PickAction
- `examples/arm/pick_drop_arm_demo.py` — reference for box spawning pattern

## Success Criteria

- [ ] `python examples/arm/mobile_manipulator_demo.py` completes full sequence
- [ ] Box follows EE during JointAction (arm motion after pick)
- [ ] Box follows base during MoveAction (base motion while attached)
- [ ] Box placed at correct drop position
- [ ] `pytest tests/ -x -q` — 723 passed (no regressions)
- [ ] `pre-commit run --all-files` — all passed
