# Tutorial 5: End-Effector Control — IK & PoseAction

**Source files:**
- [`examples/arm/pick_drop_arm_ee_demo.py`](https://github.com/yuokamoto/PyBulletFleet/blob/main/examples/arm/pick_drop_arm_ee_demo.py) — low-level callback approach
- [`examples/arm/pick_drop_arm_ee_action_demo.py`](https://github.com/yuokamoto/PyBulletFleet/blob/main/examples/arm/pick_drop_arm_ee_action_demo.py) — action-queue approach

This tutorial shows how to control a robot arm by specifying **end-effector (EE) positions**
instead of joint angles. PyBulletFleet's built-in IK solver converts Cartesian targets
to joint angles automatically.

**What you'll learn:**

- Using `move_end_effector()` for direct EE position control
- `PoseAction` — an action that moves the EE via IK
- Using `ee_target_position` in `PickAction` / `DropAction`
- Tuning IK behaviour with `IKParams`
- Orientation control with `target_orientation`
- **Mobile manipulator IK** — auto-detection, wheel locking, `ik_joint_names`
- **`drop_relative_pose`** — dropping objects relative to the EE position

**Prerequisites:** [Tutorial 4 — Arm Joint Control](arm-pick-drop) (joint targets, `use_fixed_base`, link attachment).

---

## 1. Why EE Position Control?

[Tutorial 4](arm-pick-drop) uses `JointAction` to specify exact joint angles for pick/drop positions.
This works, but requires the user to **pre-compute** the joint angles for every target position.

EE position control inverts the problem: you specify **where the end-effector should go**
(in world coordinates), and the IK solver figures out the joint angles.

| Approach | You specify | Library computes |
|----------|------------|-----------------|
| `JointAction` | Joint angles `[1.5, 1.5, 1.5, 0.0]` | — |
| `PoseAction` / `move_end_effector` | EE position `[0.0, -0.3, 0.3]` | Joint angles via IK |

---

## 2. Direct EE Control: `move_end_effector()`

The simplest way to command the EE:

```python
# Move end-effector to position [x, y, z] in world frame
reachable = arm_agent.move_end_effector([0.0, -0.3, 0.3])

if reachable:
    print("IK solution found — joints moving to target")
else:
    print("Target unreachable — best-effort joint targets set")
```

`move_end_effector()` internally:
1. Calls `_solve_ik()` — multi-seed iterative IK solver
2. Checks reachability (`_check_ik_reachability()`)
3. Sets joint targets via `set_all_joints_targets()`
4. Returns `True` if the IK solution places the EE within tolerance

After calling `move_end_effector()`, joints move toward the solution over subsequent
`update()` steps — just like `set_all_joints_targets()`.

### With Orientation

To control the EE orientation as well as position, pass a quaternion `[x, y, z, w]`:

```python
import pybullet as p

# Downward-pointing orientation (Z- axis)
orn = list(p.getQuaternionFromEuler([0, 3.14, 0]))

reachable = arm_agent.move_end_effector(
    [0.0, -0.3, 0.3],
    target_orientation=orn,
)
```

### Checking Convergence

Use `are_joints_at_targets()` (no arguments) to check if all joints have reached
the last commanded targets from `move_end_effector()`:

```python
if arm_agent.are_joints_at_targets(tolerance=0.05):
    print("EE has reached the target position")
```

---

## 3. PoseAction — EE Control in the Action Queue

`PoseAction` wraps `move_end_effector()` as an Action for use in action sequences:

```python
from pybullet_fleet.action import PoseAction

actions = [
    PoseAction(target_position=[0.0, -0.3, 0.3], tolerance=0.02),
    PoseAction(target_position=[0.0, 0.3, 0.3], tolerance=0.02),
    PoseAction(target_position=[0.0, 0.0, 0.75]),  # home
]
arm_agent.add_action_sequence(actions)
```

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `target_position` | `list[float]` | required | EE target `[x, y, z]` in world frame |
| `target_orientation` | `list[float]` | `None` | Quaternion `[x, y, z, w]`; `None` = position only |
| `end_effector_link` | `int \| str \| None` | `None` | EE link index/name; `None` = last link |
| `tolerance` | `float` | `0.02` | EE Cartesian distance tolerance (metres) |
| `max_force` | `float` | `500.0` | Motor force for physics mode |

**Completion:** Joints within default joint tolerance of the IK solution **and** EE within `tolerance` of the target position.

**Unreachable targets:** When the IK solver determines the target is unreachable,
`PoseAction` does **not** fail immediately. Instead:

1. A best-effort IK solution is computed and joint targets are set
2. Joints move toward the best-effort targets and settle
3. After settling, the action completes with `ActionStatus.FAILED`
4. An error message is logged: `"IK target was not reachable"`

This means the action queue **does not stall** — the `FAILED` status propagates
to the next action in the sequence. To handle failures explicitly, check the
action's status after completion:

```python
action = PoseAction(target_position=[100, 100, 100])  # unreachable
# ... after execution ...
if action.status == ActionStatus.FAILED:
    print(f"Failed: {action.error_message}")
```

> **`move_end_effector()` behaviour:** Returns `False` for unreachable targets
> but still sets joint targets (best-effort). The same applies to
> `PickAction(ee_target_position=...)` and `DropAction(ee_target_position=...)`.

---

## 4. EE-Based Pick & Drop

`PickAction` and `DropAction` accept `ee_target_position` as an alternative to
joint-level positioning. When set, the action uses IK to move the EE to the
target position before picking/dropping
(see [`pick_drop_arm_ee_action_demo.py`](https://github.com/yuokamoto/PyBulletFleet/blob/main/examples/arm/pick_drop_arm_ee_action_demo.py)):

```python
from pybullet_fleet.action import PickAction, DropAction, PoseAction, WaitAction

EE_LINK = arm_agent.get_num_joints() - 1
OFFSET = Pose.from_xyz(0, 0, 0.14)  # box offset from EE

actions = [
    # Pick: move EE to box position via IK, then attach
    PickAction(
        target_object_id=box.body_id,
        use_approach=False,
        ee_target_position=[0.0, -0.3, 0.3],
        attach_link=EE_LINK,
        attach_relative_pose=OFFSET,
    ),
    # Drop: move EE to drop position via IK, then detach
    DropAction(
        drop_pose=Pose.from_xyz(0.0, 0.3, 0.1),
        use_approach=False,
        ee_target_position=[0.0, 0.3, 0.3],
    ),
    # Return home
    PoseAction(target_position=[0.0, 0.0, 0.75]),
    WaitAction(duration=0.5),
]
arm_agent.add_action_sequence(actions)
```

**Key difference from Tutorial 4:**

| Tutorial 4 (joint targets) | Tutorial 5 (EE position) |
|----------------------------|--------------------------|
| `JointAction(target_joint_positions=[...])` | `PoseAction(target_position=[x, y, z])` |
| `PickAction(use_approach=False)` + separate `JointAction` | `PickAction(ee_target_position=[x, y, z])` |
| User computes joint angles | IK solver computes angles |

---

## 5. Tuning IK with `IKParams`

The IK solver uses a multi-seed iterative strategy. You can tune its behaviour
by passing an `IKParams` dataclass to `Agent.from_urdf()`:

```python
from pybullet_fleet.agent import Agent, IKParams

ik_cfg = IKParams(
    max_outer_iterations=5,     # refinement iterations per seed (default: 5)
    convergence_threshold=0.01, # position error threshold in metres (default: 0.01)
    max_inner_iterations=200,   # PyBullet IK iterations per attempt (default: 200)
    residual_threshold=1e-4,    # IK residual threshold (default: 1e-4)
    reachability_tolerance=0.02,# tolerance for reachability check in metres (default: 0.02)
    seed_quartiles=(0.25, 0.5, 0.75),# joint range quartiles for seed generation (default)
)

arm_agent = Agent.from_urdf(
    urdf_path="robots/arm_robot.urdf",
    pose=Pose.from_xyz(0, 0, 0),
    use_fixed_base=True,
    sim_core=sim_core,
    ik_params=ik_cfg,
)
```

The solver tries the following seed strategies (each refined up to
`max_outer_iterations` times):
1. **Current joint positions** — works well when EE is near the target
2. **Quartile seeds** — joint-range fractions in `seed_quartiles` (default
   25 %, 50 %, 75 %) for diversified exploration across the joint space

For most use cases, the defaults work well. Increase `max_outer_iterations` if
the solver fails to converge for targets requiring large joint displacements.

---

## 6. Mobile Manipulator IK

On composite robots (mobile base + arm), the IK solver must ignore the base joints
and only solve for arm joints. PyBulletFleet handles this automatically:

- **FIXED joints** are skipped (PyBullet excludes them from IK)
- **Continuous joints** (lower limit ≥ upper limit, e.g. drive wheels) are **locked** at their current positions

This means `move_end_effector()` works on a mobile manipulator without extra
configuration — the wheels stay put while the arm moves to reach the target.

### Explicit control with `ik_joint_names`

For cases where auto-detection isn't enough, use `ik_joint_names` to explicitly
list the joints the IK solver is allowed to move:

```python
from pybullet_fleet.agent import Agent, IKParams

ik_cfg = IKParams(
    ik_joint_names=(
        "mount_to_shoulder",
        "shoulder_to_elbow",
        "elbow_to_wrist",
        "wrist_to_end",
    ),
)

agent = Agent.from_urdf(
    urdf_path="robots/mobile_manipulator.urdf",
    pose=Pose.from_xyz(0, 0, 0),
    sim_core=sim_core,
    ik_params=ik_cfg,
    mass=0.0,  # kinematic mode
)
```

All movable joints **not** in `ik_joint_names` are locked at their current positions.
When `ik_joint_names` is `None` (default), auto-detection is used.

See [`examples/arm/mobile_manipulator_demo.py`](https://github.com/yuokamoto/PyBulletFleet/blob/main/examples/arm/mobile_manipulator_demo.py)
for a full mobile manipulator demo with IK-based pick/drop.

### `drop_relative_pose` — Relative Drop Positioning

When an object is attached to the EE of a mobile manipulator, you often don't know
the exact world position where it should be dropped (since the base can be anywhere).
Use `drop_relative_pose` to specify an offset from the object's current position:

```python
from pybullet_fleet.action import DropAction

# Drop the object exactly where the EE currently is (no offset)
drop = DropAction(
    drop_pose=Pose.from_xyz(0, 0, 0),  # base navigation target (still used for approach/move-to-drop)
    drop_relative_pose=Pose.from_xyz(0, 0, 0),  # (0,0,0) = release in place
    use_approach=False,
)

# Drop with a small forward offset from EE
drop = DropAction(
    drop_pose=Pose.from_xyz(0, 0, 0),  # base navigation target
    drop_relative_pose=Pose.from_xyz(0.1, 0, 0),  # 10cm forward
    use_approach=False,
)
```

When `drop_relative_pose` is set, the object's final position is computed as:
*current_object_pose* × *drop_relative_pose* — instead of teleporting to `drop_pose`.

---

## 7. Low-Level Callback Approach

For full control, use `move_end_effector()` in a state-machine callback
(see `pick_drop_arm_ee_demo.py`):

```python
step_state = 0
_ee_set = False

def pick_drop_callback(sim_core, dt):
    global step_state, _ee_set

    def _move_ee_once(pos):
        global _ee_set
        if not _ee_set:
            arm_agent.move_end_effector(pos)
            _ee_set = True

    def _at_target():
        return arm_agent.are_joints_at_targets(tolerance=0.05)

    def _advance(next_state):
        global step_state, _ee_set
        step_state = next_state
        _ee_set = False

    if step_state == 0:  # Move to pick position
        _move_ee_once([0.0, -0.3, 0.3])
        if _at_target():
            _advance(1)
    elif step_state == 1:  # Pick
        arm_agent.attach_object(box, parent_link_index=EE_LINK,
                                relative_pose=Pose.from_xyz(0, 0, 0.14))
        _advance(2)
    elif step_state == 2:  # Move to drop position
        _move_ee_once([0.0, 0.3, 0.3])
        if _at_target():
            _advance(3)
    elif step_state == 3:  # Drop
        arm_agent.detach_object(box)
        _advance(0)

sim_core.register_callback(pick_drop_callback, frequency=10)
```

---

## 8. Running the Demos

```bash
# Action-queue approach (recommended)
python examples/arm/pick_drop_arm_ee_action_demo.py

# Low-level callback approach
python examples/arm/pick_drop_arm_ee_demo.py

# Rail arm — prismatic (linear) + revolute joints with EE control
python examples/arm/rail_arm_demo.py

# Mobile manipulator (base + arm) — kinematic IK pick/drop
python examples/arm/mobile_manipulator_demo.py
```

Both single-arm EE demos use kinematic mode for fast execution.

### Switching Robot Models

All EE demos accept a `--robot` argument to swap the arm model.
Pass an arm model name or a direct URDF path:

```bash
python examples/arm/pick_drop_arm_ee_action_demo.py --robot kuka_iiwa
python examples/arm/pick_drop_arm_ee_demo.py --robot arm_robot
```

| `--robot` default | Alternatives |
|-------------------|-------------|
| `panda` | `kuka_iiwa`, `arm_robot` |

See [Tutorial 6 — Robot Models](robot-models) for the full model resolution system.

---

## 9. Prismatic Joints & Rail Arms

The IK solver works with **prismatic (linear) joints** out of the box. A prismatic
joint in the kinematic chain lets the IK solver adjust both the linear position
(e.g., rail height) and revolute angles to reach the target.

[`examples/arm/rail_arm_demo.py`](https://github.com/yuokamoto/PyBulletFleet/blob/main/examples/arm/rail_arm_demo.py)
demonstrates a **rail arm** (1 prismatic Z-axis + 4 revolute = 5 DOF) picking a box
from a high shelf and placing it at a low position:

```python
agent = Agent.from_urdf(
    urdf_path="robots/rail_arm_robot.urdf",
    pose=Pose.from_xyz(0, 0, 0),
    use_fixed_base=True,
    sim_core=sim_core,
)

# EE control — IK decides optimal rail height + arm configuration
actions = [
    PickAction(
        target_object_id=box.body_id,
        use_approach=False,
        ee_target_position=[0.1, -0.3, 1.0],  # high shelf
        attach_link=EE_LINK,
        attach_relative_pose=Pose.from_xyz(0, 0, 0.14),
    ),
    DropAction(
        drop_pose=Pose.from_xyz(0.1, 0.3, 0.3),  # low position
        use_approach=False,
        ee_target_position=[0.1, 0.3, 0.3],
    ),
]
```

The demo also shows **per-joint tolerance** via dict — tightening prismatic accuracy
(±5 mm) while keeping revolute joints looser (±0.05 rad):

```python
JointAction(
    target_joint_positions={"rail_joint": 0.0, "base_to_shoulder": 0.0, ...},
    tolerance={"rail_joint": 0.005, "base_to_shoulder": 0.05, ...},
)
```

See [Tutorial 4 — JointAction Tolerance](arm-pick-drop) for the full tolerance
reference (scalar, dict, agent-level fallback).

---

## API Summary

| API | Description |
|-----|-------------|
| `agent.move_end_effector(pos, orientation)` | Direct EE position command via IK |
| `agent.are_joints_at_targets(tolerance)` | Check convergence to last targets |
| `agent._solve_ik(pos, orientation, ee_link)` | Low-level IK solver (internal) |
| `PoseAction(target_position, orientation, ...)` | EE control as an Action |
| `PickAction(ee_target_position=...)` | IK-based pick positioning |
| `DropAction(ee_target_position=...)` | IK-based drop positioning |
| `IKParams(...)` | IK solver tuning parameters |
| `IKParams(ik_joint_names=(...))` | Restrict IK to specific joints |
| `DropAction(drop_relative_pose=...)` | Drop relative to current EE position |

---

## See Also

- [Tutorial 4 — Arm Joint Control](arm-pick-drop): `JointAction`, joint-level pick/drop, tolerance reference
- [Tutorial 2 — Action System](action-system): mobile robot pick/drop with `MoveAction`
- [Rail Arm Demo](https://github.com/yuokamoto/PyBulletFleet/blob/main/examples/arm/rail_arm_demo.py): prismatic + revolute EE control
- [Mobile Manipulator Demo](https://github.com/yuokamoto/PyBulletFleet/blob/main/examples/arm/mobile_manipulator_demo.py): kinematic mobile manipulator with IK pick/drop
- [Architecture Overview](../architecture/overview): IK internals, joint control modes
