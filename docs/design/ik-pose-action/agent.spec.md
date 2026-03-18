# IK Pose Action - Agent Specification

## Requirements

- `PoseAction(target_position=[x,y,z])` moves end-effector to target via IK → JointAction delegation
- `PoseAction(target_position=[x,y,z], target_orientation=[qx,qy,qz,qw])` constrains orientation
- `end_effector_link` parameter: `Union[int, str, None]` — `None` auto-detects last link
- `PickAction(ee_target_position=[x,y,z])` and `DropAction(ee_target_position=[x,y,z])` convert EE pose to joint targets via IK before executing the PICKING/DROPPING phase
- `joint_targets` and `ee_target_position` are mutually exclusive — raise `ValueError` in `__post_init__` if both are set
- `Agent.move_end_effector(target_position, target_orientation, end_effector_link, max_force)` — public method for direct EE position commands without Action system (mirrors `set_joint_target()` pattern)
- Works in both physics and kinematic modes (delegates to existing `set_joint_target` path)
- IK solver uses URDF joint limits (lower, upper, range, rest poses)

## Constraints

- `_solve_ik()` is a private `Agent` method — not exported in `__init__.py`
- Joint limits extracted from `self.joint_info`: index 8=lower, 9=upper, 11=maxVelocity
- PyBullet `calculateInverseKinematics` requires `physicsClientId` — use `self._pid`
- `PoseAction` tolerance is Euclidean distance of EE position from target (not joint-space)
- EE link index cached in `Agent._end_effector_link_index` (lazy-init on first `_solve_ik` call)

## Approach

### Component 1: `Agent._solve_ik()` (agent.py)

```python
def _solve_ik(
    self,
    target_position: List[float],
    target_orientation: Optional[Tuple[float, float, float, float]] = None,
    end_effector_link: Union[int, str, None] = None,
) -> List[float]:
```

**Implementation:**
1. Resolve `end_effector_link` → index via `_get_end_effector_link_index(end_effector_link)`
2. Extract joint limits from `self.joint_info`:
   - `lower_limits`: `[info[8] for info in self.joint_info]`
   - `upper_limits`: `[info[9] for info in self.joint_info]`
   - `joint_ranges`: `[upper - lower for ...]`
   - `rest_poses`: current joint positions from `get_joint_state()`
3. Call `p.calculateInverseKinematics()` with limits
4. Return list of floats (one per joint)

**With orientation:**
```python
if target_orientation is not None:
    joint_angles = p.calculateInverseKinematics(
        self.body_id, ee_index, target_position, target_orientation,
        lowerLimits=lower_limits, upperLimits=upper_limits,
        jointRanges=joint_ranges, restPoses=rest_poses,
        physicsClientId=self._pid,
    )
else:
    joint_angles = p.calculateInverseKinematics(
        self.body_id, ee_index, target_position,
        lowerLimits=lower_limits, upperLimits=upper_limits,
        jointRanges=joint_ranges, restPoses=rest_poses,
        physicsClientId=self._pid,
    )
```

### Component 2: `Agent._get_end_effector_link_index()` (agent.py)

```python
def _get_end_effector_link_index(
    self,
    end_effector_link: Union[int, str, None] = None,
) -> int:
```

- If `end_effector_link` is not None: resolve via `resolve_link_index()`
- If None: return `len(self.joint_info) - 1` (last link = last joint's child link)
- Validate robot has joints — log warning and return -1 if no joints

### Component 3: `Agent.move_end_effector()` (agent.py)

```python
def move_end_effector(
    self,
    target_position: List[float],
    target_orientation: Optional[Tuple[float, float, float, float]] = None,
    end_effector_link: Union[int, str, None] = None,
    max_force: float = 500.0,
) -> None:
```

**Implementation:**
1. Call `_solve_ik(target_position, target_orientation, end_effector_link)` → joint angles
2. Call `self.set_all_joints_targets(joint_angles, max_force=max_force)`
3. Return immediately (actual movement happens in `update()` via kinematic interpolation or motor control)

**Relationship to `set_joint_target()`:**
| Method | Input | Internal path |
|--------|-------|---------------|
| `set_joint_target(idx, angle)` | Joint-space | Direct motor/kinematic |
| `move_end_effector(pos, orn)` | Cartesian | IK → `set_all_joints_targets()` |

Both are "fire and forget" — they set targets, actual movement happens in the sim loop.

### Component 4: `PoseAction` (action.py)

```python
@dataclass
class PoseAction(Action):
    target_position: List[float]                                    # EE target [x, y, z] in world frame
    target_orientation: Optional[Tuple[float, ...]] = None          # quaternion [qx, qy, qz, qw]
    end_effector_link: Union[int, str, None] = None                 # None = auto-detect
    max_force: float = 500.0
    tolerance: float = 0.02                                         # Euclidean distance (meters)

    # Internal
    _joint_action: Optional[JointAction] = field(default=None, init=False)
    _ee_link_index: int = field(default=-1, init=False)
```

**execute() flow:**
1. First call (`NOT_STARTED`):
   - `agent._solve_ik(target_position, target_orientation, end_effector_link)` → joint angles
   - Create `JointAction(target_joint_positions=joint_angles, max_force=max_force)`
   - Cache resolved EE link index for completion check
2. Subsequent calls: delegate to `_joint_action.execute(agent, dt)`
3. **Completion check**: after `JointAction` reports complete, verify EE position:
   - `p.getLinkState(agent.body_id, ee_index, computeForwardKinematics=1)` → actual EE pos
   - Euclidean distance from `target_position` ≤ `tolerance` → COMPLETED
   - If JointAction completed but EE not within tolerance → re-solve IK and retry (max 1 retry, then COMPLETED anyway to avoid infinite loop)

### Component 5: PickAction / DropAction extensions (action.py)

**New optional fields on both PickAction and DropAction:**
```python
ee_target_position: Optional[List[float]] = None      # EE target position (world frame)
ee_target_orientation: Optional[Tuple[float, ...]] = None
ee_end_effector_link: Union[int, str, None] = None     # For IK resolution
```

**Validation in `__post_init__`:**
```python
if self.joint_targets is not None and self.ee_target_position is not None:
    raise ValueError("Cannot specify both joint_targets and ee_target_position")
```

**Modification in PICKING/DROPPING phase:**
Replace the existing `if self.joint_targets:` block with:
```python
# Resolve joint targets from EE pose if specified
if self.ee_target_position is not None and self.joint_targets is None:
    self.joint_targets = agent._solve_ik(
        self.ee_target_position,
        self.ee_target_orientation,
        self.ee_end_effector_link,
    )

if self.joint_targets:
    # ... existing JointAction delegation (unchanged)
```

This happens **once** at the start of the PICKING/DROPPING phase — IK is solved, `joint_targets` is populated, then existing joint control takes over.

## Design

### Architecture

```
User API
  │
  ├── PoseAction(target_position, target_orientation)
  │       │
  │       ├── Agent._solve_ik()  →  joint angles
  │       └── JointAction         →  motor/kinematic control
  │
  ├── PickAction(ee_target_position=...)
  │       │
  │       ├── Agent._solve_ik()  →  joint_targets (computed once)
  │       └── JointAction         →  existing pick phase
  │
  └── DropAction(ee_target_position=...)
          │
          ├── Agent._solve_ik()  →  joint_targets (computed once)
          └── JointAction         →  existing drop phase
```

### Key Components

| Component | Responsibility | Location |
|-----------|---------------|----------|
| `Agent._solve_ik()` | IK wrapper with joint limits | `pybullet_fleet/agent.py` |
| `Agent._get_end_effector_link_index()` | EE link auto-detection | `pybullet_fleet/agent.py` |
| `Agent.move_end_effector()` | Direct EE position command | `pybullet_fleet/agent.py` |
| `PoseAction` | Pose→IK→JointAction adapter | `pybullet_fleet/action.py` |
| `PickAction` extensions | EE pose fields + IK resolution | `pybullet_fleet/action.py` |
| `DropAction` extensions | EE pose fields + IK resolution | `pybullet_fleet/action.py` |

### Data Flow

```
PoseAction.execute(agent, dt)
  → agent._solve_ik(pos, orn, link)
    → agent._get_end_effector_link_index(link)  → int
    → extract joint limits from agent.joint_info
    → p.calculateInverseKinematics(...)          → Tuple[float, ...]
    → return List[float]
  → JointAction(target_joint_positions=angles)
  → JointAction.execute(agent, dt)
    → agent.set_joints_targets(angles)
    → (kinematic: _update_kinematic_joints / physics: motor control)
  → verify: p.getLinkState(ee_index) within tolerance
  → COMPLETED
```

### Code Patterns

Follow `JointAction` pattern for `PoseAction`:
```python
# From action.py — JointAction pattern to follow
@dataclass
class JointAction(Action):
    target_joint_positions: Union[list, dict]
    max_force: float = 500.0
    tolerance: Union[float, list, dict] = 0.01

    def __post_init__(self):
        super().__init__()

    def execute(self, agent, dt: float) -> bool:
        if self.status == ActionStatus.NOT_STARTED:
            self.status = ActionStatus.IN_PROGRESS
            self.start_time = agent.sim_core.sim_time if agent.sim_core else 0.0
            self._log_start(agent)
            agent.set_joints_targets(self.target_joint_positions, max_force=self.max_force)
        if agent.are_joints_at_targets(self.target_joint_positions, tolerance=self.tolerance):
            self.status = ActionStatus.COMPLETED
            ...
```

Follow PickAction `joint_targets` pattern for EE pose extension:
```python
# From action.py — existing joint_targets in PickAction PICKING phase
if self.joint_targets:
    if self._joint_action is None:
        self._joint_action = JointAction(
            target_joint_positions=self.joint_targets,
            max_force=self.joint_max_force,
            tolerance=self.joint_tolerance,
        )
    if not self._joint_action.execute(agent, dt):
        return False
```

Follow `_compute_use_kinematic_joints` for lazy-init pattern:
```python
# From agent.py — lazy init with guard
self._use_kinematic_joints: bool = self._compute_use_kinematic_joints()
if self._use_kinematic_joints and self.joint_info:
    # ... init cache
```

## File References

Files the plan agent MUST read before planning:
- `pybullet_fleet/action.py:281-325` — JointAction (pattern to follow for PoseAction)
- `pybullet_fleet/action.py:383-620` — PickAction (EE pose extension point: PICKING phase)
- `pybullet_fleet/action.py:716-992` — DropAction (EE pose extension point: DROPPING phase)
- `pybullet_fleet/agent.py:240-296` — Agent.__init__ joint_info, kinematic init
- `pybullet_fleet/agent.py:1493-1555` — _compute_use_kinematic_joints, _update_kinematic_joints
- `pybullet_fleet/agent.py:1643-1830` — Joint control API (set_joint_target, etc.)
- `pybullet_fleet/tools.py:51-120` — resolve_link_index, resolve_joint_index
- `robots/arm_robot.urdf` — 4-joint arm URDF (test subject)
- `tests/test_action.py` — Action unit tests structure
- `tests/test_action_integration.py` — Integration tests with sim core
- `pybullet_fleet/__init__.py` — Public exports (PoseAction must be added)
- `pybullet_fleet/types.py` — ActionStatus enum

## Success Criteria

- [ ] `PoseAction` moves end-effector to specified XYZ within 2cm tolerance
- [ ] `PoseAction` with orientation constrains EE orientation within tolerance
- [ ] `PoseAction` works for kinematic (mass=0) and physics (mass>0) robots
- [ ] `Agent.move_end_effector()` sets joint targets via IK (fire-and-forget, like `set_joint_target`)
- [ ] `PickAction(ee_target_position=[x,y,z])` resolves IK and picks correctly
- [ ] `DropAction(ee_target_position=[x,y,z])` resolves IK and drops correctly
- [ ] `ValueError` raised when both `joint_targets` and `ee_target_position` are set
- [ ] Existing JointAction / PickAction / DropAction tests pass (zero regressions)
- [ ] Demo script: IK-based arm pick/drop cycle working
- [ ] Pre-commit hooks pass (black, pyright, flake8)
