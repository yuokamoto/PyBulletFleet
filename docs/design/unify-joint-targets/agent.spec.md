# Unify Joint Target Storage - Agent Specification

## Requirements

- Replace `_kinematic_joint_targets: Dict[int, float]` with `_last_joint_targets: Dict[int, float]`
- `_last_joint_targets` records every `set_joint_target()` call (both kinematic and physics mode)
- Entries are **never deleted** when joints arrive — they persist as "last commanded" targets
- `_update_kinematic_joints()` iterates `_last_joint_targets`, skips joints where `abs(diff) < 1e-7`
- `are_all_joints_at_targets(target_positions=None)` defaults to `_last_joint_targets` when None
- `are_joints_at_targets(targets=None)` defaults to `_last_joint_targets` when None
- Property `last_joint_targets` exposes read-only copy
- `PoseAction.execute()` simplifies: remove EE distance check, use `agent.are_joints_at_targets()`
- All existing tests pass (callers updated as needed)

**Status:** Validated

## Constraints

- `_kinematic_joint_positions` cache is **not** affected (separate concern)
- `_use_kinematic_joints` flag is **not** affected
- Backward compatibility is not required — API signatures and callers may be updated freely
- `move_end_effector()` is unchanged (calls `set_all_joints_targets()` → `set_joint_target()`)
- `PickAction` / `DropAction` are unchanged (they call `_solve_ik` → populate `joint_targets`)

## Approach

Single-pass refactoring: rename dict, update `set_joint_target`, update `_update_kinematic_joints`, add `targets=None` default, simplify `PoseAction`, add tests.

## Design

### Changes in `agent.py`

#### 1. `__init__` — Replace dict (line ~282)

**Before:**
```python
# Kinematic joint control (mass=0 robots): targets for smooth interpolation
# Dict: {joint_index: target_position}
self._kinematic_joint_targets: Dict[int, float] = {}
```

**After:**
```python
# Last commanded joint targets — persists after arrival.
# Written by set_joint_target() in both kinematic and physics modes.
# Used by are_joints_at_targets(None) and _update_kinematic_joints().
self._last_joint_targets: Dict[int, float] = {}
```

#### 2. `set_joint_target()` — Always record (line ~1745)

**Before:**
```python
if self._use_kinematic_joints:
    self._kinematic_joint_targets[joint_index] = target_position
else:
    p.setJointMotorControl2(...)
```

**After:**
```python
# Always record for are_joints_at_targets(None) / PoseAction completion
self._last_joint_targets[joint_index] = target_position

if self._use_kinematic_joints:
    pass  # _update_kinematic_joints reads from _last_joint_targets
else:
    p.setJointMotorControl2(...)
```

#### 3. `_update_kinematic_joints()` — Iterate `_last_joint_targets`, skip settled (line ~1533)

**Before:**
```python
if not self._kinematic_joint_targets:
    return
reached: list = []
for joint_index, target in self._kinematic_joint_targets.items():
    ...
    if abs(diff) <= max_step:
        new_pos = target
        reached.append(joint_index)
    ...
for idx in reached:
    del self._kinematic_joint_targets[idx]
```

**After:**
```python
if not self._last_joint_targets:
    return
for joint_index, target in self._last_joint_targets.items():
    current_pos = self._kinematic_joint_positions.get(joint_index, 0.0)
    diff = target - current_pos
    if abs(diff) < 1e-7:
        continue   # Already at target — skip
    max_vel = self.joint_info[joint_index][11]
    if max_vel <= 0:
        max_vel = self._KINEMATIC_JOINT_FALLBACK_VELOCITY
    max_step = max_vel * dt
    if abs(diff) <= max_step:
        new_pos = target
    else:
        new_pos = current_pos + math.copysign(max_step, diff)
    p.resetJointState(self.body_id, joint_index, new_pos, physicsClientId=self._pid)
    self._kinematic_joint_positions[joint_index] = new_pos
```

Key difference: no `reached` list, no `del`. Settled joints are simply skipped.

#### 4. `are_all_joints_at_targets()` — Optional targets (line ~1847)

**Before:**
```python
def are_all_joints_at_targets(self, target_positions: list, tolerance=0.01) -> bool:
```

**After:**
```python
def are_all_joints_at_targets(
    self, target_positions: Optional[list] = None, tolerance=0.01
) -> bool:
    if target_positions is None:
        # Use last commanded targets
        if not self._last_joint_targets:
            return True  # No targets ever set → vacuously true
        for idx, target in self._last_joint_targets.items():
            if not self.is_joint_at_target(idx, target, tolerance if not isinstance(tolerance, (list, tuple)) else ...):
                return False
            ...
        return True
    # existing list-based logic unchanged
```

Detailed: when `target_positions is None`, iterate `_last_joint_targets` dict and check each joint individually. Use scalar tolerance (lists not supported for no-arg mode).

#### 5. `are_joints_at_targets()` — Optional targets (line ~1889)

**Before:**
```python
def are_joints_at_targets(self, targets: Union[list, dict], tolerance=0.01) -> bool:
```

**After:**
```python
def are_joints_at_targets(
    self, targets: Union[list, dict, None] = None, tolerance=0.01
) -> bool:
    if targets is None:
        return self.are_all_joints_at_targets(None, tolerance)
    ...  # existing dispatch unchanged
```

#### 6. New property: `last_joint_targets`

```python
@property
def last_joint_targets(self) -> Dict[int, float]:
    """Read-only copy of last commanded joint targets."""
    return dict(self._last_joint_targets)
```

### Changes in `action.py`

#### 7. `PoseAction.execute()` — Simplify completion (line ~366)

**Before:**
```python
# Check EE distance to target
if self._ee_link_index >= 0:
    link_state = p.getLinkState(...)
    actual_pos = np.array(link_state[0])
    distance = float(np.linalg.norm(actual_pos - np.array(self.target_position)))
    if distance <= self.tolerance:
        self.status = ActionStatus.COMPLETED
        ...
        return True

# Joints settled (kinematic mode: no pending targets)
if agent._use_kinematic_joints and not agent._kinematic_joint_targets:
    self.status = ActionStatus.COMPLETED
    ...
    return True
```

**After:**
```python
# All joints reached their IK-solved targets
if agent.are_joints_at_targets(tolerance=self.tolerance):
    self.status = ActionStatus.COMPLETED
    self.end_time = agent.sim_core.sim_time if agent.sim_core else 0.0
    self._log_end()
    return True
```

Benefits:
- Works in both kinematic and physics modes (no `_use_kinematic_joints` check)
- Implicitly verifies orientation (joint angles converged → FK matches)
- Removes `_ee_link_index` field and EE distance computation each step
- Removes `_reachable` field (no longer needed for completion logic; `move_end_effector` return is still useful for logging)

#### 8. `PoseAction` fields cleanup

Remove:
- `_ee_link_index: int = field(default=-1, init=False)` — no longer needed for per-step checks

Keep:
- `_reachable: bool = field(default=False, init=False)` — still set for logging

### Changes in `tests/test_ik.py`

#### 9. New tests

```python
class TestLastJointTargets:
    """Tests for _last_joint_targets and are_joints_at_targets(None)."""

    def test_last_joint_targets_set_on_kinematic(self, arm_agent):
        """set_joint_target records in _last_joint_targets."""
        agent, _ = arm_agent
        agent.set_joint_target(0, 1.0)
        assert agent._last_joint_targets == {0: 1.0}

    def test_last_joint_targets_persist_after_arrival(self, arm_agent):
        """_last_joint_targets entries persist after joint reaches target."""
        agent, sim_core = arm_agent
        agent.set_joint_target(0, 0.1)
        for _ in range(1000):
            sim_core.tick()
            agent.update(sim_core._dt)
        assert 0 in agent._last_joint_targets  # NOT deleted
        assert agent.are_joints_at_targets(tolerance=0.01)  # No args

    def test_are_joints_at_targets_no_args(self, arm_agent):
        """are_joints_at_targets() with no args checks _last_joint_targets."""
        agent, sim_core = arm_agent
        agent.set_all_joints_targets([0.1, 0.2, -0.1, 0.05])
        for _ in range(2000):
            sim_core.tick()
            agent.update(sim_core._dt)
        assert agent.are_joints_at_targets(tolerance=0.05)

    def test_are_joints_at_targets_no_targets_set(self, arm_agent):
        """are_joints_at_targets() returns True when no targets ever set."""
        agent, _ = arm_agent
        assert agent.are_joints_at_targets()

    def test_last_joint_targets_property(self, arm_agent):
        """last_joint_targets property returns a copy."""
        agent, _ = arm_agent
        agent.set_joint_target(1, 0.5)
        prop = agent.last_joint_targets
        assert prop == {1: 0.5}
        prop[1] = 999.0  # mutate copy
        assert agent.last_joint_targets == {1: 0.5}  # original unchanged
```

## File References

Files the plan agent MUST read before planning:
- `pybullet_fleet/agent.py:275-296` — `__init__` kinematic joint init
- `pybullet_fleet/agent.py:1523-1554` — `_update_kinematic_joints`
- `pybullet_fleet/agent.py:1724-1758` — `set_joint_target`
- `pybullet_fleet/agent.py:1847-1917` — `are_all_joints_at_targets`, `are_joints_at_targets`
- `pybullet_fleet/action.py:326-419` — `PoseAction`
- `tests/test_ik.py:316-394` — `TestPoseAction`
- `tests/test_agent_core.py:1461-1495` — existing `are_joints_at_targets` tests

## Success Criteria

- [ ] `_kinematic_joint_targets` does not appear anywhere in codebase
- [ ] `_last_joint_targets` persists after joints arrive (test verifies)
- [ ] `are_joints_at_targets()` with no args works correctly
- [ ] `are_joints_at_targets(explicit_targets)` still accepts explicit targets
- [ ] `PoseAction` uses `are_joints_at_targets()` — no EE distance check
- [ ] All 616+ existing tests pass without modification
- [ ] 5+ new tests for `_last_joint_targets` and no-arg `are_joints_at_targets`
- [ ] Pre-commit hooks pass (black, pyright, flake8)
