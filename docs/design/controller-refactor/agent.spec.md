# Controller Refactor — Implementation Spec

**Status:** Draft
**Feature:** Unified Kinematic Controller (Approach B + B3 + H1 + T1)

## Approach

Replace the dual-mode system (VelocityController vs Agent-embedded TPI) with unified controllers
that handle both velocity and pose commands internally. Each kinematic model (omni, differential)
gets one controller class that owns all movement logic for that model.

Key principles:
- Input-driven implicit mode switching with interruption (B3)
- TPI logic moves from Agent into controller subclasses (T1)
- `KinematicController` ABC provides template methods for `compute()` dispatch
- Default controller is `DifferentialController` (set when none specified)
- No backward-compatible aliases — clean rename

## Design

### Architecture

```
Controller (ABC)                           controller.py  (unchanged)
  │  compute(agent, dt) → bool
  │  from_config(config) → Controller
  │
  └── KinematicController (ABC)            controller.py  (was VelocityController)
        │  mode: ControllerMode  [IDLE | VELOCITY | POSE]
        │
        │  # Velocity interface
        │  set_velocity(vx, vy, vz, wx, wy, wz)
        │
        │  # Pose interface (NEW)
        │  set_goal_pose(agent, goal, **kwargs)
        │  set_path(agent, path, **kwargs)
        │  cancel_pose()
        │
        │  # Template method
        │  compute(agent, dt):
        │    VELOCITY → _check_watchdog → _apply_velocity(agent, dt)
        │    POSE     → _apply_pose(agent, dt)
        │    IDLE     → False
        │
        │  # Abstract (subclass implements)
        │  _apply_velocity(agent, dt) → bool
        │  _apply_pose(agent, dt) → bool
        │  _init_pose_trajectory(agent, goal) → None
        │
        ├── OmniController                 controller.py  (was OmniVelocityController)
        │     _apply_velocity: 6-DoF quat body→world Euler integration
        │     _apply_pose: per-axis TPI (owns _tpi_pos[3])
        │     _init_pose_trajectory: compute displacement ratios, init 3 TPIs
        │
        └── DifferentialController         controller.py  (was DifferentialVelocityController)
              _apply_velocity: unicycle (vx, wz)
              _apply_pose: 2-phase TPI (ROTATE→FORWARD)
              _init_pose_trajectory: compute rotation→init slerp, distance→init forward TPI
              Owns: _tpi_rotation_angle, _tpi_forward, _slerp_precomp, _differential_phase
```

### Key Components

| Component | Responsibility | Location |
|-----------|---------------|----------|
| `Controller` ABC | Base interface, `from_config` factory | `pybullet_fleet/controller.py` |
| `KinematicController` ABC | Mode state machine, velocity+pose dispatch, watchdog | `pybullet_fleet/controller.py` |
| `OmniController` | 6-DoF velocity + omnidirectional TPI pose | `pybullet_fleet/controller.py` |
| `DifferentialController` | Unicycle velocity + rotate-then-forward TPI pose | `pybullet_fleet/controller.py` |
| `ControllerMode` enum | IDLE / VELOCITY / POSE | `pybullet_fleet/types.py` |
| `Agent` | Simplified dispatch, delegates to controller | `pybullet_fleet/agent.py` |
| `RobotHandler` | Simplified — no mode toggling | `ros2_bridge/.../robot_handler.py` |
| `BridgeNode` | Default controller always assigned | `ros2_bridge/.../bridge_node.py` |

### Data Flow

**Velocity command flow:**
```
ROS cmd_vel → robot_handler.apply_cmd_vel()
  → controller.set_velocity(vx, ..., wz)   [auto-switch to VELOCITY mode]
  → Agent.update(dt)
    → controller.compute(agent, dt)
      → mode == VELOCITY → _apply_velocity(agent, dt)
        → agent.set_pose(new_pose)
```

**Pose command flow:**
```
ROS goal_pose → robot_handler._goal_pose_cb()
  → controller.set_goal_pose(agent, pose)   [auto-switch to POSE mode, init TPI]
  → Agent.update(dt)
    → controller.compute(agent, dt)
      → mode == POSE → _apply_pose(agent, dt)
        → TPI.get_point(sim_time)
        → agent.set_pose_raw(pos, orient)
        → trajectory complete? → IDLE
```

**Interruption flow (B3):**
```
POSE active + cmd_vel arrives:
  → controller.set_velocity(...)
    → cancel POSE (reset TPI state)
    → agent.stop()  (clear _is_moving, _goal_pose)
    → mode = VELOCITY

VELOCITY active + goal_pose arrives:
  → controller.set_goal_pose(agent, pose)
    → zero velocity
    → mode = POSE
    → _init_pose_trajectory(agent, goal)
```

### Code Patterns

Pattern 1 — KinematicController compute() template method:
```python
def compute(self, agent: "Agent", dt: float) -> bool:
    if self._mode == ControllerMode.VELOCITY:
        if self._check_velocity_timeout(agent):
            return False  # watchdog → IDLE
        if np.allclose(self._linear_velocity, 0.0) and np.allclose(self._angular_velocity, 0.0):
            return False
        return self._apply_velocity(agent, dt)
    elif self._mode == ControllerMode.POSE:
        return self._apply_pose(agent, dt)
    return False  # IDLE
```

Pattern 2 — set_goal_pose with mode switching:
```python
def set_goal_pose(self, agent: "Agent", goal: Pose, **kwargs) -> None:
    if self._mode == ControllerMode.VELOCITY:
        self._zero_velocity()
    self._mode = ControllerMode.POSE
    self._init_pose_trajectory(agent, goal, **kwargs)
```

Pattern 3 — set_velocity with interruption:
```python
def set_velocity(self, vx, vy=0, vz=0, wx=0, wy=0, wz=0) -> None:
    if self._mode == ControllerMode.POSE:
        self._cancel_pose()         # reset TPI state
    self._mode = ControllerMode.VELOCITY
    self._last_cmd_vel_time = ...   # watchdog reset
    # store and clamp velocity (subclass-specific)
    self._set_velocity_impl(vx, vy, vz, wx, wy, wz)
```

Pattern 4 — Agent.update() simplified:
```python
def update(self, dt: float) -> bool:
    self._update_actions(dt)
    joints_moved = self._update_kinematic_joints(dt)
    moved = False
    if not self.use_fixed_base and self._controller is not None:
        moved = self._controller.compute(self, dt)
    self.update_attached_objects_kinematics()
    return moved or joints_moved
```

Pattern 5 — Default controller in Agent construction:
```python
# In Agent.from_params() or Agent.__init__
if self._controller is None:
    if self._motion_mode == MotionMode.OMNIDIRECTIONAL:
        self._controller = OmniController(
            max_linear_vel=self._max_linear_vel,
            max_angular_vel=self._max_angular_vel,
        )
    else:
        self._controller = DifferentialController(
            max_linear_vel=self._max_linear_vel,
            max_angular_vel=self._max_angular_vel,
        )
```

## File References

Files the implementation plan MUST read before planning:

### Core (must read fully)
- `pybullet_fleet/controller.py` — current controller hierarchy (471 lines)
- `pybullet_fleet/agent.py` — TPI fields (L338-L364), set_motion_mode (L860-L897), set_path (L938-L1027), _init_omnidirectional_trajectory (L1108-L1199), _update_omnidirectional (L1201-L1246), _init_differential_rotation_trajectory (L1250-L1432), _init_differential_forward_distance_trajectory (L1434-L1497), _update_differential (L1498-L1592), update (L1777-L1826), set_controller (L1844-L1858)
- `pybullet_fleet/types.py` — MotionMode, PosePhase, MovementDirection enums
- `pybullet_fleet/__init__.py` — public exports

### Bridge (must read fully)
- `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/robot_handler.py` — mode switching logic
- `ros2_bridge/pybullet_fleet_ros/pybullet_fleet_ros/bridge_node.py` — controller creation

### Tests (must read to understand usage)
- `tests/test_action.py` — action tests that may use controllers
- `tests/test_agent_core.py` — agent core tests
- `tests/test_e2e.py` — end-to-end tests
- `ros2_bridge/pybullet_fleet_ros/tests/` — bridge tests

### Config (must check for registry names)
- `config/*.yaml` — fleet config files
- `ros2_bridge/pybullet_fleet_ros/config/*.yaml` — bridge config files

### Examples (check for old class name references)
- `examples/*.py`

## Implementation Tasks (Ordered)

### Phase 1: Types + Controller Refactor
1. Add `ControllerMode` enum to `types.py`
2. Rename `VelocityController` → `KinematicController` in `controller.py`
   - Add `_mode: ControllerMode` state
   - Add `set_goal_pose()`, `set_path()`, `cancel_pose()` methods
   - Update `compute()` template to dispatch on mode
   - Update `set_velocity()` to handle interruption (POSE→VELOCITY)
3. Rename `OmniVelocityController` → `OmniController`
   - Move `_update_omnidirectional` + `_init_omnidirectional_trajectory` from Agent
   - Implement `_apply_pose()` and `_init_pose_trajectory()`
   - Own `_tpi_pos[3]` instances
4. Rename `DifferentialVelocityController` → `DifferentialController`
   - Move `_update_differential` + `_init_differential_*_trajectory` from Agent
   - Implement `_apply_pose()` and `_init_pose_trajectory()`
   - Own `_tpi_rotation_angle`, `_tpi_forward`, `_slerp_precomp`, `_differential_phase`
5. Update registry: `"omni"` and `"differential"`

### Phase 2: Agent Simplification
6. Remove TPI fields from Agent (`_tpi_pos`, `_tpi_rotation_angle`, etc.)
7. Remove TPI methods from Agent (`_update_omnidirectional`, `_update_differential`, `_init_*_trajectory`)
8. Simplify `Agent.update()` — single controller dispatch
9. Update `set_goal_pose()` / `set_path()` to delegate to controller
10. Add default controller in `Agent.from_params()` (DifferentialController when none specified)
11. Update `set_motion_mode()` to create matching controller
12. Update `__init__.py` exports

### Phase 3: Bridge Simplification
13. Simplify `robot_handler.py` — remove `set_controller(None)` calls, remove `_vel_controller` field
14. Simplify `bridge_node.py` — remove special-case controller creation

### Phase 4: Tests + Config
15. Update all tests for new class names
16. Update config YAML files for new registry names
17. Update examples for new class names
18. Run `make verify`

## Success Criteria

- [ ] All tests pass (updated for new names/API)
- [ ] Coverage ≥ 75%
- [ ] `agent.update()` has no TPI branching — single `controller.compute()` path
- [ ] Bridge never calls `set_controller(None)`
- [ ] Both velocity and pose modes work through same controller instance
- [ ] Mode auto-switches on input per B3 rules
- [ ] Default controller is `DifferentialController` when none specified
- [ ] No backward-compatible aliases exist
- [ ] `make verify` passes
