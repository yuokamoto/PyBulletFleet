# Controller Configuration

How to pick a controller and pass its shared parameters
(`ControllerParams`) and impl-specific options to an
[`Agent`](../api/index).

The same `controller` argument is accepted at every layer of the API
(YAML → `AgentSpawnParams` → `Agent.from_params` / `from_urdf` /
`from_mesh` → `Agent.__init__`).  All parsing happens in **one place**
(`Agent.__init__`); the higher layers just forward the value verbatim.

## TL;DR — Five ways to specify a controller

```python
from pybullet_fleet.controller import DifferentialController
from pybullet_fleet.controller_params import ControllerParams

# 1) Implicit — framework default (OmniController) with default params.
#    motion_mode is a legacy hint; prefer the explicit forms below.
Agent.from_urdf("robot.urdf")
```

For **batch controllers** (vectorised, shared across many agents) use
`AgentManager` with `batch_controller=`.  See § *Batch Controllers* below for the
full API.

```python
from pybullet_fleet.agent_manager import AgentManager

mgr = AgentManager(sim_core=sim, batch_controller="batch_differential")
agents = mgr.spawn_agents_grid(100, grid_params, spawn_params)
bc = mgr.batch_controller   # BatchDifferentialController
```

```python

# 2) String shortcut — registry name (recommended for built-ins)
Agent.from_urdf("robot.urdf", controller="differential")

# 3) Full config dict — ★ recommended for YAML / config-driven workflows
Agent.from_urdf("robot.urdf", controller={
    "type": "differential",
    "max_linear_vel": 1.5,        # → ControllerParams.max_linear_vel
    "max_angular_vel": 2.0,       # → ControllerParams.max_angular_vel
    "navigation_2d": True,        # → ControllerParams.navigation_2d (behaviour flag)
    "wheel_separation": 0.3,      # → DifferentialController.__init__ (impl extra)
})

# 4) Controller instance — ★ recommended for Python workflows that need
#    custom controllers, complex construction, or type-safe injection.
#    Installed verbatim; motion_mode is ignored for selection.
Agent.from_urdf(
    "robot.urdf",
    controller=DifferentialController(
        params=ControllerParams(max_linear_vel=1.5, max_angular_vel=2.0),
        wheel_separation=0.3,
    ),
)

# 5) ControllerParams instance — params-only override; motion_mode still
#    picks the Omni/Diff class.  Use only when keeping the default class.
Agent.from_urdf("robot.urdf", controller=ControllerParams(max_linear_vel=2.0))
```

YAML equivalent (consumed by `AgentSpawnParams.from_dict`):

```yaml
agent:
  urdf_path: robot.urdf
  controller:                        # single source of truth
    type: differential
    max_linear_vel: 1.5
    max_angular_vel: 2.0
    wheel_separation: 0.3
```

```{note}
**YAML can express forms 1–3 and 5 only.**  Form 4 (a pre-built
`Controller` instance) is Python-only — it is the escape hatch for
custom controller subclasses or constructions that don't fit cleanly in
a config dict.
```

```{note}
`motion_mode` (`omnidirectional` / `differential`) is still accepted at
both the Python and YAML layers but is **deprecated** as a controller
selector — use `controller=` instead.  See § *motion_mode vs
controller.type* below for the interaction rules and the one corner where
`motion_mode` is still meaningful (a coarse "2D mobile base shape" hint
for batched controllers).
```

## Recommended pattern — pick by use case

| Use case | Recommended form | Why |
|----------|------------------|-----|
| YAML / config-file driven simulation | **(3) dict with `type`** | Single source of truth; survives a round-trip through `AgentSpawnParams.from_dict`. |
| Quick built-in with defaults | **(2) string** | Shortest path; no boilerplate. |
| Custom `Controller` subclass, or anything needing non-trivial construction | **(4) `Controller` instance** | Build it however you like in Python and inject the exact instance — no dict serialization required. |
| Tweak shared params only, keep default Omni/Diff class chosen by `motion_mode` | **(5) `ControllerParams` instance** | Avoids re-specifying `type`. |
| Don't care, prototyping | **(1) `None`** | Falls back to defaults. |

## The five accepted forms of `controller`

| Form | Example | What happens |
|------|---------|--------------|
| `None` | `controller=None` (default) | Fallback path: `motion_mode` picks `OmniController` / `DifferentialController` with framework-default `ControllerParams`. |
| `str` | `controller="patrol"` | Equivalent to `{"type": "patrol"}` — registry lookup, no `ControllerParams` overrides. |
| `dict` | `controller={"type": "...", "max_linear_vel": 1.5, ...}` | **Recommended for config-driven flows.**  `type` selects the controller class; the remaining keys are split between `ControllerParams` (see § *ControllerParams* below) and the controller's `__init__` (impl extras). |
| `Controller` instance | `controller=DifferentialController(params=..., wheel_separation=0.3)` | **Recommended for Python flows with custom controllers.**  Installed as-is via `set_controller`; `controller_params` is taken from `controller.params`.  `motion_mode` is ignored for selection. |
| `ControllerParams` instance | `controller=ControllerParams(max_linear_vel=2.0)` | Python-only escape hatch.  The instance is used as-is; `motion_mode` still picks the controller class (no `type` override). |

## Two-step build inside `Agent.__init__`

The controller is created **exactly once** during `__init__` (via
`self.set_controller(...)`).  No intermediate Omni/Diff instance is
built and discarded when an explicit `type` is given.

```{mermaid}
flowchart TD
    A[controller arg] --> B{kind?}
    B -- Controller instance --> P[install verbatim<br/>controller_params = controller.params]
    B -- ControllerParams --> C[self.controller_params = instance<br/>ctrl_type = None]
    B -- str / dict / None --> D[parse_config →<br/>ctrl_type, ctrl_cfg]
    D --> E[ControllerParams.from_dict(ctrl_cfg)<br/>→ self.controller_params]
    P --> J[self.set_controller(...)]
    C --> G{ctrl_type set?}
    E --> G
    G -- no --> H[_make_default_controller()<br/>→ Omni/Diff from motion_mode]
    G -- yes --> I[create_controller(ctrl_type, ctrl_cfg)<br/>warn if motion_mode mismatches]
    H --> J
    I --> J
```

### Step 1 — Parse `controller`

```python
if isinstance(controller, Controller):
    _preset_controller = controller
    self.controller_params = getattr(controller, "params", None) or ControllerParams()
    ctrl_type, ctrl_cfg = None, {}
elif isinstance(controller, ControllerParams):
    self.controller_params = controller
    ctrl_type, ctrl_cfg = None, {}
else:
    ctrl_type, ctrl_cfg = ControllerParams.parse_config(controller)
    self.controller_params = ControllerParams.from_dict(ctrl_cfg)
```

* `ControllerParams.parse_config` normalises `None` / `str` / `dict`
  to `(ctrl_type, raw_dict)`.
* `ControllerParams.from_dict` pulls only the keys that match its
  `@dataclass` fields and **silently drops unknown keys** so that
  impl extras (e.g. `wheel_separation`) can coexist in one dict.
* The full `ctrl_cfg` (including the dropped keys) is kept around for
  step 3; impl extras will be routed by `Controller.from_config`
  using `inspect.signature` on the subclass `__init__`.

### Step 2 — Build the controller and install via `set_controller`

`Agent.__init__` decides in one place:

```python
if _preset_controller is not None:
    # form (4): a Controller instance was passed — install it as-is.
    self.set_controller(_preset_controller)
elif ctrl_type:
    # form (2) / (3): explicit registry lookup; warn on motion_mode mismatch.
    self.set_controller(create_controller(ctrl_type, ctrl_cfg))
else:
    # form (1) / (5): fall back to motion_mode → Omni/Diff with self.controller_params.
    self.set_controller(self._make_default_controller())
```

`set_controller` is the **single** entry point that installs a
controller on the agent.  Use it at runtime to swap controllers:

```python
from pybullet_fleet.controller import create_controller

agent.set_controller(create_controller("patrol", {"max_linear_vel": 1.5}))
```

`create_controller(name, config)` → `Controller.from_config(config)`
routes the dict into:

* `params=` → `ControllerParams.from_dict(config)` (keys that match
  `ControllerParams` fields)
* `**impl_kwargs` → controller subclass `__init__` (e.g.
  `wheel_separation=0.3` for `DifferentialController`)

### `set_motion_mode` (legacy)

```python
agent.set_motion_mode(MotionMode.DIFFERENTIAL)
```

Replaces the active controller with the matching default Omni / Diff
(via the same `_make_default_controller` helper).  **Deprecated** for
new code — prefer `agent.set_controller(create_controller(...))`.

## `motion_mode` vs `controller.type`

`motion_mode` is **deprecated as a controller selector** — prefer
`controller="omni"` / `controller="differential"` or a full
`controller={"type": ..., ...}` dict.  It is still accepted (and kept
in sync with the chosen controller) because batched controllers and a
few utilities read it as a coarse "2D mobile base shape" hint.

When both are given, the rule is:

| Given | Selected controller | Warning? |
|-------|--------------------|----------|
| `motion_mode=OMNI`, `controller=None` | `OmniController` | no |
| `motion_mode=DIFF`, `controller=None` | `DifferentialController` | no |
| `motion_mode=OMNI`, `controller={"type": "omni", ...}` | `OmniController` (built with cfg) | no |
| `motion_mode=OMNI`, `controller={"type": "differential", ...}` | `DifferentialController` (override) | **yes** — logged at WARNING |
| `motion_mode=OMNI`, `controller={"type": "patrol", ...}` | `PatrolController` (override) | no (non-omni/diff) |

> **Recommendation:** set `controller.type` explicitly and **omit**
> `motion_mode` (or set it to match) to avoid the warning.

## `ControllerParams` — shared parameters for every controller

`ControllerParams` is the dataclass of parameters that every
`KinematicController` subclass shares.  It bundles both **kinematic
limits** (velocity / acceleration caps) and **behaviour flags**
(2D navigation, default movement direction, command-velocity watchdog).
Impl-specific knobs (e.g. `wheel_separation` for differential drive)
live on the controller subclass itself — not here.

`Agent.controller_params` holds the agent's instance.  The legacy
`Agent.max_linear_vel` / `max_angular_vel` / `max_linear_accel` /
`max_angular_accel` are read-only properties that delegate to it.

```python
agent.controller_params.max_linear_vel = [2.0, 1.0, 0.0]   # write here
agent.max_linear_vel                                        # → np.array([2.0, 1.0, 0.0])  (read-only)
```

Fields:

| Field | Type | Notes |
|-------|------|-------|
| `max_linear_vel` (m/s) | scalar or `[vx, vy, vz]` | Scalar = Euclidean magnitude clamp.  3-element = per-axis cap in body frame. |
| `max_angular_vel` (rad/s) | scalar or `[wx, wy, wz]` | ROS/PyBullet order; differential drive reads `wz` (idx 2). |
| `max_linear_accel` (m/s²) | scalar or per-axis | Used by TPI. |
| `max_angular_accel` (rad/s²) | scalar or per-axis | Used by TPI. |
| `cmd_vel_timeout` (s) | float | Velocity-command watchdog; `0.0` disables. |
| `navigation_2d` | bool | Preserve current `z` during pose-mode trajectories. |
| `default_direction` | `MovementDirection` | Default direction for `set_path` when none is given; differential-drive only.  `from_dict` accepts the string form. |

The batched controllers (`BatchOmniController`,
`BatchDifferentialController`) read the same instance when an agent is
registered (via `AgentManager.add_object`) and pack the fields into
`(N,)` arrays — no duplication between per-agent and batch paths.

## Passing the same `controller` value at every layer

Each layer just forwards `controller=` to the next — only
`Agent.__init__` parses it.

```text
YAML (controller: {...})
  └── AgentSpawnParams.from_dict     ← stores raw value in spawn_params.controller
        └── Agent.from_params         ← extra_kwargs={"controller": spawn_params.controller}
              └── Agent.from_urdf / from_mesh  ← controller=controller (factory kwarg)
                    └── Agent.__init__          ← ★ parses & applies
```

Direct Python calls skip the upper layers but use the same kwarg:

```python
agent = Agent.from_urdf("robot.urdf", controller={"type": "omni", "max_linear_vel": 2.5})
agent = Agent.from_mesh(visual_shape=..., controller="differential")
agent = Agent.from_urdf("robot.urdf", controller=ControllerParams(max_linear_vel=3.0))
agent = Agent.from_urdf(
    "robot.urdf",
    controller=DifferentialController(params=ControllerParams(max_linear_vel=1.5), wheel_separation=0.3),
)
```

## Batch Controllers

Batch controllers replace per-agent Python dispatch with a single vectorised
`batch_advance()` call that drives **all registered agents at once** using
NumPy arrays.  The public API (`agent.set_path()`, `MoveAction`, `agent.stop()`)
is identical to the per-agent path — the controller switch is transparent to
application code.

### When to use

| Scenario | Recommendation |
|---|---|
| < 50 agents | per-agent is fine; overhead is negligible |
| 50–500 agents | batch gives a measurable controller-loop speedup (~1.3–4×) |
| 500+ agents | batch is strongly recommended |

The speedup is concentrated in the controller-loop phase (`agent_update` /
`phase1_update` in profiling output).  Collision detection, pose flush, and
AABB refresh are unaffected.

### Available batch types

| Registry name | Per-agent equivalent | Motion mode |
|---|---|---|
| `"batch_omni"` | `omni` | `OMNIDIRECTIONAL` |
| `"batch_differential"` | `differential` | `DIFFERENTIAL` |

### Python API

The primary way to use batch controllers is through `AgentManager`.  Pass
`batch_controller=` at construction time, or call `enable_batch()` after the fact.

#### Basic usage

```python
from pybullet_fleet.agent_manager import AgentManager

mgr = AgentManager(sim_core=sim, batch_controller="batch_differential")
agents = mgr.spawn_agents_grid(100, grid_params, spawn_params)

bc = mgr.batch_controller          # BatchDifferentialController instance
for a in agents:
    bc.set_path(a, [goal_pose])
```

#### Adding agents manually

When you spawn agents outside `AgentManager` (e.g. via `Agent.from_params`
directly), register them with `mgr.add_object()`.  The batch controller is
applied automatically:

```python
mgr = AgentManager(sim_core=sim, batch_controller="batch_differential")
agents = _spawn_grid(sim, n, MotionMode.DIFFERENTIAL)   # your own helper
for a in agents:
    mgr.add_object(a)
    mgr.batch_controller.set_path(a, wp_fn(a.get_pose()))
```

#### Enable batch after spawning

You can spawn agents first and enable the batch controller later.  All
existing agents in the manager are registered at that point:

```python
mgr = AgentManager(sim_core=sim)
agents = mgr.spawn_agents_grid(...)
mgr.enable_batch("batch_omni")   # existing agents are auto-registered
bc = mgr.batch_controller
for a in agents:
    bc.set_path(a, goal)
```

#### Disable batch

```python
mgr.disable_batch()   # agents revert to their per-agent controllers
```

### Config API

The YAML config loader also supports batch controllers through entity
declarations.  Two styles are available.

#### Option B — named manager

Declare a named manager in `managers:` with `batch_controller:`, then reference
it by name from each entity group:

```yaml
managers:
  - name: delivery_fleet
    batch_controller: batch_omni
    controller:              # ControllerParams defaults — per-agent kinematics
      max_linear_vel: 1.5
      max_linear_accel: 2.0
      navigation_2d: true

entities:
  - urdf_path: robots/simple_cube.urdf
    motion_mode: omnidirectional
    manager: delivery_fleet
    grid:
      count: 50
      spacing: [2, 2]
```

`controller:` defaults are applied at spawn time to any agent that has no explicit
`controller:` block.  Individual entities can still override:

```yaml
managers:
  - name: mixed_fleet
    batch_controller: batch_differential
    controller:
      max_linear_vel: 1.0   # default for all entities below

entities:
  - urdf_path: robots/husky.urdf
    manager: mixed_fleet
    # inherits max_linear_vel: 1.0 from fleet defaults

  - urdf_path: robots/fast_husky.urdf
    manager: mixed_fleet
    controller:
      max_linear_vel: 3.0   # overrides fleet default for this entity only
```

Retrieve the manager after loading:

```python
sim = MultiRobotSimulationCore.from_yaml("config.yaml")
mgr = sim.get_manager("delivery_fleet")
bc  = mgr.batch_controller
```

Multiple fleets with different batch types:

```yaml
managers:
  - name: omni_fleet
    batch_controller: batch_omni
  - name: diff_fleet
    batch_controller: batch_differential

entities:
  - urdf_path: robots/simple_cube.urdf
    motion_mode: omnidirectional
    manager: omni_fleet
    grid:
      count: 50
      spacing: [2, 2]

  - urdf_path: robots/husky.urdf
    motion_mode: differential
    manager: diff_fleet
    grid:
      count: 30
      spacing: [3, 3]
```

#### Option A — shorthand (anonymous manager)

Set `batch_controller:` directly on an entity group.  An anonymous
`AgentManager` is created automatically:

```yaml
entities:
  - urdf_path: robots/simple_cube.urdf
    motion_mode: differential
    batch_controller: batch_differential
    grid:
      count: 100
      spacing: [2, 2]
```

Use `sim.get_manager()` (no argument, or by index) to retrieve the
auto-created manager.

### Fleet-wide kinematic defaults

Use the `controller:` key on a named manager (or `fleet_controller=` in Python) to set
kinematic defaults for all agents in the manager:

```python
from pybullet_fleet.agent_manager import AgentManager

mgr = AgentManager(
    sim_core=sim,
    batch_controller="batch_differential",
    fleet_controller={"max_linear_vel": 1.0, "navigation_2d": True},
)
# Agents spawned without an explicit controller: block inherit these defaults.
```

Fleet defaults are applied at spawn time.  If an agent already has non-default
`ControllerParams` (i.e. an explicit `controller:` block was provided), the fleet
defaults are **not** applied — per-agent config always takes precedence.

#### `navigation_2d`

`ControllerParams.navigation_2d` is `None` by default (not explicitly set), which both
per-agent and batch controllers treat as `False`:

| Setting | Behaviour |
|---|---|
| `None` (default) | follows goal Z (3D navigation) |
| `True` | preserves current Z (2D / flat-floor navigation) |
| `False` | follows goal Z |

Set it fleet-wide via `controller:` on the manager, or per-entity via the entity's own
`controller:` block.

### Kinematic params — per-agent, not per-controller

The shared batch controller has **no velocity params of its own**.  It reads
`agent.controller_params` at `set_path()` time, so each agent can have
different speed / acceleration limits:

```python
fast_params = AgentSpawnParams(
    controller={"max_linear_vel": 5.0, "max_linear_accel": 4.0, ...},
    motion_mode=MotionMode.DIFFERENTIAL,
)
slow_params = AgentSpawnParams(
    controller={"max_linear_vel": 1.0, "max_linear_accel": 1.0, ...},
    motion_mode=MotionMode.DIFFERENTIAL,
)
# Both share one BatchDifferentialController; TPI params differ per agent.
mgr = AgentManager(sim_core=sim, batch_controller="batch_differential")
fast_agents = mgr.spawn_agents_grid(50, grid_a, fast_params)
slow_agents = mgr.spawn_agents_grid(50, grid_b, slow_params)
```

### Features — full parity with per-agent

#### Direction (differential only)

```python
agent.set_path(path, direction=MovementDirection.FORWARD)    # default
agent.set_path(path, direction=MovementDirection.BACKWARD)   # robot moves backward
agent.set_path(path, direction=MovementDirection.AUTO)       # auto-select per waypoint
                                                             # (yaw-delta > 90° → BACKWARD)
```

#### Final-orientation alignment

After the last waypoint is reached, an in-place rotation aligns the robot
to `path[-1].orientation` (default `True` for both omni and diff):

```python
agent.set_path(path, final_orientation_align=True)   # default — rotate at end
agent.set_path(path, final_orientation_align=False)  # arrive and stop, no rotation
```

#### Action system

`MoveAction`, `PickAction`, and other actions in the action queue work
identically in batch and per-agent modes.  `agent.update()` continues to
run the action queue; only `controller.compute()` is skipped (the batch
controller handles movement via `batch_advance()`).

```python
# Works transparently regardless of batch or per-agent
agent.add_action(MoveAction(path=Path(waypoints=[...]), direction=MovementDirection.BACKWARD))
agent.add_action(PickAction(target=pallet))
agent.add_action(MoveAction(path=Path(waypoints=[...])))
agent.add_action(DropAction(target=drop_zone))
```

### Performance reference

Measured on 100 differential agents running a cube-patrol path (collision
detection enabled, `physics=False`):

```
batch     total=1.68 ms/step  agent_update=0.59 ms
per_agent total=1.81 ms/step  agent_update=0.70 ms

Speedup total:           1.09×
Speedup controller loop: 1.26×
```

The controller-loop speedup increases with agent count: ~3–5× at 500 agents
with collision disabled (see `examples/scale/batch_controller_500_demo.py`).

Run `python3 examples/scale/100robots_cube_patrol_demo.py --benchmark` for a
live batch vs per-agent comparison on your hardware.

---

## Adding a custom controller

1. Subclass [`Controller`](../api/index) (or `KinematicController`) and set
   a registry name:

   ```python
   class PatrolController(KinematicController):
       _registry_name = "patrol"

       def __init__(self, params: ControllerParams, *, waypoints: list):
           super().__init__(params)
           self.waypoints = waypoints
   ```

2. Import the module once (e.g. in your sim entrypoint) so the
   `__init_subclass__` hook registers it in `CONTROLLER_REGISTRY`.

3. Use it from YAML or Python by name:

   ```yaml
   controller:
     type: patrol
     max_linear_vel: 1.0    # → ControllerParams (kinematic limit)
     waypoints: [...]       # → PatrolController.__init__ (impl extra)
   ```

`Controller.from_config` inspects the subclass signature and routes
unknown-to-`ControllerParams` keys (e.g. `waypoints`) to the subclass
`__init__` automatically.
