# GUI Interaction MVP

**Date:** 2026-08-16
**Status:** Draft
**Related:** [Camera Control GUI](../2026-04-18-camera-control/spec.md),
[EventBus](../../architecture/plugins-events.md), [Roadmap](../../roadmap.md)

## Goal

Make a PyBulletFleet GUI run inspectable and controllable enough to develop and
demonstrate a warehouse scenario without creating a second simulation API or
adding work to headless runs.  The PyBullet window remains the 3D viewport;
the optional `DataMonitor` window is the richer inspector and control surface.
This is an interim layer for `p.GUI`, not a replacement for a future Rerun/RViz
visualizer.

The MVP answers four questions quickly:

1. Is the simulation playing, paused, or advancing one step at a time?
2. Which named entity is selected, and what is its current pose and activity?
3. Can the camera focus on that entity while it moves?
4. Which keyboard controls are available in this scene?

## Decision

Add a GUI-only `GuiController`, owned by `MultiRobotSimulationCore` and
created only after `configure_visualizer()` succeeds.  It composes the existing
`CameraController`; it does not replace PyBullet's native camera interaction.
It owns viewport-only state: keyboard shortcuts, selection highlight, camera
follow, and a small shortcut overlay.

Extend the separate `DataMonitor` window with an inspector/control adapter. It
renders monitor frames and turns button/menu input into toolkit-neutral
`GuiCommand` requests.  Neither tkinter nor a future replacement UI calls
PyBullet or mutates `MultiRobotSimulationCore` directly.  The simulation
thread consumes requests at the start of `step_once()`, including while paused.

The core remains the authority for simulation state: the command consumer calls
public core methods such as `pause()` and `resume()` and uses a single-step
request boundary rather than mutating agent, action, or collision internals.

```
PyBullet keyboard              DataMonitor window (tkinter)
       |                                  |
       v                                  v
GuiController                    GuiCommand queue
(viewport-only)                         |
       |                                  |
       +-----------+  +------------------+
                   v  v
          MultiRobotSimulationCore (simulation thread)
                   |
                   +-- MonitorFrame --> DataMonitor renderer
                   +-- CameraController: focus / follow / highlight
                   +-- EventBus events
```

The existing `DataMonitor.write_data()` JSON file remains a compatibility
output in this MVP.  The new in-process frame/command boundary becomes the
preferred path for the built-in tkinter UI; a future external UI can use a
different adapter without changing simulation semantics.  The initial
integration may live next to the existing visualizer methods in
`core_simulation.py`.  Extracting it to `VisualizerController` is deliberately
out of scope, so this feature does not become a prerequisite for the broader
core decomposition.

## Configuration

All GUI controls are opt-in through a new `SimulationParams.gui_controls`
mapping.  It is ignored when `gui=False` and defaults to the following when
`gui=True`:

```yaml
simulation:
  gui: true
  gui_controls:
    enabled: true
    update_rate: 10            # Hz; valid range 1..30
    show_control_panel: true
    show_selection_panel: true
    keyboard_shortcuts: true
    initial_follow: false
```

No viewport GUI object, timer, event polling, or selection state is created
when `gui=False` or `gui_controls.enabled=false`.  The monitor's existing
`monitor` and `enable_monitor_gui` options independently control the separate
DataMonitor window.  `p.DIRECT` remains the only test backend; controller logic
is unit-tested by mocking its PyBullet adapter and core command boundary.

## MVP behaviour

### Playback

- `SPACE` remains pause/resume.
- `.` requests exactly one simulation step while paused.  The request is
  consumed by `step_once()` after GUI input has been processed; it must not
  re-enter `step_once()` from an input callback.
- The DataMonitor window provides pause/resume and single-step controls.  Its
  callbacks enqueue `GuiCommand` values; they never run a simulation step.
- A speed multiplier is intentionally deferred.  The current real-time pacing
  behaviour and its relationship to ROS clock require a separate contract.

### Selection and inspection

- `[` and `]` cycle deterministically through registered visible entities,
  ordered by stable entity ID.  This is the required selection mechanism for
  the MVP.
- A later click-to-select implementation may use a camera ray and `rayTest`,
  but is not needed to establish selection semantics and must not consume
  native left-drag camera controls.
- The selected entity is highlighted with a PyBullet debug outline/AABB and
  label.  Highlights are transient visual aids and never alter visual-shape
  colours, collision modes, or entity state.
- The DataMonitor inspector shows stable ID, display name, kind (agent/object),
  pose, current action type/status where applicable, and whether follow is
  enabled.  Missing or removed selections are cleared safely at the next
  update.

### Camera and help

- `f` toggles camera follow for the selected entity.  While enabled, the
  controller updates only the camera target; yaw, pitch, and distance remain
  user-controlled.
- Existing right-drag pan, `=`/`-` zoom, and `o` top-down controls remain
  unchanged.
- `?` toggles a throttled viewport shortcut overlay.  The DataMonitor may also
  show the same help text.  Neither attempts to list PyBullet's built-in
  shortcuts.

## Event contract

The controller emits custom events on `sim.events` only after the operation
has been accepted by the core.  These names are reserved for the GUI surface:

| Event | Required fields | Meaning |
|---|---|---|
| `gui.selection_changed` | `entity_id`, `source` | The selected entity changed or was cleared (`entity_id=None`). |
| `gui.follow_changed` | `entity_id`, `enabled` | Follow was enabled or disabled. |
| `gui.single_step_requested` | `source` | A paused single step was requested. |
| `gui.help_toggled` | `visible` | The shortcut overlay changed visibility. |

Pause and resume continue to emit `SimEvents.PAUSED` and `SimEvents.RESUMED`.
The MVP does **not** introduce `gui.manipulate` or `gui.action_cancel`; those
events require authorization, failure, and persistence semantics that are not
yet defined.

`source` is `keyboard` or `monitor` for MVP inputs. Event handlers must remain
optional.  GUI rendering and input processing must not construct snapshots or
perform serialization merely because no handler is registered.

## Performance and lifecycle constraints

- Viewport overlay text, labels, highlights, camera-follow updates, and
  DataMonitor frame publication are limited to `update_rate` (default 10 Hz,
  hard maximum 30 Hz), independent of simulation FPS.
- Input polling still occurs once per GUI `step_once()` so key presses are not
  missed.  It performs only constant-time checks when no relevant key changes.
- Follow reads only the selected entity pose.  It must not scan all entities
  after selection has been resolved.
- Viewport debug item IDs are owned by `GuiController` and removed/replaced safely on
  shutdown, reset, selection change, and PyBullet disconnect.
- The command queue is bounded and coalesces state-setting requests such as
  selection and follow.  A closed or slow monitor window must not block the
  simulation thread.
- GUI exceptions and a closed GUI connection are logged and disable further
  GUI updates; they must not terminate a headless or running simulation.

## Public API boundary

The first release exposes only read-only state suitable for an optional future
external GUI:

```python
@property
def selected_entity_id(self) -> str | None: ...

@property
def gui_follow_enabled(self) -> bool: ...
```

Programmatic selection/follow commands are deferred until their intended use
by plugins, ROS tooling, or a replacement visualizer is established.  The
public `SimulationParams` mapping and EventBus event payloads are therefore
the compatibility commitments of this MVP.

## Out of scope

- Object deletion and any other mutation from GUI. Paused-only pose editing is
  supported through the monitor's XYZ/yaw fields and Ctrl+left-drag in the
  viewport; it clears velocity and does not cancel or alter queued actions.
- Action queue cancellation/editing and an action timeline.
- Floor visibility controls and multi-floor navigation.
- Mouse click picking, a scene tree, search/filter, and persistent layouts.
- A tkinter/web/Rerun panel, ROS remote control, or GUI state persistence.
- A PyBullet debug-parameter control panel; DataMonitor is the control surface.
- Snapshot/Replay, event serialization, or the `MultiRobotSimulationCore`
  decomposition.

## Implementation phases

1. **State/command boundary and tests** — add `GuiCommand`, a bounded command
   queue, a read-only `MonitorFrame`, GUI-only controller creation, a
   non-reentrant one-step request, and direct-mode unit tests for
   defaults/no-op behaviour.
2. **Playback and help** — add DataMonitor playback controls, implement
   `SPACE`/`.` keyboard parity and the viewport shortcut overlay while
   preserving existing camera keys.
3. **Selection and follow** — deterministic cycling, monitor inspector,
   safe selection cleanup, viewport highlight, follow camera target, and
   EventBus events.
4. **Warehouse validation** — manually validate with a large GUI scene and
   add a documented keyboard-control example.  The example must also run
   headless without `gui_controls` work.

## Acceptance criteria

- `gui=False` creates no `GuiController` and does not poll GUI APIs.
- Existing camera controls, transparency toggle, pause/resume, and `p.DIRECT`
  tests remain compatible.
- A paused `.` input advances exactly one simulation step and returns paused.
- A DataMonitor single-step click has the same request/consume behaviour as
  the `.` shortcut and never invokes `step_once()` from the tkinter thread.
- Selection remains valid across spawn/remove; removing the selected entity
  clears the highlight and emits one selection-change event.
- Follow tracks the selected entity without overwriting yaw, pitch, or zoom.
- UI redraw work is capped at 30 Hz and is absent from headless profiler
  output.
- GUI input emits the documented events once per accepted state transition.

## Follow-up decisions

The warehouse example will decide whether click picking, action inspection,
floor controls, or safe object manipulation provides the highest next value.
Those additions should be designed separately rather than expanding this MVP
while its public event and configuration boundaries are still new.
