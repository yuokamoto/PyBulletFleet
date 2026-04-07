# SimulationRecorder — Agent Technical Specification

## Requirements

### Functional
- `SimulationRecorder` class in `pybullet_fleet/recorder.py`
- `start_recording()` / `stop_recording()` methods on `MultiRobotSimulationCore`
- Environment variable `RECORD=<path>` triggers automatic recording in `run_simulation()`
- Camera modes: `auto` (AABB framing), `orbit` (auto + rotation), `manual` (user-specified)
- Output formats: `.gif` (default), `.mp4` (if ffmpeg available)
- Auto-stop after configurable `duration` seconds

### Non-Functional
- GIF output < 5 MB at default settings (800×600, 15fps, 4s = 60 frames)
- Frame capture overhead < 5ms per frame at 800×600 (ER_TINY_RENDERER)
- No new runtime dependencies (imageio already in requirements-dev.txt)

## Constraints

- Must use `p.getCameraImage()` + `p.ER_TINY_RENDERER` for headless compatibility
- Must use `p.computeViewMatrix()` / `p.computeProjectionMatrixFOV()` (not debugVisualizer)
- Must follow `DataMonitor` integration pattern (optional component, gated, daemon-safe)
- Must use `get_lazy_logger(__name__)` for logging
- Must use `TYPE_CHECKING` guard for circular import prevention
- Never call `p.stepSimulation()` — recorder is observation-only

## Approach

### Architecture

```
MultiRobotSimulationCore
    │  ._recorder: Optional[SimulationRecorder]
    │  start_recording() → creates & starts recorder
    │  stop_recording()  → stops & saves, returns path
    │  run_simulation()  → checks RECORD env var at start
    │                       auto-stops recorder at end
    ▼
SimulationRecorder (pybullet_fleet/recorder.py)
    ├── __init__(sim_core, output, width, height, fps, duration, camera_mode, ...)
    ├── start()   → registers callback on sim_core
    ├── stop()    → unregisters callback
    ├── save()    → writes frames to GIF/MP4, returns path
    ├── _capture_frame(sim_core, dt)  → callback function
    └── _compute_view_matrix()        → camera positioning
```

### Key Components

| Component | Responsibility | Location |
|-----------|---------------|----------|
| `SimulationRecorder` | Frame capture, camera control, GIF/MP4 output | `pybullet_fleet/recorder.py` |
| `start_recording()` | Convenience API on sim core | `pybullet_fleet/core_simulation.py` |
| `stop_recording()` | Stop + save convenience | `pybullet_fleet/core_simulation.py` |
| `RECORD` env var check | Auto-trigger in `run_simulation()` | `pybullet_fleet/core_simulation.py` |

### Data Flow

```
run_simulation() start
    │
    ├── if os.environ.get("RECORD"): start_recording(path)
    │
    ▼ simulation loop
    step_once()
    │ → callbacks executed
    │   → SimulationRecorder._capture_frame()
    │     → p.computeViewMatrix() based on camera_mode
    │     → p.getCameraImage(w, h, view, proj, renderer=ER_TINY_RENDERER)
    │     → np.array(rgba).reshape(h, w, 4)[:, :, :3]
    │     → append to self._frames list
    │     → if duration exceeded: self.stop()
    │
    ▼ simulation loop end
    if self._recorder: stop_recording()
```

### Camera Modes

| Mode | View Matrix Computation |
|------|------------------------|
| `auto` | Compute AABB of all sim_objects → center + extent → fixed overhead camera |
| `orbit` | Same as auto but `yaw += orbit_speed * elapsed_time` |
| `manual` | User provides `camera_distance`, `camera_yaw`, `camera_pitch`, `camera_target` |

**Auto camera logic** (reuse setup_camera pattern):
```python
def _compute_auto_camera(self) -> Tuple[List[float], List[float]]:
    """Compute eye position and target from entity AABBs."""
    all_positions = []
    for obj in self._sim_core.sim_objects:
        pos, _ = p.getBasePositionAndOrientation(obj.body_id, physicsClientId=client)
        all_positions.append(pos)

    if not all_positions:
        return [0, 0, 10], [0, 0, 0]  # fallback

    positions = np.array(all_positions)
    center = positions.mean(axis=0)
    extent = positions.max(axis=0) - positions.min(axis=0)
    max_extent = max(extent[0], extent[1], 1.0)

    # Camera height proportional to scene extent
    eye = [center[0], center[1], max_extent * 0.8]
    target = [center[0], center[1], 0.0]
    return eye.tolist(), target.tolist()
```

### SimulationRecorder Class

```python
class SimulationRecorder:
    def __init__(
        self,
        sim_core: "MultiRobotSimulationCore",
        output: str = "recording.gif",
        width: int = 800,
        height: int = 600,
        fps: int = 15,
        duration: Optional[float] = None,
        camera_mode: str = "auto",         # "auto" | "orbit" | "manual"
        camera_params: Optional[Dict[str, Any]] = None,
        orbit_degrees: float = 60.0,
    ):
        self._sim_core = sim_core
        self._output = output
        self._width = width
        self._height = height
        self._fps = fps
        self._duration = duration
        self._camera_mode = camera_mode
        self._camera_params = camera_params or {}
        self._orbit_degrees = orbit_degrees
        self._frames: List[np.ndarray] = []
        self._recording = False
        self._start_sim_time: float = 0.0
        self._proj_matrix = p.computeProjectionMatrixFOV(
            fov=50, aspect=width / height, nearVal=0.1, farVal=200
        )

    def start(self) -> None:
        """Start recording. Registers capture callback on sim_core."""
        self._recording = True
        self._start_sim_time = self._sim_core.sim_time
        self._frames.clear()
        self._sim_core.register_callback(self._capture_frame, frequency=float(self._fps))

    def stop(self) -> None:
        """Stop recording. Unregisters callback."""
        self._recording = False
        # Remove callback from sim_core._callbacks

    def save(self) -> str:
        """Write frames to output file. Returns output path."""
        if not self._frames:
            raise ValueError("No frames captured")
        import imageio
        imageio.mimsave(self._output, self._frames, fps=self._fps, loop=0)
        return self._output

    def _capture_frame(self, sim_core, dt) -> None:
        """Callback: capture one frame."""
        if not self._recording:
            return
        # Check duration
        if self._duration and (sim_core.sim_time - self._start_sim_time) >= self._duration:
            self.stop()
            return
        view_matrix = self._compute_view_matrix()
        _, _, rgba, _, _ = p.getCameraImage(
            self._width, self._height,
            viewMatrix=view_matrix,
            projectionMatrix=self._proj_matrix,
            renderer=p.ER_TINY_RENDERER,
            physicsClientId=sim_core.client,
        )
        rgb = np.array(rgba, dtype=np.uint8).reshape(self._height, self._width, 4)[:, :, :3]
        self._frames.append(rgb)

    @property
    def is_recording(self) -> bool:
        return self._recording

    @property
    def frame_count(self) -> int:
        return len(self._frames)
```

### Core Simulation Integration

```python
# Added to MultiRobotSimulationCore

def start_recording(
    self,
    output: str = "recording.gif",
    width: int = 800,
    height: int = 600,
    fps: int = 15,
    duration: Optional[float] = None,
    camera_mode: str = "auto",
    **kwargs,
) -> "SimulationRecorder":
    from pybullet_fleet.recorder import SimulationRecorder
    if self._recorder is not None:
        self.stop_recording()  # stop existing
    self._recorder = SimulationRecorder(
        sim_core=self, output=output, width=width, height=height,
        fps=fps, duration=duration, camera_mode=camera_mode, **kwargs,
    )
    self._recorder.start()
    logger.info("Recording started: %s", output)
    return self._recorder

def stop_recording(self) -> Optional[str]:
    if self._recorder is None:
        return None
    self._recorder.stop()
    path = self._recorder.save()
    logger.info("Recording saved: %s (%d frames)", path, self._recorder.frame_count)
    self._recorder = None
    return path
```

**`run_simulation()` env var check** (added at start of method):
```python
def run_simulation(self, duration=None):
    # Auto-recording from RECORD env var
    record_path = os.environ.get("RECORD")
    if record_path and self._recorder is None:
        record_duration = float(os.environ.get("RECORD_DURATION", "4.0"))
        record_fps = int(os.environ.get("RECORD_FPS", "15"))
        self.start_recording(output=record_path, duration=record_duration, fps=record_fps)

    # ... existing run_simulation logic ...

    # Auto-save at end
    if self._recorder is not None:
        self.stop_recording()
```

### Callback Unregistration

Currently `MultiRobotSimulationCore` has no `unregister_callback()`. We need to add one:

```python
def unregister_callback(self, callback_func: Callable) -> bool:
    """Remove a registered callback by function reference.

    Returns True if callback was found and removed.
    """
    for i, cbinfo in enumerate(self._callbacks):
        if cbinfo["func"] is callback_func:
            self._callbacks.pop(i)
            return True
    return False
```

### __init__.py Export

Add `SimulationRecorder` to `pybullet_fleet/__init__.py`:
```python
from pybullet_fleet.recorder import SimulationRecorder
```

### capture_demo.py Refactor

Replace the current standalone spawning script with a thin wrapper:
```python
#!/usr/bin/env python3
"""Capture demo GIFs by running actual demo scripts with RECORD env var."""
import os
import subprocess
import sys

DEMOS = {
    "100robots_grid_mixed": ("examples/scale/100robots_grid_demo.py", ["--mode=mixed"]),
    "100robots_grid_single": ("examples/scale/100robots_grid_demo.py", ["--mode=single"]),
    "100robots_grid_arm": ("examples/scale/100robots_grid_demo.py", ["--mode=mixed", "--arm-robot=arm_robot"]),
    # ... more demos
}

for name, (script, extra_args) in DEMOS.items():
    output = f"docs/media/{name}.gif"
    env = {**os.environ, "RECORD": output, "RECORD_DURATION": "4.0"}
    cmd = [sys.executable, script] + extra_args
    print(f"Capturing {name}...")
    subprocess.run(cmd, env=env, check=True)
    print(f"  → {output} ({os.path.getsize(output) / 1024 / 1024:.1f} MB)")
```

## File References

Files the plan agent MUST read before planning:
- `pybullet_fleet/core_simulation.py` — `run_simulation()` (L2077-2180), `register_callback()` (L336-356), `step_once()` (L2241-2310), callback execution loop (L2331-2340), `setup_camera()` (L1353-1445), `_client` property (L302-304)
- `pybullet_fleet/data_monitor.py` — integration pattern (optional component, start/stop lifecycle)
- `pybullet_fleet/__init__.py` — current exports
- `pybullet_fleet/sim_object.py` — `body_id` property
- `scripts/capture_demo.py` — demo capture script using `RECORD` env var
- `scripts/capture_model_catalog.py` — model thumbnail generator (unchanged)
- `examples/scale/100robots_grid_demo.py` — demo structure, argparse, callback pattern
- `tests/conftest.py` — MockSimCore, autouse fixtures

## Testing Strategy

### Unit Tests (`tests/test_recorder.py`)

1. **test_recorder_init** — Verify default parameters
2. **test_recorder_start_registers_callback** — `start()` adds callback to sim_core
3. **test_recorder_stop_unregisters_callback** — `stop()` removes callback
4. **test_recorder_capture_frame** — Single frame capture produces correct shape
5. **test_recorder_auto_stop_on_duration** — Stops after `duration` seconds
6. **test_recorder_save_gif** — Produces valid GIF file
7. **test_recorder_save_no_frames_raises** — `save()` with empty frames raises ValueError
8. **test_recorder_camera_auto** — Auto camera computes reasonable eye/target
9. **test_recorder_camera_orbit** — Orbit mode rotates camera
10. **test_recorder_camera_manual** — Manual mode uses provided params

### Integration Tests

11. **test_core_start_stop_recording** — `sim.start_recording()` / `stop_recording()` lifecycle
12. **test_core_record_env_var** — `RECORD` env var triggers recording in `run_simulation()`
13. **test_core_run_simulation_auto_saves** — Recording auto-saved when `run_simulation()` ends
14. **test_unregister_callback** — New `unregister_callback()` method works correctly

### All tests use `p.DIRECT` mode, `SimulationParams(gui=False, monitor=False)`.

## Success Criteria

- [ ] `SimulationRecorder` captures frames from actual simulation runs
- [ ] `sim.start_recording("test.gif", duration=2.0)` + `run_simulation()` produces valid GIF
- [ ] `RECORD=test.gif python examples/scale/100robots_grid_demo.py` works with zero code changes
- [ ] Works in both `p.DIRECT` and `p.GUI` modes
- [ ] `unregister_callback()` added to `MultiRobotSimulationCore`
- [x] `scripts/capture_demo.py` refactored to use `RECORD` env var
- [ ] All existing tests pass (`make verify`)
- [ ] New tests for recorder achieve > 90% coverage of `recorder.py`
