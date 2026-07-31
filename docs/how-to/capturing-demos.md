# Capturing Demo Videos

Record demo videos for documentation and presentations.
PyBulletFleet provides four recording approaches.

## Capture Pipeline Overview

| Method | Mode | Output | Requirements |
|--------|------|--------|-------------|
| **Python API** (`start_recording`) | Headless or GUI | MP4 / GIF | Pillow (GIF); `imageio[pyav]` (MP4) |
| **`RECORD` env var** | `run_simulation()` auto-capture | MP4 / GIF | Pillow (GIF); `imageio[pyav]` (MP4) |
| `capture_demo.py` | Batch headless | MP4 / GIF | Pillow (GIF); `imageio[pyav]` (MP4) |
| `capture_screen_demo.py` | GUI screen recording | MP4 | X11 + ffmpeg + xdotool |

## Python API

Call `start_recording()` on the simulation core to record from within your script:

```python
from pybullet_fleet import MultiRobotSimulationCore

sim = MultiRobotSimulationCore.from_yaml("config/config.yaml")
# ... spawn agents ...

# Record 4 seconds as MP4 (auto-saved when run_simulation ends)
sim.start_recording("output.mp4", duration=4.0, fps=30)
sim.run_simulation(duration=5.0)
```

Key parameters:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `output` | `"recording.gif"` | File path (`.gif` or `.mp4`) |
| `duration` | `None` | Recording length in seconds (`None` = manual stop) |
| `fps` | `15` | Capture & playback frame rate |
| `width` / `height` | `800` / `600` | Frame resolution |
| `camera_mode` | `"auto"` | `"auto"` \| `"gui"` \| `"orbit"` \| `"manual"` |
| `time_base` | `"sim"` | `"sim"` (1× playback) or `"real"` (wall-clock speed) |

`camera_mode="auto"` selects the GUI camera when `SimulationParams.gui=True`;
otherwise it frames the scene from an automatically computed camera position.
`camera_mode="gui"` explicitly requires a GUI connection. Use `"orbit"` or
`"manual"` for a moving or fully specified headless camera.

For advanced usage (manual start/stop, orbit camera, custom camera params), see the
{class}`~pybullet_fleet.recorder.SimulationRecorder` API reference.

## Environment Variable

When the `RECORD` environment variable is set, recording starts automatically
at the beginning of `run_simulation()` and the file is saved when the simulation
ends. No script changes are required.

```bash
# Basic usage
RECORD=output.mp4 python pybullet_fleet/examples/scale/100robots_grid_demo.py

# With custom settings
RECORD=demo.gif RECORD_DURATION=6.0 RECORD_FPS=10 \
  python pybullet_fleet/examples/basics/action_system_demo.py
```

| Variable | Default | Description |
|----------|---------|-------------|
| `RECORD` | — | Output path (triggers recording) |
| `RECORD_DURATION` | `4.0` | Duration in seconds |
| `RECORD_FPS` | `15` | Frame rate |
| `RECORD_WIDTH` / `RECORD_HEIGHT` | `800` / `600` | Frame resolution |
| `RECORD_TIME_BASE` | `"sim"` | `"sim"` or `"real"` |
| `RECORD_GUI` | unset | Set to `1` to keep a requested GUI connection and use its hardware OpenGL camera |

For reproducible headless capture, `RECORD` forces `gui=False` when the
simulation was configured with a GUI. Set `RECORD_GUI=1` when you intentionally
want to keep the GUI window and capture from its camera.

## Batch Capture Scripts

For batch recording of multiple demos, two scripts read the shared **`scripts/demos.yaml`** configuration.

### Headless Capture — `capture_demo.py`

Uses `SimulationRecorder` to render frames internally (no window needed).
Works in CI and headless environments.

```bash
# Record all demos as GIF (the default)
python scripts/capture_demo.py

# Record all demos as MP4
python scripts/capture_demo.py --format mp4

# Single demo
python scripts/capture_demo.py --demo 100robots_grid_mixed --format gif
```

### GUI Screen Capture — `capture_screen_demo.py`

Records the actual PyBullet GUI window including the DataMonitor overlay.
Requires a running X11 display.

```bash
# Record all demos
python scripts/capture_screen_demo.py

# Single demo with custom delay
python scripts/capture_screen_demo.py --demo 100robots_grid_mixed --delay 3

# Dry run (show commands without executing)
python scripts/capture_screen_demo.py --dry-run

# List available demos
python scripts/capture_screen_demo.py --list
```

### Parameter resolution (3-layer merge)

For `capture_demo.py`, recording parameters are resolved in order (later wins):

1. **`format_defaults`** from `demos.yaml` (e.g. `mp4: {fps: 30, duration: 6.0}`)
2. **CLI arguments** (`--fps`, `--duration`)
3. **Per-demo overrides** in the YAML

`capture_screen_demo.py` uses the MP4 defaults and accepts only `--fps` and
`--duration` as recording overrides. Its per-demo `recording_duration`, when
present, takes precedence over both the MP4 default and CLI `--duration`.

### Per-demo delay

Demos that load many robots need extra time to stabilize before recording.
Set `delay` per-demo in `demos.yaml`:

```yaml
demos:
  pick_drop_arm_100robots:
    script: pybullet_fleet/examples/scale/pick_drop_arm_100robots_demo.py
    delay: 5          # Wait 5s after window appears before recording

  action_system:
    script: pybullet_fleet/examples/basics/action_system_demo.py
    # No delay — starts recording immediately
```

The CLI `--delay` flag serves as a global fallback for demos without a
per-demo `delay` setting.

### `demos.yaml` Format

```yaml
format_defaults:
  gif:
    fps: 10
    width: 640
    height: 480
    duration: 4.0
    time_base: sim
  mp4:
    fps: 30
    width: 1280
    height: 960
    duration: 6.0
    time_base: sim

demos:
  demo_name:
    script: pybullet_fleet/examples/path/to/demo.py  # Required
    args: ["--mode=mixed"]                 # Optional CLI args for the demo
    delay: 3                               # Optional stabilize delay (seconds)
    recording_duration: 8.0                # Screen-capture duration; takes priority
    mp4:                                   # Optional per-format overrides
      fps: 60
      duration: 10.0
```

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `script` | str | *required* | Path to demo Python script |
| `args` | list | `[]` | CLI arguments passed to the demo |
| `delay` | int | CLI `--delay` or None | Stabilize delay before recording |
| `recording_duration` | float | MP4 `duration` | Screen-capture duration in wall-clock seconds; overrides `--duration` |
| `mp4` / `gif` | dict | `{}` | Per-format parameter overrides |

### Output

Recordings are saved to `docs/media/` by default:

```
docs/media/
  100robots_grid_mixed.mp4
  100robots_cube_patrol.mp4
  pick_drop_arm_100robots.mp4
  ...
```

For shell-level usage of `screen_capture.sh`, see `scripts/README.md`.
