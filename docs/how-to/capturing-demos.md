# Capturing Demo Videos

Record demo videos for documentation and presentations.
PyBulletFleet provides three recording approaches.

## Capture Pipeline Overview

| Method | Mode | Output | Requirements |
|--------|------|--------|-------------|
| **Python API** (`start_recording`) | Headless or GUI | MP4 / GIF | PyBullet only |
| **`RECORD` env var** | `run_simulation()` auto-capture | MP4 / GIF | PyBullet only |
| `capture_demo.py` | Batch headless | MP4 / GIF | PyBullet only |
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

For advanced usage (manual start/stop, orbit camera, custom camera params), see the
{class}`~pybullet_fleet.recorder.SimulationRecorder` API reference.

## Environment Variable

When the `RECORD` environment variable is set, recording starts automatically
at the beginning of `run_simulation()` and the file is saved when the simulation
ends. No script changes are required.

```bash
# Basic usage
RECORD=output.mp4 python examples/scale/100robots_grid_demo.py

# With custom settings
RECORD=demo.gif RECORD_DURATION=6.0 RECORD_FPS=10 \
  python examples/basics/action_system_demo.py
```

| Variable | Default | Description |
|----------|---------|-------------|
| `RECORD` | — | Output path (triggers recording) |
| `RECORD_DURATION` | `4.0` | Duration in seconds |
| `RECORD_FPS` | `15` | Frame rate |
| `RECORD_WIDTH` / `RECORD_HEIGHT` | `800` / `600` | Frame resolution |
| `RECORD_TIME_BASE` | `"sim"` | `"sim"` or `"real"` |

## Batch Capture Scripts

For batch recording of multiple demos, two scripts read the shared **`scripts/demos.yaml`** configuration.

### Headless Capture — `capture_demo.py`

Uses `SimulationRecorder` to render frames internally (no window needed).
Works in CI and headless environments.

```bash
# Record all demos as MP4
python scripts/capture_demo.py

# Single demo, custom format
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

Screen capture parameters are resolved in order (later wins):

1. **`format_defaults`** from `demos.yaml` (e.g. `mp4: {fps: 30, duration: 6.0}`)
2. **CLI arguments** (`--fps`, `--duration`)
3. **Per-demo overrides** in the YAML

### Per-demo delay

Demos that load many robots need extra time to stabilize before recording.
Set `delay` per-demo in `demos.yaml`:

```yaml
demos:
  pick_drop_arm_100robots:
    script: examples/scale/pick_drop_arm_100robots_demo.py
    sim_duration: 8.0
    delay: 5          # Wait 5s after window appears before recording

  action_system:
    script: examples/basics/action_system_demo.py
    sim_duration: 8.0
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
    script: examples/path/to/demo.py      # Required
    args: ["--mode=mixed"]                 # Optional CLI args for the demo
    sim_duration: 5.0                      # Optional sim-time override
    delay: 3                               # Optional stabilize delay (seconds)
    mp4:                                   # Optional per-format overrides
      fps: 60
      duration: 10.0
```

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `script` | str | *required* | Path to demo Python script |
| `args` | list | `[]` | CLI arguments passed to the demo |
| `sim_duration` | float | from `format_defaults` | Simulation duration |
| `delay` | int | CLI `--delay` or None | Stabilize delay before recording |
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
