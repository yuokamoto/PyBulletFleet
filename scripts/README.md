# scripts/

Utility scripts for capturing demo videos, model catalogs, and other media assets.

## Quick Reference

| Script | Purpose | Requirements |
|--------|---------|-------------|
| `capture_demo.py` | Headless video capture via `SimulationRecorder` | PyBullet only |
| `capture_screen_demo.py` | GUI screen recording (YAML-driven) | X11 + ffmpeg + xdotool |
| `screen_capture.sh` | Low-level X11 screen recorder | ffmpeg + xdotool (or kazam) |
| `capture_model_catalog.py` | Generate model catalog images | PyBullet only |
| `demos.yaml` | Shared demo configuration | — |

## `screen_capture.sh`

Records a PyBullet GUI window by name using ffmpeg (or kazam).

```bash
# Basic usage
bash scripts/screen_capture.sh -d 10 -o output.mp4 -- python examples/scale/100robots_grid_demo.py

# With stabilize delay and custom FPS
bash scripts/screen_capture.sh -d 10 -f 60 --delay 5 -o output.mp4 -- python demo.py

# Using kazam backend
bash scripts/screen_capture.sh --kazam -d 10 -o output.mp4 -- python demo.py
```

### Options

| Flag | Default | Description |
|------|---------|-------------|
| `-d SECS` | 10 | Recording duration |
| `-f FPS` | 30 | Capture framerate |
| `-o FILE` | `output.mp4` | Output file path |
| `--delay SECS` | 2 | Wait after window appears before recording |
| `--wait SECS` | 30 | Window detection timeout |
| `--kazam` | off | Use kazam instead of ffmpeg |
| `--` | — | End of options (required before demo script) |

**Important:** Always use `--` before the demo script path to prevent
demo arguments (e.g. `--mode=mixed`) from being interpreted as
`screen_capture.sh` options.

### Dependencies

```bash
# Ubuntu/Debian
sudo apt-get install ffmpeg xdotool

# Optional: kazam (alternative recorder)
sudo apt-get install kazam
```

## `capture_screen_demo.py`

YAML-driven orchestrator that runs `screen_capture.sh` for each demo
defined in `demos.yaml`.

```bash
python scripts/capture_screen_demo.py              # Record all demos
python scripts/capture_screen_demo.py --list        # List available demos
python scripts/capture_screen_demo.py --demo NAME   # Record one demo
python scripts/capture_screen_demo.py --dry-run     # Show commands only
```

See [Capturing Demos](../docs/how-to/capturing-demos.md) for the full
parameter resolution and `demos.yaml` format reference.

## `capture_demo.py`

Headless capture using `SimulationRecorder` — no GUI window needed.

```bash
python scripts/capture_demo.py                      # All demos, MP4
python scripts/capture_demo.py --format gif          # All demos, GIF
python scripts/capture_demo.py --demo 100robots_grid_mixed
```
