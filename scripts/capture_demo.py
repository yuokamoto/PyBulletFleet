#!/usr/bin/env python3
"""
capture_demo.py — Capture GIFs or MP4s from actual demo scripts using SimulationRecorder.

Runs demo scripts with the RECORD environment variable set, so the
``MultiRobotSimulationCore.run_simulation()`` method automatically
records and saves the output — zero code changes in each demo.

Recording parameters are resolved in three layers (later wins):

1. ``format_defaults`` from ``scripts/demos.yaml`` for the chosen format
2. CLI arguments (``--fps``, ``--width``, ``--duration``, …)
3. Per-demo format overrides in the YAML (e.g. ``mp4: {duration: 10}``)

Usage::

    # Capture all configured demos as GIF (default)
    python scripts/capture_demo.py

    # Capture as MP4 (smaller files, better quality)
    python scripts/capture_demo.py --format mp4

    # Capture a specific demo
    python scripts/capture_demo.py --demo 100robots_grid_mixed

    # Custom duration / fps / resolution (overrides format_defaults)
    python scripts/capture_demo.py --duration 6 --fps 10 --width 640 --height 480

    # Use a custom config file
    python scripts/capture_demo.py --config my_demos.yaml

    # List available demos
    python scripts/capture_demo.py --list
"""
import argparse
import os
import subprocess
import sys

import yaml

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
DEFAULT_CONFIG = os.path.join(os.path.dirname(__file__), "demos.yaml")


def load_config(path: str) -> dict:
    """Load the demo capture YAML config."""
    with open(path) as f:
        return yaml.safe_load(f)


def resolve_recording_params(
    format_defaults: dict,
    fmt: str,
    cli_args: argparse.Namespace,
    demo_overrides: dict,
) -> dict:
    """Merge recording parameters: format_defaults → CLI → per-demo override.

    Returns dict with keys: fps, width, height, duration, time_base.
    """
    # Layer 1: format defaults
    params = dict(format_defaults.get(fmt, {}))

    # Layer 2: CLI overrides (only if explicitly provided)
    cli_explicit = {k for k, v in vars(cli_args).items() if v is not None}
    if "fps" in cli_explicit:
        params["fps"] = cli_args.fps
    if "width" in cli_explicit:
        params["width"] = cli_args.width
    if "height" in cli_explicit:
        params["height"] = cli_args.height
    if "duration" in cli_explicit:
        params["duration"] = cli_args.duration
    if "time_base" in cli_explicit:
        params["time_base"] = cli_args.time_base

    # Layer 3: per-demo format overrides
    params.update(demo_overrides.get(fmt, {}))

    return params


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

parser = argparse.ArgumentParser(description="Capture demo GIFs/MP4s via SimulationRecorder")
parser.add_argument("--demo", default=None, help="Capture a single demo by name (default: all)")
parser.add_argument("--list", action="store_true", help="List available demos and exit")
parser.add_argument("--config", default=DEFAULT_CONFIG, help="Config YAML (default: scripts/demos.yaml)")
parser.add_argument("--format", default="gif", choices=["gif", "mp4"], help="Output format (default: gif)")
parser.add_argument("--duration", type=float, default=None, help="Recording duration in sim-seconds")
parser.add_argument("--fps", type=int, default=None, help="Capture/playback FPS")
parser.add_argument("--width", type=int, default=None, help="Frame width")
parser.add_argument("--height", type=int, default=None, help="Frame height")
parser.add_argument("--output-dir", default="docs/media", help="Output directory (default: docs/media)")
parser.add_argument(
    "--time-base",
    default=None,
    choices=["sim", "real"],
    help="Recording time base: 'sim' (default) or 'real' (wall-clock)",
)
parser.add_argument(
    "--gui",
    action="store_true",
    help="Keep GUI window open during recording (GPU-quality via ER_BULLET_HARDWARE_OPENGL)",
)
args = parser.parse_args()

# Load config
cfg = load_config(args.config)
format_defaults = cfg.get("format_defaults", {})
DEMOS: dict = cfg.get("demos", {})

if args.list:
    print("Available demos:")
    for name, info in sorted(DEMOS.items()):
        print(f"  {name:30s} → {info['script']}")
    sys.exit(0)

# ---------------------------------------------------------------------------
# Capture
# ---------------------------------------------------------------------------

if args.demo and args.demo not in DEMOS:
    print(f"ERROR: Unknown demo '{args.demo}'. Use --list to see available demos.")
    sys.exit(1)

demos_to_run = {args.demo: DEMOS[args.demo]} if args.demo else DEMOS

os.makedirs(args.output_dir, exist_ok=True)
results = []

for name, info in demos_to_run.items():
    output = os.path.join(args.output_dir, f"{name}.{args.format}")
    script = os.path.join(PROJECT_ROOT, info["script"])

    if not os.path.exists(script):
        print(f"  SKIP {name}: script not found ({info['script']})")
        results.append((name, "SKIP", 0))
        continue

    # Resolve recording params: format_defaults → CLI → per-demo override
    rec = resolve_recording_params(format_defaults, args.format, args, info)

    env = {
        **os.environ,
        "RECORD": output,
        "RECORD_DURATION": str(rec.get("duration", 4.0)),
        "RECORD_FPS": str(rec.get("fps", 15)),
        "RECORD_WIDTH": str(rec.get("width", 800)),
        "RECORD_HEIGHT": str(rec.get("height", 600)),
    }
    time_base = rec.get("time_base")
    if time_base:
        env["RECORD_TIME_BASE"] = str(time_base)
    if args.gui:
        env["RECORD_GUI"] = "1"

    # The demo runs until stop_recording() is called (via RECORD env var).
    # subprocess timeout ensures the process doesn't hang indefinitely.
    cmd = [sys.executable, script] + info.get("args", [])

    print(f"Capturing {name}...")
    print(f"  cmd: {' '.join(cmd)}")
    try:
        result = subprocess.run(
            cmd,
            env=env,
            cwd=PROJECT_ROOT,
            timeout=120,
            capture_output=True,
            text=True,
        )
        if os.path.exists(output):
            size_mb = os.path.getsize(output) / (1024 * 1024)
            print(f"  OK → {output} ({size_mb:.1f} MB)")
            results.append((name, "OK", size_mb))
        else:
            print(f"  FAIL: output not created (exit code {result.returncode})")
            if result.stderr:
                # Print last few lines of stderr for debugging
                for line in result.stderr.strip().split("\n")[-5:]:
                    print(f"    {line}")
            results.append((name, "FAIL", 0))
    except subprocess.TimeoutExpired:
        print("  TIMEOUT (120s)")
        results.append((name, "TIMEOUT", 0))
    except Exception as e:
        print(f"  ERROR: {e}")
        results.append((name, "ERROR", 0))

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------

print(f"\n{'='*50}")
print(f"{'Demo':30s} {'Status':8s} {'Size':>8s}")
print(f"{'-'*50}")
total_mb = 0.0
for name, status, size in results:
    size_str = f"{size:.1f} MB" if size > 0 else "-"
    print(f"{name:30s} {status:8s} {size_str:>8s}")
    total_mb += size
print(f"{'-'*50}")
print(f"{'Total':30s} {'':8s} {total_mb:.1f} MB")
ok_count = sum(1 for _, s, _ in results if s == "OK")
print(f"\n{ok_count}/{len(results)} demos captured successfully")
