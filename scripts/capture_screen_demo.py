#!/usr/bin/env python3
"""
capture_screen_demo.py — YAML-driven screen recording via screen_capture.sh.

The screen-capture counterpart of ``capture_demo.py``.  Reads ``demos.yaml``
and runs ``screen_capture.sh`` for each demo, producing real GUI recordings
(with DataMonitor FPS/RTF overlay visible).

The demo process runs at its **native speed** — no ``--rtf`` or ``--duration``
is injected.  ``screen_capture.sh`` records the GUI window for the configured
``recording_duration`` (or ``format_defaults.mp4.duration``), then kills the
demo process.  This means demo scripts must default to ``duration=None``
(run forever) so the window stays open long enough.

Recording parameters are resolved in three layers (later wins):

1. ``format_defaults`` from ``scripts/demos.yaml`` for the ``mp4`` format
2. CLI arguments (``--fps``, ``--duration``, …)
3. Per-demo overrides in the YAML (including ``recording_duration``)

Usage::

    # Record all configured demos
    python scripts/capture_screen_demo.py

    # Record a single demo
    python scripts/capture_screen_demo.py --demo 100robots_grid_mixed

    # Custom duration / fps
    python scripts/capture_screen_demo.py --duration 15 --fps 60

    # Use kazam instead of ffmpeg
    python scripts/capture_screen_demo.py --recorder kazam

    # List available demos
    python scripts/capture_screen_demo.py --list

Requirements:
    Same as screen_capture.sh — ffmpeg + xdotool (or kazam).
    Requires a running X11 display (not headless).
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
SCREEN_CAPTURE_SH = os.path.join(os.path.dirname(__file__), "screen_capture.sh")
DEFAULT_CONFIG = os.path.join(os.path.dirname(__file__), "demos.yaml")


# ---------------------------------------------------------------------------
# Parameter resolution
# ---------------------------------------------------------------------------


def resolve_screen_params(
    format_defaults: dict,
    fmt: str,
    cli_overrides: dict,
    demo_overrides: dict,
) -> dict:
    """Merge recording parameters: format_defaults → CLI → per-demo override.

    Parameters
    ----------
    format_defaults : dict
        Top-level ``format_defaults`` section from demos.yaml.
    fmt : str
        Target format key (typically ``"mp4"``).
    cli_overrides : dict
        CLI arguments that were explicitly provided (non-None).
    demo_overrides : dict
        Per-demo dict which may contain format sub-keys and ``sim_duration``.

    Returns
    -------
    dict
        Merged parameter dict with keys like fps, duration, sim_duration, etc.
    """
    # Layer 1: format defaults
    params = dict(format_defaults.get(fmt, {}))

    # Layer 2: CLI overrides
    params.update(cli_overrides)

    # Layer 3: per-demo format overrides
    params.update(demo_overrides.get(fmt, {}))

    # Also pull in top-level demo keys like sim_duration, delay, recording_duration
    for key in ("sim_duration", "delay", "recording_duration"):
        if key in demo_overrides:
            params[key] = demo_overrides[key]

    return params


# ---------------------------------------------------------------------------
# Command builder
# ---------------------------------------------------------------------------


def build_screen_capture_cmd(
    script_path: str,
    output_path: str,
    duration: float,
    fps: int,
    demo_args: list,
    recorder: str = "ffmpeg",
    stabilize_delay: float = None,
    window_wait: int = None,
) -> list:
    """Build the ``screen_capture.sh`` command for one demo.

    Parameters
    ----------
    script_path : str
        Path to the demo Python script.
    output_path : str
        Where to save the recording (e.g. ``docs/media/demo.mp4``).
    duration : float
        Recording duration in seconds.
    fps : int
        Capture framerate.
    demo_args : list
        Extra arguments to pass to the demo script.
    recorder : str
        ``"ffmpeg"`` (default) or ``"kazam"``.
    stabilize_delay : float, optional
        Seconds to wait after window appears before recording.
    window_wait : int, optional
        Seconds to wait for the window to appear.

    Returns
    -------
    list
        Command list suitable for ``subprocess.run()``.
    """
    cmd = [
        SCREEN_CAPTURE_SH,
        "-o",
        output_path,
        "-d",
        f"{duration:g}",
        "-f",
        str(int(fps)),
    ]

    if recorder == "kazam":
        cmd.append("--kazam")

    if stabilize_delay is not None:
        cmd.extend(["--delay", f"{stabilize_delay:g}"])

    if window_wait is not None:
        cmd.extend(["--wait", str(int(window_wait))])

    # End-of-options separator so demo args like --mode=mixed aren't
    # interpreted as screen_capture.sh options.
    cmd.append("--")

    # Demo script and its args (positional, must come last)
    cmd.append(script_path)
    cmd.extend(demo_args)

    return cmd


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    """Build the argument parser."""
    p = argparse.ArgumentParser(description="Record demo videos via screen_capture.sh, driven by demos.yaml")
    p.add_argument("--demo", default=None, help="Record a single demo by name (default: all)")
    p.add_argument("--list", action="store_true", help="List available demos and exit")
    p.add_argument("--config", default=DEFAULT_CONFIG, help="Config YAML (default: scripts/demos.yaml)")
    p.add_argument("--duration", type=float, default=None, help="Recording duration in seconds")
    p.add_argument("--fps", type=int, default=None, help="Capture FPS")
    p.add_argument("--output-dir", default="docs/media", help="Output directory (default: docs/media)")
    p.add_argument(
        "--recorder",
        default="ffmpeg",
        choices=["ffmpeg", "kazam"],
        help="Recording backend (default: ffmpeg)",
    )
    p.add_argument("--delay", type=float, default=None, help="Stabilize delay after window appears (seconds)")
    p.add_argument("--wait", type=int, default=None, help="Window detection timeout (seconds)")
    p.add_argument("--dry-run", action="store_true", help="Print commands without executing")
    return p


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main(argv: list = None) -> int:
    """Entry point.  Returns exit code (0 = all OK)."""
    parser = build_parser()
    args = parser.parse_args(argv)

    # Load config
    with open(args.config) as f:
        cfg = yaml.safe_load(f)

    format_defaults = cfg.get("format_defaults", {})
    demos: dict = cfg.get("demos", {})

    if args.list:
        print("Available demos:")
        for name, info in sorted(demos.items()):
            print(f"  {name:30s} → {info['script']}")
        return 0

    # Validate --demo
    if args.demo and args.demo not in demos:
        print(f"ERROR: Unknown demo '{args.demo}'. Use --list to see available demos.")
        return 1

    demos_to_run = {args.demo: demos[args.demo]} if args.demo else demos

    if not demos_to_run:
        print("No demos configured. Check demos.yaml.")
        return 1

    # Collect explicit CLI overrides (non-None only)
    cli_overrides = {}
    if args.duration is not None:
        cli_overrides["duration"] = args.duration
    if args.fps is not None:
        cli_overrides["fps"] = args.fps

    os.makedirs(args.output_dir, exist_ok=True)
    results = []

    for name, info in demos_to_run.items():
        output = os.path.join(args.output_dir, f"{name}.mp4")
        script = os.path.join(PROJECT_ROOT, info["script"])

        if not os.path.exists(script):
            print(f"  SKIP {name}: script not found ({info['script']})")
            results.append((name, "SKIP"))
            continue

        # Resolve params.  Per-demo ``recording_duration`` takes priority
        # over the shared ``format_defaults.mp4.duration``.
        params = resolve_screen_params(format_defaults, "mp4", cli_overrides, info)
        duration = params.get("recording_duration", params.get("duration", 10))
        fps = params.get("fps", 30)

        # Per-demo delay (from YAML) takes priority; fall back to CLI --delay
        stabilize_delay = params.get("delay", args.delay)

        # Build demo args from YAML only.  We intentionally do NOT inject
        # --duration or --rtf: the demo runs at its native speed until
        # screen_capture.sh kills it after the recording completes.
        # This keeps screen capture independent of simulation parameters.
        #
        # NOTE: Demo scripts must default to duration=None (run forever)
        # so the GUI window stays open for delay + recording_duration.
        demo_args = list(info.get("args", []))

        cmd = build_screen_capture_cmd(
            script_path=script,
            output_path=output,
            duration=duration,
            fps=fps,
            demo_args=demo_args,
            recorder=args.recorder,
            stabilize_delay=stabilize_delay,
            window_wait=args.wait,
        )

        print(f"\n=== {name} ===")
        print(f"  cmd: {' '.join(cmd)}")

        if args.dry_run:
            results.append((name, "DRY-RUN"))
            continue

        try:
            result = subprocess.run(cmd, cwd=PROJECT_ROOT, timeout=duration + 60)
            if result.returncode == 0 and os.path.exists(output):
                size_mb = os.path.getsize(output) / (1024 * 1024)
                print(f"  OK → {output} ({size_mb:.1f} MB)")
                results.append((name, "OK"))
            else:
                print(f"  FAIL (exit code {result.returncode})")
                results.append((name, "FAIL"))
        except subprocess.TimeoutExpired:
            print("  TIMEOUT")
            results.append((name, "TIMEOUT"))
        except Exception as e:
            print(f"  ERROR: {e}")
            results.append((name, "ERROR"))

    # Summary
    print(f"\n{'='*50}")
    print(f"{'Demo':30s} {'Status':8s}")
    print(f"{'-'*50}")
    for name, status in results:
        print(f"{name:30s} {status:8s}")
    ok = sum(1 for _, s in results if s in ("OK", "DRY-RUN"))
    print(f"\n{ok}/{len(results)} demos recorded successfully")

    return 0 if all(s in ("OK", "DRY-RUN", "SKIP") for _, s in results) else 1


if __name__ == "__main__":
    sys.exit(main())
