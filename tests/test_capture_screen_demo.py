"""Tests for capture_screen_demo — YAML-driven screen recording orchestrator."""

import os
import sys

# Import the module under test (will fail until we create it)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from capture_screen_demo import build_screen_capture_cmd, resolve_screen_params


class TestResolveScreenParams:
    """Test parameter resolution: format_defaults → CLI → per-demo override."""

    def test_format_defaults_only(self):
        format_defaults = {"mp4": {"fps": 30, "duration": 6.0}}
        result = resolve_screen_params(format_defaults, "mp4", cli_overrides={}, demo_overrides={})
        assert result["fps"] == 30
        assert result["duration"] == 6.0

    def test_cli_overrides_format_defaults(self):
        format_defaults = {"mp4": {"fps": 30, "duration": 6.0}}
        cli = {"fps": 60}
        result = resolve_screen_params(format_defaults, "mp4", cli_overrides=cli, demo_overrides={})
        assert result["fps"] == 60
        assert result["duration"] == 6.0

    def test_demo_overrides_cli(self):
        format_defaults = {"mp4": {"fps": 30, "duration": 6.0}}
        cli = {"fps": 60}
        demo = {"mp4": {"fps": 15}}
        result = resolve_screen_params(format_defaults, "mp4", cli_overrides=cli, demo_overrides=demo)
        assert result["fps"] == 15

    def test_missing_format_returns_empty_base(self):
        result = resolve_screen_params({}, "mp4", cli_overrides={}, demo_overrides={})
        assert isinstance(result, dict)

    def test_delay_from_demo(self):
        """delay is a top-level demo key, passed through demo_overrides."""
        demo = {"delay": 3}
        result = resolve_screen_params({}, "mp4", cli_overrides={}, demo_overrides=demo)
        assert result.get("delay") == 3

    def test_delay_not_present_when_unset(self):
        """delay should NOT appear in result when not specified anywhere."""
        result = resolve_screen_params({}, "mp4", cli_overrides={}, demo_overrides={})
        assert "delay" not in result

    def test_recording_duration_from_demo(self):
        """recording_duration is a top-level demo key, passed through demo_overrides."""
        demo = {"recording_duration": 15.0}
        result = resolve_screen_params({}, "mp4", cli_overrides={}, demo_overrides=demo)
        assert result.get("recording_duration") == 15.0

    def test_recording_duration_overrides_format_duration(self):
        """Per-demo recording_duration should take priority over format_defaults duration."""
        format_defaults = {"mp4": {"duration": 6.0}}
        demo = {"recording_duration": 12.0}
        result = resolve_screen_params(format_defaults, "mp4", cli_overrides={}, demo_overrides=demo)
        # recording_duration is a separate key that the caller uses to override duration
        assert result.get("recording_duration") == 12.0
        assert result.get("duration") == 6.0  # format default unchanged


class TestBuildScreenCaptureCmd:
    """Test building the screen_capture.sh invocation command."""

    def test_basic_command(self):
        cmd = build_screen_capture_cmd(
            script_path="examples/scale/demo.py",
            output_path="docs/media/demo.mp4",
            duration=10,
            fps=30,
            demo_args=[],
        )
        assert cmd[0].endswith("screen_capture.sh")
        assert "-o" in cmd
        assert "docs/media/demo.mp4" in cmd
        assert "-d" in cmd
        assert "10" in cmd
        assert "-f" in cmd
        assert "30" in cmd
        assert "examples/scale/demo.py" in cmd

    def test_with_demo_args(self):
        cmd = build_screen_capture_cmd(
            script_path="examples/scale/demo.py",
            output_path="docs/media/demo.mp4",
            duration=10,
            fps=30,
            demo_args=["--mode=mixed", "--robots=50"],
        )
        # "--" separator must appear before the demo script to prevent
        # screen_capture.sh from interpreting demo args as its own options
        assert "--" in cmd
        sep_idx = cmd.index("--")
        script_idx = cmd.index("examples/scale/demo.py")
        assert sep_idx < script_idx
        assert "--mode=mixed" in cmd[script_idx + 1 :]
        assert "--robots=50" in cmd[script_idx + 1 :]

    def test_separator_always_present(self):
        """'--' must be emitted even when demo_args is empty."""
        cmd = build_screen_capture_cmd(
            script_path="examples/scale/demo.py",
            output_path="docs/media/demo.mp4",
            duration=10,
            fps=30,
            demo_args=[],
        )
        assert "--" in cmd
        sep_idx = cmd.index("--")
        script_idx = cmd.index("examples/scale/demo.py")
        assert sep_idx < script_idx

    def test_custom_recorder(self):
        cmd = build_screen_capture_cmd(
            script_path="examples/scale/demo.py",
            output_path="docs/media/demo.mp4",
            duration=10,
            fps=30,
            demo_args=[],
            recorder="kazam",
        )
        assert "--kazam" in cmd

    def test_default_recorder_is_ffmpeg(self):
        cmd = build_screen_capture_cmd(
            script_path="examples/scale/demo.py",
            output_path="docs/media/demo.mp4",
            duration=10,
            fps=30,
            demo_args=[],
        )
        assert "--kazam" not in cmd

    def test_stabilize_delay(self):
        cmd = build_screen_capture_cmd(
            script_path="examples/scale/demo.py",
            output_path="docs/media/demo.mp4",
            duration=10,
            fps=30,
            demo_args=[],
            stabilize_delay=5,
        )
        delay_idx = cmd.index("--delay")
        assert cmd[delay_idx + 1] == "5"

    def test_stabilize_delay_fractional(self):
        """Fractional stabilize_delay must be preserved, not truncated."""
        cmd = build_screen_capture_cmd(
            script_path="examples/scale/demo.py",
            output_path="docs/media/demo.mp4",
            duration=10,
            fps=30,
            demo_args=[],
            stabilize_delay=2.5,
        )
        delay_idx = cmd.index("--delay")
        assert cmd[delay_idx + 1] == "2.5"

    def test_screen_capture_does_not_inject_duration_or_rtf(self):
        """Screen capture mode should NOT inject --duration or --rtf to demo args.

        Previous design injected --duration={sim_duration} and --rtf=1.0.
        New design: demo runs at native speed, killed after recording.
        """
        demo_info = {"args": ["--mode=mixed"]}

        # Simulate new main() logic: only pass through YAML-defined args
        demo_args = list(demo_info.get("args", []))
        # No --duration, no --rtf injection

        cmd = build_screen_capture_cmd(
            script_path="examples/scale/demo.py",
            output_path="docs/media/demo.mp4",
            duration=10,
            fps=30,
            demo_args=demo_args,
        )
        script_idx = cmd.index("examples/scale/demo.py")
        trailing = cmd[script_idx + 1 :]
        assert "--mode=mixed" in trailing
        # Neither --duration nor --rtf should be injected
        assert not any("--duration" in a for a in trailing)
        assert not any("--rtf" in a for a in trailing)

    def test_recording_duration_used_for_screen_capture_duration(self):
        """Per-demo recording_duration should override format_defaults duration
        when passed to build_screen_capture_cmd."""
        format_defaults = {"mp4": {"duration": 6.0}}
        demo_info = {"recording_duration": 15.0}
        params = resolve_screen_params(format_defaults, "mp4", cli_overrides={}, demo_overrides=demo_info)

        # Simulate main() logic: recording_duration takes priority over duration
        duration = params.get("recording_duration", params.get("duration", 10))
        assert duration == 15.0

        cmd = build_screen_capture_cmd(
            script_path="examples/scale/demo.py",
            output_path="docs/media/demo.mp4",
            duration=duration,
            fps=30,
            demo_args=[],
        )
        assert "15" in cmd  # -d 15

    def test_fallback_to_format_duration_when_no_recording_duration(self):
        """When recording_duration is not set, falls back to format_defaults duration."""
        format_defaults = {"mp4": {"duration": 6.0}}
        demo_info = {}  # no recording_duration
        params = resolve_screen_params(format_defaults, "mp4", cli_overrides={}, demo_overrides=demo_info)

        duration = params.get("recording_duration", params.get("duration", 10))
        assert duration == 6.0
