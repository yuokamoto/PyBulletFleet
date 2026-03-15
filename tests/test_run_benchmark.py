"""Tests for benchmark/run_benchmark.py aggregation logic."""

import sys
import os
from unittest.mock import patch

# Ensure benchmark/ is importable
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "benchmark"))

from run_benchmark import run_multiple


def _make_mobile_result(**overrides):
    """Return a minimal mobile-worker result dict."""
    base = {
        "spawn_time_s": 0.1,
        "spawn_cpu_s": 0.05,
        "spawn_cpu_percent": 50.0,
        "simulation_wall_s": 1.0,
        "simulation_cpu_s": 0.8,
        "simulation_cpu_percent": 80.0,
        "real_time_factor": 1.0,
        "avg_step_time_ms": 1.0,
        "system_info": {"cpu": "test"},
        "mem_spawn_mb": {"rss_mb": 10.0},
        "mem_total_mb": {"rss_mb": 20.0},
        "expected_steps": 100,
    }
    base.update(overrides)
    return base


def _make_arm_result(**overrides):
    """Return a minimal arm-worker result dict."""
    base = _make_mobile_result()
    base.pop("expected_steps", None)
    base.update({"mode": "kinematic", "timestep": 0.01, "total_joints": 6})
    base.update(overrides)
    return base


class TestRunMultipleEmptyResults:
    """run_multiple must not crash when num_reps=0 (empty results)."""

    @patch("run_benchmark.run_worker")
    def test_mobile_zero_reps_does_not_raise(self, mock_worker):
        """IndexError guard: results[0] must not be accessed when results is empty."""
        mock_worker.return_value = None  # never called anyway
        result = run_multiple(
            num_agents=10,
            duration=1.0,
            num_reps=0,
            benchmark_type="mobile",
        )
        assert result["num_agents"] == 10
        assert result["num_reps"] == 0
        # Mobile-specific key should be absent
        assert "expected_steps" not in result

    @patch("run_benchmark.run_worker")
    def test_arm_zero_reps_does_not_raise(self, mock_worker):
        """IndexError guard: arm-specific results[0] must be guarded."""
        mock_worker.return_value = None
        result = run_multiple(
            num_agents=5,
            duration=1.0,
            num_reps=0,
            benchmark_type="arm",
        )
        assert result["num_agents"] == 5
        assert result["num_reps"] == 0


class TestRunMultipleWithResults:
    """run_multiple aggregates results correctly when num_reps >= 1."""

    @patch("run_benchmark.run_worker")
    def test_mobile_single_rep(self, mock_worker):
        mock_worker.return_value = _make_mobile_result()
        result = run_multiple(
            num_agents=10,
            duration=1.0,
            num_reps=1,
            benchmark_type="mobile",
        )
        assert result["num_agents"] == 10
        assert result["expected_steps"] == 100

    @patch("run_benchmark.run_worker")
    def test_arm_single_rep(self, mock_worker):
        mock_worker.return_value = _make_arm_result()
        result = run_multiple(
            num_agents=5,
            duration=1.0,
            num_reps=1,
            benchmark_type="arm",
        )
        assert result["mode"] == "kinematic"
        assert "expected_steps" not in result
