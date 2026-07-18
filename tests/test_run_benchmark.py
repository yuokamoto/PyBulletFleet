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


def _make_batch_result(**overrides):
    """Return a minimal batch-worker result dict."""
    base = {
        "setup_s": 0.01,
        "accepted": 10,
        "rejected": 0,
        "wall_s": 0.2,
        "mean_ms": 1.0,
        "p50_ms": 0.9,
        "p95_ms": 1.5,
        "max_ms": 2.0,
        "state_snapshot_s": 0.001,
    }
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

    @patch("run_benchmark.run_worker")
    def test_batch_zero_reps_does_not_raise(self, mock_worker):
        mock_worker.return_value = None
        result = run_multiple(
            num_agents=5,
            duration=1.0,
            num_reps=0,
            benchmark_type="mobile_control_path",
            steps=10,
            controller="batch",
            command_interface="fleet",
        )
        assert result["num_agents"] == 5
        assert result["num_reps"] == 0
        assert result["controller_impl"] == "batch"
        assert result["command_interface"] == "fleet"

    @patch("run_benchmark.run_worker")
    def test_batch_zero_reps_reports_effective_defaults(self, mock_worker):
        mock_worker.return_value = None
        result = run_multiple(
            num_agents=5,
            duration=1.0,
            num_reps=0,
            benchmark_type="mobile_control_path",
        )
        assert result["steps"] == 600
        assert result["collision_freq"] == 60
        assert result["controller_impl"] == "batch"
        assert result["command_interface"] == "fleet"


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

    @patch("run_benchmark.run_worker")
    def test_batch_single_rep(self, mock_worker):
        mock_worker.return_value = _make_batch_result()
        result = run_multiple(
            num_agents=10,
            duration=1.0,
            num_reps=1,
            benchmark_type="mobile_control_path",
            steps=20,
            mode="diff",
            collision_freq=0,
            controller="batch",
            command_interface="fleet",
        )
        assert result["benchmark_type"] == "mobile_control_path"
        assert result["steps"] == 20
        assert result["controller_impl"] == "batch"
        assert result["command_interface"] == "fleet"
        assert result["mode"] == "diff"
        assert result["collision_freq"] == 0
        assert result["setup_s"]["median"] == 0.01
        assert result["accepted"]["median"] == 10
