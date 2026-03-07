"""
Test cases for memory profiling functionality.

This test suite validates the memory profiling feature to ensure:
1. Memory profiling can be enabled/disabled via configuration
2. Memory statistics are collected correctly during simulation
3. Memory usage API returns valid data
4. Memory profiling works alongside time profiling
"""

import pytest
from pybullet_fleet.core_simulation import MultiRobotSimulationCore


class TestMemoryProfiling:
    """Test suite for memory profiling feature."""

    def test_memory_profiling_disabled_by_default(self):
        """Memory profiling should be disabled by default."""
        config = {
            "timestep": 0.1,
            "target_rtf": 1.0,
            "duration": 0,
            "gui": False,
            "physics": False,
            "monitor": False,
        }
        sim = MultiRobotSimulationCore.from_dict(config)
        assert sim.enable_memory_profiling is False
        assert sim._memory_tracemalloc_started is False

    def test_memory_profiling_can_be_enabled(self):
        """Memory profiling can be enabled via configuration."""
        config = {
            "timestep": 0.1,
            "target_rtf": 1.0,
            "duration": 0,
            "gui": False,
            "physics": False,
            "monitor": False,
            "enable_memory_profiling": True,
        }
        sim = MultiRobotSimulationCore.from_dict(config)
        assert sim.enable_memory_profiling is True

    def test_memory_profiling_get_usage_returns_none_when_disabled(self):
        """get_memory_usage() should return None when profiling is disabled."""
        config = {
            "timestep": 0.1,
            "target_rtf": 1.0,
            "duration": 0,
            "gui": False,
            "physics": False,
            "monitor": False,
            "enable_memory_profiling": False,
        }
        sim = MultiRobotSimulationCore.from_dict(config)
        assert sim.get_memory_usage() is None

    def test_memory_profiling_collects_data(self):
        """Memory profiling should collect data during simulation."""
        config = {
            "timestep": 0.1,
            "target_rtf": 0,  # Run as fast as possible
            "duration": 0,
            "gui": False,
            "physics": False,
            "monitor": False,
            "enable_memory_profiling": True,
            "profiling_interval": 10,
        }
        sim = MultiRobotSimulationCore.from_dict(config)

        # Run a few steps
        sim.initialize_simulation()
        for _ in range(20):
            sim.step_once()

        # Memory usage should be available
        mem_usage = sim.get_memory_usage()
        assert mem_usage is not None
        assert "current_mb" in mem_usage
        assert "peak_mb" in mem_usage
        assert mem_usage["current_mb"] > 0
        assert mem_usage["peak_mb"] >= mem_usage["current_mb"]

    def test_memory_profiling_with_time_profiling(self):
        """Memory profiling should work alongside time profiling."""
        config = {
            "timestep": 0.1,
            "target_rtf": 0,  # Run as fast as possible
            "duration": 0,
            "gui": False,
            "physics": False,
            "monitor": False,
            "enable_time_profiling": True,  # Time profiling
            "enable_memory_profiling": True,  # Memory profiling
            "profiling_interval": 10,
        }
        sim = MultiRobotSimulationCore.from_dict(config)

        # Run a few steps
        sim.initialize_simulation()
        for _ in range(20):
            sim.step_once()

        # Both profiling features should work
        assert sim._enable_time_profiling is True
        assert sim.enable_memory_profiling is True

        # Memory usage should be available
        mem_usage = sim.get_memory_usage()
        assert mem_usage is not None
        assert mem_usage["current_mb"] > 0

    def test_memory_profiling_detects_memory_growth(self):
        """Memory profiling should detect memory usage changes."""
        config = {
            "timestep": 0.1,
            "target_rtf": 0,  # Run as fast as possible
            "duration": 0,
            "gui": False,
            "physics": False,
            "monitor": False,
            "enable_memory_profiling": True,
        }
        sim = MultiRobotSimulationCore.from_dict(config)
        sim.initialize_simulation()

        # Get initial memory
        mem_before = sim.get_memory_usage()
        assert mem_before is not None

        # Allocate some memory (create large lists)
        temp_data = []
        for _ in range(100):
            sim.step_once()
            # Allocate memory to simulate memory growth
            temp_data.append([0] * 10000)

        # Get memory after allocation
        mem_after = sim.get_memory_usage()
        assert mem_after is not None

        # Memory should have increased
        assert mem_after["current_mb"] > mem_before["current_mb"]


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
