#!/usr/bin/env python3
"""
Shared benchmark and profiling helpers.

Extracted from arm_benchmark.py and performance_benchmark.py to eliminate
duplication of system info collection, memory measurement, cleanup, and
warmup routines.
"""
import gc
import os
import platform
import subprocess
import time
from contextlib import contextmanager
from typing import Any, Dict, Optional

import psutil
import tracemalloc
import yaml
import pybullet as p


def get_system_info() -> Dict[str, Any]:
    """Collect system info (CPU, memory, OS) for reproducibility.

    Returns:
        Dictionary with platform, CPU, memory, and Python version details.
    """
    info = {
        "platform": platform.system(),
        "platform_release": platform.release(),
        "platform_version": platform.version(),
        "architecture": platform.machine(),
        "processor": platform.processor(),
        "python_version": platform.python_version(),
        "cpu_count": psutil.cpu_count(logical=False),
        "cpu_count_logical": psutil.cpu_count(logical=True),
        "total_memory_gb": round(psutil.virtual_memory().total / (1024**3), 2),
    }
    if platform.system() == "Linux":
        try:
            result = subprocess.run(["lscpu"], capture_output=True, text=True, timeout=2)
            for line in result.stdout.split("\n"):
                if "Model name:" in line:
                    info["cpu_model"] = line.split(":", 1)[1].strip()
                elif "CPU max MHz:" in line:
                    try:
                        info["cpu_max_mhz"] = float(line.split(":", 1)[1].strip())
                    except (ValueError, IndexError):
                        pass
        except (FileNotFoundError, IOError):
            pass
    return info


def get_memory_info() -> Dict[str, float]:
    """Get RSS + tracemalloc memory measurements.

    Returns:
        Dictionary with rss_mb, vms_mb, py_traced_mb, rss_minus_tracemalloc_mb.
    """
    process = psutil.Process()
    mem = process.memory_info()
    py_traced_mb = 0.0
    if tracemalloc.is_tracing():
        current, _peak = tracemalloc.get_traced_memory()
        py_traced_mb = current / 1024 / 1024
    rss_mb = mem.rss / 1024 / 1024
    return {
        "rss_mb": rss_mb,
        "vms_mb": mem.vms / 1024 / 1024,
        "py_traced_mb": py_traced_mb,
        "rss_minus_tracemalloc_mb": rss_mb - py_traced_mb,
    }


def force_cleanup():
    """Best-effort cleanup inside a process (triple gc.collect + sleep)."""
    gc.collect()
    gc.collect()
    gc.collect()
    time.sleep(0.05)


def cpu_time_s(process: psutil.Process) -> float:
    """Return user+sys CPU time seconds."""
    t = process.cpu_times()
    return float(t.user + t.system)


def ensure_disconnected():
    """Disconnect PyBullet if connected."""
    if p.isConnected():
        p.disconnect()


@contextmanager
def suppress_stdout():
    """Suppress C-level stdout/stderr (e.g. PyBullet URDF warnings).

    Works by redirecting file descriptors at the OS level, which catches
    output from C extensions that bypass Python's ``sys.stdout``.

    Usage::

        with suppress_stdout():
            p.loadURDF(...)
    """
    devnull = os.open(os.devnull, os.O_WRONLY)
    old_stdout_fd = os.dup(1)
    old_stderr_fd = os.dup(2)
    try:
        os.dup2(devnull, 1)
        os.dup2(devnull, 2)
        yield
    finally:
        os.dup2(old_stdout_fd, 1)
        os.dup2(old_stderr_fd, 2)
        os.close(devnull)
        os.close(old_stdout_fd)
        os.close(old_stderr_fd)


def warmup_steps(sim_core, n: int = 10):
    """Run *n* warmup steps to stabilise timing."""
    for _ in range(n):
        sim_core.step_once()


def load_config(config_path: str, scenario: Optional[str] = None) -> Dict[str, Any]:
    """Load benchmark configuration from YAML file.

    Args:
        config_path: Path to YAML config file.
        scenario: Optional scenario name. When given, the matching
            entry under ``scenarios:`` is deep-merged into the
            top-level config (dict values are updated, others replaced).

    Returns:
        Merged configuration dictionary, or ``{}`` if *config_path*
        does not exist.
    """
    if not os.path.exists(config_path):
        return {}

    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    # Merge scenario config if specified
    if scenario and "scenarios" in config and scenario in config["scenarios"]:
        base_config = config.copy()
        scenario_config = config["scenarios"][scenario]

        for key, value in scenario_config.items():
            if isinstance(value, dict) and key in base_config:
                base_config[key].update(value)
            else:
                base_config[key] = value

        return base_config

    return config


def cleanup_simulation(sim_core=None, agents=None, extra_refs=None):
    """Clean up simulation resources.

    Args:
        sim_core: MultiRobotSimulationCore instance to delete.
        agents: List of agents to delete.
        extra_refs: Additional objects to delete (list).
    """
    if p.isConnected():
        p.resetSimulation()
        p.disconnect()
    if tracemalloc.is_tracing():
        tracemalloc.stop()
    # Delete references to help GC
    if agents is not None:
        del agents[:]
    if extra_refs is not None:
        for ref in extra_refs:
            del ref
    force_cleanup()
