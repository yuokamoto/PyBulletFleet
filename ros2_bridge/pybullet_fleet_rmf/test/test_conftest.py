"""Tests for RMF pytest fixtures."""

from __future__ import annotations

import importlib.util
from pathlib import Path


def _load_conftest():
    path = Path(__file__).resolve().parent / "conftest.py"
    spec = importlib.util.spec_from_file_location("rmf_test_conftest", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_missing_ros_dependency_filter_only_accepts_external_ros_packages():
    module = _load_conftest()

    assert module._is_missing_ros_dependency(ImportError(name="rclpy.action"))
    assert module._is_missing_ros_dependency(ImportError(name="nav2_msgs"))
    assert not module._is_missing_ros_dependency(ImportError(name="pybullet_fleet_rmf.internal"))
    assert not module._is_missing_ros_dependency(ImportError("custom import failure"))
