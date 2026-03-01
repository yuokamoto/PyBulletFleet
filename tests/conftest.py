"""Shared fixtures for all test modules."""

import pytest

from pybullet_fleet.data_monitor import DataMonitor
from pybullet_fleet.sim_object import SimObject


@pytest.fixture(autouse=True)
def _clear_shared_shapes():
    """Clear SimObject shape cache before every test.

    ``SimObject._shared_shapes`` is a class-level dict that caches PyBullet
    shape IDs.  Those IDs become invalid after ``p.disconnect()``, so they
    must be cleared before each test to prevent stale IDs from being reused
    in a new physics session.
    """
    SimObject._shared_shapes.clear()


@pytest.fixture(autouse=True)
def _disable_monitor_gui(monkeypatch):
    """Prevent DataMonitor from opening matplotlib windows during tests.

    Even when ``monitor=False`` is passed to SimulationParams, some code
    paths may still instantiate a DataMonitor.  This fixture globally
    patches ``DataMonitor.start`` to be a no-op so that matplotlib windows
    never appear during headless test runs.
    """
    monkeypatch.setattr(DataMonitor, "start", lambda self: None)
