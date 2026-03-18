"""Shared fixtures for all test modules."""

from types import SimpleNamespace

import pybullet as p
import pytest

from pybullet_fleet.data_monitor import DataMonitor
from pybullet_fleet.sim_object import SimObject


# ---------------------------------------------------------------------------
# Shared mock sim_core classes
# ---------------------------------------------------------------------------


class MockSimCore:
    """Minimal sim_core mock for tests.

    Provides object registry, time tracking, and optional physics stepping.

    When ``physics=True``, ``tick()`` calls ``p.stepSimulation()`` each step
    so that motor-controlled joints (via ``setJointMotorControl2``) take effect.

    When ``physics=False`` (default), ``tick()`` only advances ``sim_time``
    — joints move via ``resetJointState`` (kinematic interpolation).
    """

    def __init__(self, dt: float = 1.0 / 240.0, *, physics: bool = False):
        self.sim_time: float = 0.0
        self._dt: float = dt
        self.sim_objects: list = []
        self._next_object_id: int = 0
        self._kinematic_objects: set = set()
        self._client: int = 0
        self._params = SimpleNamespace(physics=physics)

    @property
    def client(self) -> int:
        return self._client

    def add_object(self, obj):
        self.sim_objects.append(obj)

    def remove_object(self, obj):
        if obj in self.sim_objects:
            self.sim_objects.remove(obj)

    def _mark_object_moved(self, object_id):
        pass

    def register_callback(self, callback, frequency=None):
        """Record registered callbacks (used by AgentManager tests)."""
        if not hasattr(self, "_registered_callbacks"):
            self._registered_callbacks: list = []
        self._registered_callbacks.append({"func": callback, "frequency": frequency})

    def tick(self, n: int = 1):
        """Advance sim_time by *n* time-steps, with physics stepping if enabled."""
        if self._params.physics:
            for _ in range(n):
                p.stepSimulation()
                self.sim_time += self._dt
        else:
            self.sim_time += self._dt * n


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
