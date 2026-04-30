"""Tests for SimObject._needs_update and update() hook."""

import pytest

from tests.conftest import MockSimCore
from pybullet_fleet import SimObject, Pose
from pybullet_fleet.sim_object import SimObjectSpawnParams


@pytest.fixture
def sim_core(pybullet_env):
    sc = MockSimCore()
    sc._client = pybullet_env
    return sc


class TestSimObjectUpdate:
    def test_needs_update_default_false(self, sim_core):
        """SimObject._needs_update defaults to False."""
        params = SimObjectSpawnParams(
            visual_shape=None,
            collision_shape=None,
            initial_pose=Pose.from_xyz(0, 0, 0),
        )
        obj = SimObject.from_params(params, sim_core)
        assert obj._needs_update is False

    def test_update_returns_false_by_default(self, sim_core):
        """SimObject.update() returns False (no-op)."""
        params = SimObjectSpawnParams(
            visual_shape=None,
            collision_shape=None,
            initial_pose=Pose.from_xyz(0, 0, 0),
        )
        obj = SimObject.from_params(params, sim_core)
        assert obj.update(0.01) is False

    def test_subclass_with_needs_update(self, sim_core):
        """Subclass with _needs_update=True gets update() called in unified loop."""
        call_count = 0

        class UpdatingObject(SimObject):
            _needs_update = True

            def update(self, dt):
                nonlocal call_count
                call_count += 1
                return True

        params = SimObjectSpawnParams(
            visual_shape=None,
            collision_shape=None,
            initial_pose=Pose.from_xyz(0, 0, 0),
        )
        obj = UpdatingObject.from_params(params, sim_core)
        sim_core.tick(5)
        assert call_count == 5
