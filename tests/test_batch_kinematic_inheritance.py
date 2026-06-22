"""Tests for ``BatchKinematicController`` architecture and API constraints.

BatchKinematicController inherits from Controller directly (not
KinematicController) to avoid carrying unused per-agent TPI/path state.
Velocity-mode is explicitly blocked — per-agent controllers should be used
when velocity commands are needed.
"""

from __future__ import annotations

import pytest

from pybullet_fleet.controller import Controller
from pybullet_fleet.controllers import BatchDifferentialController, BatchOmniController
from pybullet_fleet.controllers.batch_base import BatchKinematicController


class TestBatchIsController:
    def test_batch_base_is_controller(self) -> None:
        assert issubclass(BatchKinematicController, Controller)

    def test_batch_omni_is_controller(self) -> None:
        assert issubclass(BatchOmniController, Controller)

    def test_batch_diff_is_controller(self) -> None:
        assert issubclass(BatchDifferentialController, Controller)


class TestVelocityModeNotSupported:
    def test_set_velocity_on_omni_raises(self) -> None:
        bc = BatchOmniController()
        with pytest.raises(NotImplementedError, match="batch"):
            bc.set_velocity(vx=1.0)

    def test_set_velocity_on_diff_raises(self) -> None:
        bc = BatchDifferentialController()
        with pytest.raises(NotImplementedError, match="batch"):
            bc.set_velocity(vx=1.0)


class TestNavigation2D:
    def test_navigation_2d_default(self) -> None:
        """BC has no navigation_2d of its own; reads from agent.controller_params."""
        bc = BatchOmniController()
        assert not hasattr(bc, "_navigation_2d")
