"""Asymmetric accel/decel support in the TPI layer and the per-agent controllers.

``build_tpi`` used to pin ``dec_max`` to ``accel``, so a vehicle that brakes
harder than it accelerates could not be modelled even though
``TwoPointInterpolation`` has supported ``dec_max`` all along. These tests pin
the new behaviour and the guard that keeps the batched controllers — which
integrate the decel phase with the accel scalar — from silently producing
wrong positions.
"""

import math

import pytest

from pybullet_fleet._tpi import build_tpi, extract_phase_params
from pybullet_fleet.controller_params import ControllerParams


class TestBuildTpiDecel:
    def test_decel_defaults_to_accel(self):
        """Omitting decel reproduces the symmetric profile of earlier releases."""
        tpi = build_tpi(p0=0.0, pe=5.0, vmax=2.0, accel=1.0, t0=0.0)
        assert tpi.amax_decel == pytest.approx(tpi.amax_accel)

    def test_decel_is_forwarded(self):
        tpi = build_tpi(p0=0.0, pe=5.0, vmax=2.0, accel=1.0, t0=0.0, decel=0.25)
        assert tpi.amax_accel == pytest.approx(1.0)
        assert tpi.amax_decel == pytest.approx(0.25)

    def test_softer_braking_takes_longer(self):
        """A weaker decel must stretch the trajectory, not leave it unchanged."""
        symmetric = build_tpi(p0=0.0, pe=5.0, vmax=2.0, accel=1.0, t0=0.0)
        soft_brake = build_tpi(p0=0.0, pe=5.0, vmax=2.0, accel=1.0, t0=0.0, decel=0.25)
        assert soft_brake.get_duration() > symmetric.get_duration()

    def test_endpoint_still_reached_exactly(self):
        tpi = build_tpi(p0=0.0, pe=5.0, vmax=2.0, accel=1.0, t0=0.0, decel=0.25)
        pos, vel, _ = tpi.get_point(tpi.get_end_time())
        assert pos == pytest.approx(5.0, abs=1e-6)
        assert vel == pytest.approx(0.0, abs=1e-6)

    def test_zero_distance_fallback_keeps_decel(self):
        """The degenerate p0 -> p0 fallback must not drop the decel argument."""
        tpi = build_tpi(p0=1.0, pe=1.0, vmax=2.0, accel=1.0, t0=0.0, decel=0.25)
        assert tpi.amax_decel == pytest.approx(0.25)


class TestExtractPhaseParamsGuard:
    def test_symmetric_profile_is_accepted(self):
        tpi = build_tpi(p0=0.0, pe=5.0, vmax=2.0, accel=1.0, t0=0.0)
        t_accel, t_const, t_total, accel = extract_phase_params(tpi)
        assert accel == pytest.approx(1.0)
        assert t_total > 0.0
        assert math.isclose(t_total, t_accel + t_const + (t_total - t_accel - t_const))

    def test_asymmetric_profile_is_rejected(self):
        """Batched controllers integrate decel with the accel scalar, so refuse."""
        tpi = build_tpi(p0=0.0, pe=5.0, vmax=2.0, accel=1.0, t0=0.0, decel=0.25)
        with pytest.raises(ValueError, match="symmetric accel/decel"):
            extract_phase_params(tpi)


class TestCheckpointCaptureGuard:
    def test_capture_rejects_an_asymmetric_trajectory(self):
        """The record holds one `accel`, so an asymmetric profile is not restorable."""
        from pybullet_fleet import Pose
        from pybullet_fleet.controller import OmniController
        from pybullet_fleet.types import ControllerMode, PosePhase

        controller = OmniController(ControllerParams(max_linear_accel=1.0, max_linear_decel=0.25))
        # Put the controller in the one state capture supports, then give it a
        # trajectory whose braking differs from its acceleration.
        controller._mode = ControllerMode.POSE
        controller._pose_phase = PosePhase.FORWARD
        controller._path = [Pose.from_xyz(1.0, 0.0, 0.0)]
        controller._current_waypoint_index = 0
        controller._goal_pose = Pose.from_xyz(1.0, 0.0, 0.0)
        controller._forward_start_pos = __import__("numpy").zeros(3)
        controller._tpi_forward = build_tpi(p0=0.0, pe=1.0, vmax=2.0, accel=1.0, t0=0.0, decel=0.25)

        with pytest.raises(ValueError, match="Unsupported omni navigation state"):
            controller.capture_straight_navigation()


class TestControllerParamsDecel:
    def test_unset_decel_mirrors_accel(self):
        p = ControllerParams(max_linear_accel=3.0)
        assert p.max_linear_decel is None
        assert p._eff_linear_decel() == 3.0
        assert p.scalar_max_linear_decel() == pytest.approx(3.0)

    def test_unset_decel_mirrors_framework_default_accel(self):
        p = ControllerParams()
        assert p._eff_linear_decel() == p._eff_linear_accel()

    def test_explicit_decel_is_independent(self):
        p = ControllerParams(max_linear_accel=3.0, max_linear_decel=0.5)
        assert p.scalar_max_linear_accel() == pytest.approx(3.0)
        assert p.scalar_max_linear_decel() == pytest.approx(0.5)

    def test_per_axis_decel_projects_like_accel(self):
        """Per-axis decel uses the same direction projection as per-axis accel."""
        import numpy as np

        p = ControllerParams(max_linear_accel=[1.0, 4.0, 0.0], max_linear_decel=[2.0, 8.0, 0.0])
        x_axis = np.array([1.0, 0.0, 0.0])
        y_axis = np.array([0.0, 1.0, 0.0])
        assert p.linear_accel_along_direction(x_axis) == pytest.approx(1.0)
        assert p.linear_decel_along_direction(x_axis) == pytest.approx(2.0)
        assert p.linear_accel_along_direction(y_axis) == pytest.approx(4.0)
        assert p.linear_decel_along_direction(y_axis) == pytest.approx(8.0)

    def test_from_dict_accepts_decel(self):
        p = ControllerParams.from_dict({"max_linear_accel": 2.0, "max_linear_decel": 0.5})
        assert p.max_linear_decel == 0.5
