"""Tests for ControllerParams dataclass."""

import math

import pytest

from pybullet_fleet._defaults import CONTROLLER as _CTRL_D
from pybullet_fleet.controller_params import ControllerParams
from pybullet_fleet.types import MovementDirection


class TestControllerParamsDefaults:
    def test_defaults_are_none(self):
        """All numeric fields default to None (not explicitly set)."""
        p = ControllerParams()
        assert p.max_linear_vel is None
        assert p.max_angular_vel is None
        assert p.max_linear_accel is None
        assert p.max_angular_accel is None
        assert p.cmd_vel_timeout is None
        assert p.navigation_2d is None
        assert p.default_direction == MovementDirection.FORWARD

    def test_effective_values_match_framework_defaults(self):
        """_eff_* helpers resolve None to framework defaults."""
        p = ControllerParams()
        assert p._eff_linear_vel() == _CTRL_D["max_linear_vel"]
        assert p._eff_angular_vel() == _CTRL_D["max_angular_vel"]
        assert p._eff_linear_accel() == _CTRL_D["max_linear_accel"]
        assert p._eff_angular_accel() == _CTRL_D["max_angular_accel"]
        assert p._eff_cmd_vel_timeout() == _CTRL_D["cmd_vel_timeout"]

    def test_explicit_inf_means_no_limit(self):
        """Explicitly setting math.inf still disables the limit (not auto-defaulted)."""
        p = ControllerParams(max_linear_vel=math.inf)
        assert p.max_linear_vel == math.inf


class TestControllerParamsConstruction:
    def test_all_fields_settable(self):
        p = ControllerParams(
            max_linear_vel=2.5,
            max_angular_vel=1.5,
            max_linear_accel=4.0,
            max_angular_accel=8.0,
            cmd_vel_timeout=0.5,
            navigation_2d=True,
            default_direction=MovementDirection.BACKWARD,
        )
        assert p.max_linear_vel == 2.5
        assert p.max_angular_vel == 1.5
        assert p.max_linear_accel == 4.0
        assert p.max_angular_accel == 8.0
        assert p.cmd_vel_timeout == 0.5
        assert p.navigation_2d is True
        assert p.default_direction == MovementDirection.BACKWARD

    def test_default_direction_via_enum(self):
        p = ControllerParams(default_direction=MovementDirection.BACKWARD)
        assert p.default_direction == MovementDirection.BACKWARD


class TestControllerParamsFromDict:
    def test_basic_fields(self):
        p = ControllerParams.from_dict({"max_linear_vel": 3.0, "navigation_2d": True})
        assert p.max_linear_vel == 3.0
        assert p.navigation_2d is True
        # Unspecified fields remain None
        assert p.max_angular_vel is None

    def test_unknown_keys_are_ignored(self):
        # wheel_separation is differential-only — must not crash
        p = ControllerParams.from_dict({"max_linear_vel": 1.0, "wheel_separation": 0.5, "type": "differential"})
        assert p.max_linear_vel == 1.0
        assert not hasattr(p, "wheel_separation")

    def test_direction_string_in_dict(self):
        # from_dict coerces string → enum for ergonomic YAML configs.
        p = ControllerParams.from_dict({"default_direction": "backward"})
        assert p.default_direction == MovementDirection.BACKWARD

    def test_invalid_direction_string_raises(self):
        with pytest.raises(ValueError):
            ControllerParams.from_dict({"default_direction": "sideways"})

    def test_empty_dict_yields_defaults(self):
        p = ControllerParams.from_dict({})
        assert p == ControllerParams()
