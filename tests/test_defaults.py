"""Tests for pybullet_fleet._defaults — centralized default values."""

from dataclasses import fields

import pytest


@pytest.fixture(autouse=True)
def _restore_defaults():
    """Save and restore _DEFAULTS around every test so env-override tests don't leak."""
    import pybullet_fleet._defaults as mod

    snapshot = mod._snapshot()
    yield
    mod._restore(snapshot)


class TestDefaultsCompleteness:
    """Verify _defaults has entries for every non-mutable dataclass field."""

    def test_simulation_params_fields_covered(self):
        from pybullet_fleet._defaults import SIMULATION
        from pybullet_fleet.core_simulation import SimulationParams

        # Enum-typed fields (stored as string in _defaults, enum in dataclass)
        skip = {
            "collision_detection_method",
            "spatial_hash_cell_size_mode",
        }
        for f in fields(SimulationParams):
            if f.name not in skip:
                assert f.name in SIMULATION, f"Missing default for SimulationParams.{f.name}"

    def test_agent_params_fields_covered(self):
        from pybullet_fleet._defaults import AGENT
        from pybullet_fleet.agent import AgentSpawnParams
        from pybullet_fleet.sim_object import SimObjectSpawnParams

        # Skip inherited SimObjectSpawnParams fields (covered by sim_object test)
        # and agent-specific fields that are None/enum/complex types
        inherited = {f.name for f in fields(SimObjectSpawnParams)}
        skip = inherited | {"urdf_path", "ik_params", "controller_config"}
        for f in fields(AgentSpawnParams):
            if f.name not in skip:
                assert f.name in AGENT, f"Missing default for AgentSpawnParams.{f.name}"

    def test_sim_object_params_fields_covered(self):
        from pybullet_fleet._defaults import SIM_OBJECT
        from pybullet_fleet.sim_object import SimObjectSpawnParams

        skip = {
            "visual_shape",
            "collision_shape",
            "initial_pose",
            "name",
            "visual_frame_pose",
            "collision_frame_pose",
            "user_data",
        }
        for f in fields(SimObjectSpawnParams):
            if f.name not in skip:
                assert f.name in SIM_OBJECT, f"Missing default for SimObjectSpawnParams.{f.name}"


class TestEnvOverride:
    """Test PBF_* environment variable overrides."""

    def _reload(self):
        """Force re-evaluation of env overrides."""
        import pybullet_fleet._defaults as mod

        mod.reload_defaults()
        return mod

    def test_float_override(self, monkeypatch):
        monkeypatch.setenv("PBF_SIMULATION_TIMESTEP", "0.05")
        mod = self._reload()
        assert mod.SIMULATION["timestep"] == 0.05

    def test_bool_override_false(self, monkeypatch):
        monkeypatch.setenv("PBF_SIMULATION_GUI", "false")
        mod = self._reload()
        assert mod.SIMULATION["gui"] is False

    def test_bool_override_true(self, monkeypatch):
        monkeypatch.setenv("PBF_SIMULATION_GUI", "true")
        mod = self._reload()
        assert mod.SIMULATION["gui"] is True

    def test_int_override(self, monkeypatch):
        monkeypatch.setenv("PBF_SIMULATION_MONITOR_WIDTH", "300")
        mod = self._reload()
        assert mod.SIMULATION["monitor_width"] == 300

    def test_string_override(self, monkeypatch):
        monkeypatch.setenv("PBF_SIMULATION_LOG_LEVEL", "debug")
        mod = self._reload()
        assert mod.SIMULATION["log_level"] == "debug"

    def test_none_fields_not_overridable(self, monkeypatch):
        """Fields with None default should not be affected by env vars."""
        monkeypatch.setenv("PBF_SIMULATION_COLLISION_CHECK_FREQUENCY", "10")
        mod = self._reload()
        # collision_check_frequency is None in _DEFAULTS, so skipped by _apply_env_overrides
        assert mod.SIMULATION["collision_check_frequency"] is None


class TestConsistency:
    """Verify all initialization paths produce identical defaults."""

    def test_dataclass_matches_from_dict(self):
        """SimulationParams() and SimulationParams.from_dict({}) must match."""
        from pybullet_fleet.core_simulation import SimulationParams

        direct = SimulationParams(gui=False, monitor=False)
        from_dict = SimulationParams.from_dict({"gui": False, "monitor": False})
        for f in fields(SimulationParams):
            val_direct = getattr(direct, f.name)
            val_from_dict = getattr(from_dict, f.name)
            assert (
                val_direct == val_from_dict
            ), f"SimulationParams.{f.name}: direct={val_direct!r} vs from_dict={val_from_dict!r}"

    def test_yaml_overrides_env(self, monkeypatch):
        """YAML explicit value must take priority over env var."""
        monkeypatch.setenv("PBF_SIMULATION_TIMESTEP", "0.05")
        import pybullet_fleet._defaults as mod

        mod.reload_defaults()
        from pybullet_fleet.core_simulation import SimulationParams

        params = SimulationParams.from_dict({"gui": False, "monitor": False, "timestep": 0.01})
        assert params.timestep == 0.01  # YAML wins

    def test_timestep_unified(self):
        """The critical mismatch is fixed: dataclass and from_dict agree."""
        from pybullet_fleet.core_simulation import SimulationParams

        direct = SimulationParams(gui=False, monitor=False)
        from_dict = SimulationParams.from_dict({"gui": False, "monitor": False})
        assert direct.timestep == pytest.approx(0.1)
        assert from_dict.timestep == pytest.approx(0.1)

    def test_motion_mode_from_dict_uses_defaults(self):
        """AgentSpawnParams.from_dict({}) should use _AGT_D for motion_mode."""
        from pybullet_fleet._defaults import AGENT
        from pybullet_fleet.agent import AgentSpawnParams
        from pybullet_fleet.types import MotionMode

        params = AgentSpawnParams.from_dict({"name": "test"})
        assert params.motion_mode == MotionMode(AGENT["motion_mode"])


class TestIKDefaults:
    """Verify IKParams scalar defaults are centralized in _defaults."""

    def test_ik_section_exists(self):
        from pybullet_fleet._defaults import _DEFAULTS

        assert "ik" in _DEFAULTS, "Missing 'ik' section in _DEFAULTS"

    def test_ik_scalar_fields_covered(self):
        """All scalar IKParams fields should have entries in _defaults['ik']."""
        from pybullet_fleet._defaults import _DEFAULTS

        ik_defaults = _DEFAULTS["ik"]
        expected = {
            "max_outer_iterations",
            "convergence_threshold",
            "max_inner_iterations",
            "residual_threshold",
            "reachability_tolerance",
        }
        for key in expected:
            assert key in ik_defaults, f"Missing default for IKParams.{key}"

    def test_ik_defaults_match_dataclass(self):
        """IKParams() defaults must match _defaults['ik'] values."""
        from pybullet_fleet._defaults import _DEFAULTS
        from pybullet_fleet.agent import IKParams

        ik_defaults = _DEFAULTS["ik"]
        params = IKParams()
        assert params.max_outer_iterations == ik_defaults["max_outer_iterations"]
        assert params.convergence_threshold == ik_defaults["convergence_threshold"]
        assert params.max_inner_iterations == ik_defaults["max_inner_iterations"]
        assert params.residual_threshold == ik_defaults["residual_threshold"]
        assert params.reachability_tolerance == ik_defaults["reachability_tolerance"]

    def test_ik_env_override(self, monkeypatch):
        """IK scalar fields should be env-overridable."""
        monkeypatch.setenv("PBF_IK_MAX_OUTER_ITERATIONS", "10")
        import pybullet_fleet._defaults as mod

        mod.reload_defaults()
        assert mod._DEFAULTS["ik"]["max_outer_iterations"] == 10


class TestNoneSentinelDocumentation:
    """Verify None sentinel fields are documented in _defaults['simulation']."""

    def test_none_sentinels_present(self):
        """None sentinel fields should be listed in simulation defaults."""
        from pybullet_fleet._defaults import NONE_SENTINEL_KEYS, SIMULATION

        assert len(NONE_SENTINEL_KEYS) >= 5, "Expected at least 5 None sentinel keys"
        for field in NONE_SENTINEL_KEYS:
            assert field in SIMULATION, f"Missing sentinel doc for {field}"
            assert SIMULATION[field] is None, f"{field} should be None"

    def test_none_fields_still_not_env_overridable(self, monkeypatch):
        """Even though None sentinels are in _DEFAULTS, they remain not env-overridable."""
        from pybullet_fleet._defaults import NONE_SENTINEL_KEYS

        for key in NONE_SENTINEL_KEYS:
            monkeypatch.setenv(f"PBF_SIMULATION_{key.upper()}", "999")
        import pybullet_fleet._defaults as mod

        mod.reload_defaults()
        for key in NONE_SENTINEL_KEYS:
            assert mod.SIMULATION[key] is None, f"{key} should remain None after env override"
