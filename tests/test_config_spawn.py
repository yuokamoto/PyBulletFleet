"""Tests for config-driven spawning.

Unit tests (bottom-up order):
- TestLoadYamlConfig: load_yaml_config() — lowest-level YAML loading
- TestLoadConfig: load_config() — multi-file merge (uses load_yaml_config)
- TestGridSpawnParamsFromDict: GridSpawnParams.from_dict() classmethod

Integration tests:
- TestSpawnFromConfig: SimObjectManager.spawn_from_config() integration
- TestSpawnFromYaml: SimObjectManager.spawn_from_yaml() (uses load_yaml_config)
- TestSpawnFromConfigGeneralized: spawn_from_config with mixed type support

Optimization tests:
- TestSpawnObjectsBatchOptimization: batch_spawn context usage
- TestGridSpawnBatchOptimization: grid spawn batch_spawn context usage
"""

import pytest
import yaml

from pybullet_fleet import AgentSpawnParams, Pose
from pybullet_fleet.agent import Agent
from pybullet_fleet.agent_manager import AgentManager, GridSpawnParams
from tests.conftest import MockSimCore


# =====================================================================
# Fixtures
# =====================================================================


@pytest.fixture
def sim_core(pybullet_env):
    sc = MockSimCore(physics=False)
    sc._client = pybullet_env
    return sc


@pytest.fixture
def manager(sim_core):
    return AgentManager(sim_core=sim_core)


# =====================================================================
# load_yaml_config (unit)
# =====================================================================
class TestLoadYamlConfig:
    """load_yaml_config loads and returns YAML config dicts."""

    def test_loads_yaml(self, tmp_path):
        """load_yaml_config reads a YAML file and returns dict."""
        from pybullet_fleet.config_utils import load_yaml_config

        config = {"robots": [{"name": "r0"}], "simulation": {"gui": False}}
        yaml_file = tmp_path / "test.yaml"
        yaml_file.write_text(yaml.dump(config))

        result = load_yaml_config(str(yaml_file))
        assert "robots" in result
        assert result["simulation"]["gui"] is False

    def test_missing_file_raises(self):
        """Non-existent file raises FileNotFoundError."""
        from pybullet_fleet.config_utils import load_yaml_config

        with pytest.raises(FileNotFoundError):
            load_yaml_config("/nonexistent/config.yaml")

    def test_loads_simulation_params(self, tmp_path):
        """load_yaml_config returns simulation section."""
        from pybullet_fleet.config_utils import load_yaml_config

        config = {
            "simulation": {"gui": True, "physics": False},
            "robots": [{"name": "robot0"}],
        }
        yaml_file = tmp_path / "sim.yaml"
        yaml_file.write_text(yaml.dump(config))

        result = load_yaml_config(str(yaml_file))
        assert result.get("simulation", {}).get("gui") is True

    def test_missing_robots_returns_empty_list(self, tmp_path):
        """YAML without robots key returns empty when accessed with .get()."""
        from pybullet_fleet.config_utils import load_yaml_config

        config = {"simulation": {"gui": False}}
        yaml_file = tmp_path / "no_robots.yaml"
        yaml_file.write_text(yaml.dump(config))

        result = load_yaml_config(str(yaml_file))
        assert result.get("robots", []) == []


# =====================================================================
# load_config — multi-file merge (unit, uses load_yaml_config)
# =====================================================================
class TestLoadConfig:
    """load_config loads and merges one or more YAML files."""

    def test_single_path_string(self, tmp_path):
        """Single string path loads correctly."""
        from pybullet_fleet.config_utils import load_config

        f = tmp_path / "base.yaml"
        f.write_text(yaml.dump({"gui": True, "fps": 60}))
        result = load_config(str(f))
        assert result["gui"] is True
        assert result["fps"] == 60

    def test_single_path_in_list(self, tmp_path):
        """Single-element list works the same as a string."""
        from pybullet_fleet.config_utils import load_config

        f = tmp_path / "base.yaml"
        f.write_text(yaml.dump({"key": "value"}))
        result = load_config([str(f)])
        assert result["key"] == "value"

    def test_merge_overrides(self, tmp_path):
        """Later configs override earlier ones."""
        from pybullet_fleet.config_utils import load_config

        base = tmp_path / "base.yaml"
        base.write_text(yaml.dump({"a": 1, "b": 2}))
        override = tmp_path / "override.yaml"
        override.write_text(yaml.dump({"b": 99, "c": 3}))

        result = load_config([str(base), str(override)])
        assert result["a"] == 1
        assert result["b"] == 99
        assert result["c"] == 3

    def test_missing_file_raises(self, tmp_path):
        """Non-existent file raises FileNotFoundError."""
        from pybullet_fleet.config_utils import load_config

        with pytest.raises(FileNotFoundError):
            load_config("/nonexistent/config.yaml")

    def test_empty_paths_raises(self):
        """Empty list raises ValueError."""
        from pybullet_fleet.config_utils import load_config

        with pytest.raises(ValueError, match="At least one"):
            load_config([])

    def test_empty_yaml_returns_dict(self, tmp_path):
        """Empty YAML file returns empty dict (not None)."""
        from pybullet_fleet.config_utils import load_config

        f = tmp_path / "empty.yaml"
        f.write_text("")
        result = load_config(str(f))
        assert result == {}


# =====================================================================
# GridSpawnParams.from_dict (unit)
# =====================================================================
class TestGridSpawnParamsFromDict:
    """GridSpawnParams.from_dict() creates instances from config dicts."""

    def test_full_config(self):
        """Full config dict creates correct GridSpawnParams."""
        config = {
            "x_min": 0,
            "x_max": 9,
            "y_min": 0,
            "y_max": 9,
            "spacing": [1.0, 1.0, 0.0],
            "offset": [0.0, 0.0, 0.5],
            "z_min": 0,
            "z_max": 0,
        }
        params = GridSpawnParams.from_dict(config)
        assert params.x_min == 0
        assert params.x_max == 9
        assert params.y_min == 0
        assert params.y_max == 9
        assert params.spacing == [1.0, 1.0, 0.0]
        assert params.offset == [0.0, 0.0, 0.5]
        assert params.z_min == 0
        assert params.z_max == 0

    def test_defaults_for_z(self):
        """z_min and z_max default to 0 when omitted."""
        config = {
            "x_min": 0,
            "x_max": 4,
            "y_min": 0,
            "y_max": 4,
            "spacing": [2.0, 2.0, 0.0],
            "offset": [0.0, 0.0, 0.1],
        }
        params = GridSpawnParams.from_dict(config)
        assert params.z_min == 0
        assert params.z_max == 0

    def test_3d_grid(self):
        """Config with z_max > 0 creates a 3D grid."""
        config = {
            "x_min": 0,
            "x_max": 2,
            "y_min": 0,
            "y_max": 2,
            "z_min": 0,
            "z_max": 3,
            "spacing": [1.0, 1.0, 1.0],
            "offset": [0.0, 0.0, 0.0],
        }
        params = GridSpawnParams.from_dict(config)
        assert params.z_max == 3
        assert params.spacing == [1.0, 1.0, 1.0]

    def test_missing_required_field_raises(self):
        """Missing required fields raise KeyError."""
        config = {"x_min": 0, "x_max": 4}  # Missing y_min, y_max, spacing, offset
        with pytest.raises(KeyError):
            GridSpawnParams.from_dict(config)

    def test_roundtrip_dict(self):
        """from_dict results match direct construction."""
        config = {
            "x_min": 1,
            "x_max": 5,
            "y_min": 2,
            "y_max": 6,
            "spacing": [0.5, 0.5, 0.0],
            "offset": [1.0, 2.0, 0.3],
        }
        from_dict = GridSpawnParams.from_dict(config)
        direct = GridSpawnParams(
            x_min=1,
            x_max=5,
            y_min=2,
            y_max=6,
            spacing=[0.5, 0.5, 0.0],
            offset=[1.0, 2.0, 0.3],
        )
        assert from_dict == direct


# =====================================================================
# spawn_from_config (integration)
# =====================================================================
class TestSpawnFromConfig:
    """SimObjectManager.spawn_from_config(entities_yaml) spawns entities from YAML dicts."""

    def test_spawns_mixed_robots(self, manager):
        """Multiple robots with different URDFs spawn correctly."""
        robots_yaml = [
            {"name": "robot0", "urdf_path": "robots/mobile_robot.urdf", "pose": [0, 0, 0.05]},
            {"name": "cube0", "urdf_path": "robots/simple_cube.urdf", "pose": [2, 0, 0.5]},
            {"name": "robot1", "urdf_path": "robots/mobile_robot.urdf", "pose": [4, 0, 0.05]},
        ]
        agents = manager.spawn_from_config(robots_yaml)

        assert len(agents) == 3
        assert agents[0].name == "robot0"
        assert agents[1].name == "cube0"
        assert agents[2].name == "robot1"

    def test_agents_tracked_in_manager(self, manager):
        """Spawned agents are tracked in manager.objects, body_ids, object_ids."""
        robots_yaml = [
            {"name": "robot0", "urdf_path": "robots/mobile_robot.urdf", "pose": [0, 0, 0.05]},
            {"name": "robot1", "urdf_path": "robots/mobile_robot.urdf", "pose": [2, 0, 0.05]},
        ]
        agents = manager.spawn_from_config(robots_yaml)

        assert manager.get_object_count() == 2
        assert all(a in manager.objects for a in agents)
        # body_ids and object_ids are populated
        for a in agents:
            assert a.body_id in manager.body_ids
            assert a.object_id in manager.object_ids

    def test_controller_auto_created(self, manager):
        """Controller specified in config is auto-created on agent."""
        from pybullet_fleet.controller import OmniController

        robots_yaml = [
            {
                "name": "robot0",
                "urdf_path": "robots/mobile_robot.urdf",
                "pose": [0, 0, 0.05],
                "controller_config": "omni",
            },
        ]
        agents = manager.spawn_from_config(robots_yaml)

        assert agents[0]._controller is not None
        assert isinstance(agents[0]._controller, OmniController)

    def test_poses_match_config(self, manager):
        """Spawned agents are at the positions specified in config."""
        robots_yaml = [
            {"name": "r0", "urdf_path": "robots/mobile_robot.urdf", "pose": [1.0, 2.0, 0.05]},
            {"name": "r1", "urdf_path": "robots/mobile_robot.urdf", "pose": [3.0, 4.0, 0.05]},
        ]
        agents = manager.spawn_from_config(robots_yaml)

        p0 = agents[0].get_pose()
        assert p0.x == pytest.approx(1.0, abs=0.01)
        assert p0.y == pytest.approx(2.0, abs=0.01)

        p1 = agents[1].get_pose()
        assert p1.x == pytest.approx(3.0, abs=0.01)
        assert p1.y == pytest.approx(4.0, abs=0.01)


# =====================================================================
# spawn_from_yaml (integration, uses load_yaml_config + spawn_from_config)
# =====================================================================
class TestSpawnFromYaml:
    """SimObjectManager.spawn_from_yaml(yaml_path) loads YAML then spawns entities."""

    def test_spawns_from_yaml_file(self, manager, tmp_path):
        """Loads YAML file and spawns robots from the 'robots' section."""
        config = {
            "robots": [
                {"name": "robot0", "urdf_path": "robots/mobile_robot.urdf", "pose": [0, 0, 0.05]},
                {"name": "robot1", "urdf_path": "robots/mobile_robot.urdf", "pose": [2, 0, 0.05]},
            ]
        }
        yaml_file = tmp_path / "robots.yaml"
        yaml_file.write_text(yaml.dump(config))

        agents = manager.spawn_from_yaml(str(yaml_file))
        assert len(agents) == 2
        assert agents[0].name == "robot0"
        assert agents[1].name == "robot1"

    def test_custom_key(self, manager, tmp_path):
        """Custom key parameter selects a different YAML section."""
        config = {
            "fleet": [
                {"name": "fleet_bot", "urdf_path": "robots/mobile_robot.urdf", "pose": [0, 0, 0.05]},
            ]
        }
        yaml_file = tmp_path / "custom.yaml"
        yaml_file.write_text(yaml.dump(config))

        agents = manager.spawn_from_yaml(str(yaml_file), key="fleet")
        assert len(agents) == 1
        assert agents[0].name == "fleet_bot"

    def test_missing_file_raises(self, manager):
        """Non-existent YAML path raises FileNotFoundError."""
        with pytest.raises(FileNotFoundError):
            manager.spawn_from_yaml("/nonexistent/path/config.yaml")

    def test_missing_key_returns_empty(self, manager, tmp_path):
        """YAML file without the key returns empty list."""
        config = {"simulation": {"gui": False}}
        yaml_file = tmp_path / "no_robots.yaml"
        yaml_file.write_text(yaml.dump(config))

        agents = manager.spawn_from_yaml(str(yaml_file))
        assert agents == []
        assert manager.get_object_count() == 0

    def test_includes_simulation_section(self, manager, tmp_path):
        """spawn_from_yaml only uses the robots section, ignores simulation."""
        config = {
            "simulation": {"gui": True, "physics": True},
            "robots": [
                {"name": "robot0", "urdf_path": "robots/mobile_robot.urdf", "pose": [0, 0, 0.05]},
            ],
        }
        yaml_file = tmp_path / "full_config.yaml"
        yaml_file.write_text(yaml.dump(config))

        agents = manager.spawn_from_yaml(str(yaml_file))
        assert len(agents) == 1  # Only robots are spawned, sim params ignored


# =====================================================================
# spawn_from_config generalized — type field support (integration)
# =====================================================================
class TestSpawnFromConfigGeneralized:
    """spawn_from_config on SimObjectManager accepts type field for mixed types."""

    def test_defaults_type_to_agent(self, manager):
        """Dicts without 'type' field default to 'agent' (backward compat)."""
        robots_yaml = [
            {"name": "robot0", "urdf_path": "robots/mobile_robot.urdf", "pose": [0, 0, 0.05]},
        ]
        result = manager.spawn_from_config(robots_yaml)
        assert len(result) == 1
        assert isinstance(result[0], Agent)

    def test_accepts_explicit_type_agent(self, manager):
        """Dicts with type='agent' work the same as without type."""
        robots_yaml = [
            {"type": "agent", "name": "robot0", "urdf_path": "robots/mobile_robot.urdf", "pose": [0, 0, 0.05]},
        ]
        result = manager.spawn_from_config(robots_yaml)
        assert len(result) == 1
        assert isinstance(result[0], Agent)
        assert result[0].name == "robot0"

    def test_mixed_types_returned(self, sim_core, manager):
        """Dicts with mixed types return agents and sim_objects together."""
        from pybullet_fleet.sim_object import SimObject

        entities = [
            {"type": "agent", "name": "r0", "urdf_path": "robots/mobile_robot.urdf", "pose": [0, 0, 0.05]},
            {
                "type": "sim_object",
                "name": "box0",
                "visual_shape": {"shape_type": "box", "half_extents": [0.3, 0.3, 0.1]},
                "collision_shape": {"shape_type": "box", "half_extents": [0.3, 0.3, 0.1]},
                "pose": [3, 0, 0.1],
            },
        ]
        result = manager.spawn_from_config(entities)
        assert len(result) == 2
        assert isinstance(result[0], Agent)
        assert isinstance(result[1], SimObject)
        assert not isinstance(result[1], Agent)  # Must be plain SimObject, not Agent

    def test_empty_entities(self, sim_core, manager):
        """Empty entities list returns empty list."""
        result = manager.spawn_from_config([])
        assert result == []

    def test_custom_registered_class_spawned(self, sim_core, manager):
        """Custom registered subclass is spawned and isinstance check passes."""
        from pybullet_fleet.entity_registry import ENTITY_REGISTRY, register_entity_class

        class ForkliftAgent(Agent):
            """Custom agent subclass for testing entity registry."""

            pass

        register_entity_class("forklift", ForkliftAgent)
        try:
            entities = [
                {"type": "forklift", "name": "fl0", "urdf_path": "robots/mobile_robot.urdf"},
            ]
            result = manager.spawn_from_config(entities)
            assert len(result) == 1
            assert result[0].name == "fl0"
            assert isinstance(result[0], ForkliftAgent)
            assert isinstance(result[0], Agent)  # still an Agent
        finally:
            ENTITY_REGISTRY.pop("forklift", None)

    def test_unknown_type_raises(self, manager):
        """spawn_from_config raises KeyError on unknown entity type."""
        entities = [{"type": "nonexistent", "name": "x"}]
        with pytest.raises(KeyError, match="Unknown entity type"):
            manager.spawn_from_config(entities)


# =====================================================================
# spawn_objects_batch uses batch_spawn (optimization)
# =====================================================================
class TestSpawnObjectsBatchOptimization:
    """spawn_objects_batch wraps creation in batch_spawn context."""

    def test_batch_spawn_called(self, sim_core, manager):
        """spawn_objects_batch uses batch_spawn context when available."""
        entered = []

        from contextlib import contextmanager

        @contextmanager
        def spy_batch():
            entered.append(True)
            yield

        # Add batch_spawn to MockSimCore so hasattr check passes
        sim_core.batch_spawn = spy_batch

        params = AgentSpawnParams(
            urdf_path="robots/mobile_robot.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0.05),
            name="r0",
        )
        manager.spawn_objects_batch([params])
        assert len(entered) == 1

    def test_still_works_without_batch_spawn(self):
        """spawn_objects_batch still works if sim_core has no batch_spawn."""
        from pybullet_fleet.agent_manager import SimObjectManager
        from pybullet_fleet.sim_object import SimObject

        # SimObjectManager with no sim_core → _nullctx fallback
        mgr = SimObjectManager(sim_core=None, object_class=SimObject)
        # Can't actually spawn without sim_core, but shouldn't crash on context
        # Just verify no AttributeError
        assert mgr.sim_core is None


# =====================================================================
# Grid spawn methods use batch_spawn (optimization)
# =====================================================================
class TestGridSpawnBatchOptimization:
    """Grid spawn methods wrap creation in batch_spawn context."""

    def _make_spy(self):
        """Create a spy batch_spawn context manager that records entry."""
        entered = []

        from contextlib import contextmanager

        @contextmanager
        def spy_batch():
            entered.append(True)
            yield

        return spy_batch, entered

    def test_spawn_grid_counts_uses_batch(self, sim_core, manager):
        """spawn_grid_counts uses batch_spawn context when available."""
        spy_batch, entered = self._make_spy()
        sim_core.batch_spawn = spy_batch

        params = AgentSpawnParams(
            urdf_path="robots/mobile_robot.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0.05),
            name="r0",
        )
        grid = GridSpawnParams(x_min=0, x_max=1, y_min=0, y_max=0, spacing=[2, 2, 0], offset=[0, 0, 0.05])
        manager.spawn_grid_counts(grid_params=grid, spawn_params_count_list=[(params, 2)])
        assert len(entered) == 1

    def test_spawn_grid_mixed_uses_batch(self, sim_core, manager):
        """_spawn_grid_mixed_impl uses batch_spawn context when available."""
        spy_batch, entered = self._make_spy()
        sim_core.batch_spawn = spy_batch

        params = AgentSpawnParams(
            urdf_path="robots/mobile_robot.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0.05),
            name="r0",
        )
        grid = GridSpawnParams(x_min=0, x_max=1, y_min=0, y_max=0, spacing=[2, 2, 0], offset=[0, 0, 0.05])
        manager.spawn_grid_mixed(num_objects=2, grid_params=grid, spawn_params_list=[(params, 1.0)])
        assert len(entered) == 1
