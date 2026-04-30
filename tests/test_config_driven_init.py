"""Tests for config-driven world loading, robot spawning, and auto enable_floor.

Covers:
- MultiRobotSimulationCore.from_yaml() with nested config (simulation/world/robots)
- MultiRobotSimulationCore.from_dict() with nested config
- load_world() method
- spawn_robots_from_config() method
- Auto-disable enable_floor when world config is present
- Backward compatibility with flat config YAML
"""

import contextlib
import os
import tempfile

import pybullet as p
import pytest
import yaml

from pybullet_fleet.agent import Agent
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams


# ---------------------------------------------------------------------------
# Constants & helpers
# ---------------------------------------------------------------------------

_HEADLESS_SIM = {"gui": False, "physics": False, "monitor": False, "log_level": "warning"}
"""Minimal headless simulation config reused across tests."""

_MOBILE_URDF = "robots/mobile_robot.urdf"
_DEFAULT_POSE = [0, 0, 0.05]


def _robot(name=None, *, urdf=_MOBILE_URDF, pose=_DEFAULT_POSE, **kwargs):
    """Build a single robot config dict."""
    d = {"urdf_path": urdf}
    if name:
        d["name"] = name
    if pose is not None:
        d["pose"] = list(pose)
    d.update(kwargs)
    return d


def _spawn_params(**kwargs):
    """Build AgentSpawnParams with sensible defaults."""
    from pybullet_fleet.agent import AgentSpawnParams
    from pybullet_fleet.geometry import Pose

    kwargs.setdefault("urdf_path", _MOBILE_URDF)
    kwargs.setdefault("initial_pose", Pose.from_xyz(0, 0, 0.05))
    return AgentSpawnParams(**kwargs)


def _write_yaml(data: dict) -> str:
    """Write a dict to a temporary YAML file and return its path."""
    fd, path = tempfile.mkstemp(suffix=".yaml")
    with os.fdopen(fd, "w") as f:
        yaml.safe_dump(data, f)
    return path


@contextlib.contextmanager
def _yaml_file(data: dict):
    """Write config to a temp YAML, yield the path, then delete."""
    path = _write_yaml(data)
    try:
        yield path
    finally:
        os.unlink(path)


@contextlib.contextmanager
def _make_sim(config: dict, *, via: str = "dict"):
    """Create a sim, yield it, then disconnect.

    Args:
        via: "dict" uses from_dict(); "yaml" writes a temp file and uses from_yaml().
    """
    if via == "yaml":
        with _yaml_file(config) as path:
            sim = MultiRobotSimulationCore.from_yaml(path)
            try:
                yield sim
            finally:
                p.disconnect(sim.client)
    else:
        sim = MultiRobotSimulationCore.from_dict(config)
        try:
            yield sim
        finally:
            p.disconnect(sim.client)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def sim_core():
    """Headless sim for method-level tests."""
    params = SimulationParams(**_HEADLESS_SIM)
    sc = MultiRobotSimulationCore(params)
    sc.initialize_simulation()
    yield sc
    p.disconnect(sc.client)


# Shorthand for parametrizing the factory path.
_VIA = pytest.mark.parametrize("via", ["yaml", "dict"])


# ---------------------------------------------------------------------------
# from_yaml / from_dict — shared behaviour (parametrized)
# ---------------------------------------------------------------------------


class TestConfigFactory:
    """Behaviour shared by from_yaml() and from_dict()."""

    @_VIA
    def test_spawns_robots(self, via):
        """Robots listed in config are spawned automatically."""
        config = {
            "simulation": _HEADLESS_SIM,
            "entities": [_robot("r0")],
        }
        with _make_sim(config, via=via) as sim:
            assert len(sim.agents) == 1
            assert sim.agents[0].name == "r0"

    @_VIA
    def test_auto_disables_floor_when_world_present(self, via):
        """enable_floor defaults to False when world section is present."""
        config = {
            "simulation": _HEADLESS_SIM,
            "world": {"mesh_dir": "/nonexistent_dir_will_warn"},
        }
        with _make_sim(config, via=via) as sim:
            assert sim.params.enable_floor is False

    @_VIA
    def test_explicit_enable_floor_overrides_auto(self, via):
        """Explicit enable_floor=True is respected even with world section."""
        config = {
            "simulation": {**_HEADLESS_SIM, "enable_floor": True},
            "world": {"mesh_dir": "/nonexistent_dir"},
        }
        with _make_sim(config, via=via) as sim:
            assert sim.params.enable_floor is True

    @_VIA
    def test_no_world_keeps_floor_enabled(self, via):
        """Without world section, floor stays enabled by default."""
        with _make_sim({"simulation": _HEADLESS_SIM}, via=via) as sim:
            assert sim.params.enable_floor is True

    @_VIA
    def test_flat_config_backward_compatible(self, via):
        """Flat config without simulation/world/robots sections stays compatible."""
        with _make_sim(_HEADLESS_SIM, via=via) as sim:
            assert sim.params.gui is False
            assert sim.params.physics is False
            assert len(sim.agents) == 0

    @_VIA
    def test_spawns_robots_with_pose_and_motion_mode(self, via):
        """Robots get correct pose and motion_mode from config."""
        config = {
            "simulation": _HEADLESS_SIM,
            "entities": [
                _robot("bot0", motion_mode="omnidirectional"),
                _robot("bot1", pose=[2, 0, 0.05], motion_mode="differential"),
            ],
        }
        with _make_sim(config, via=via) as sim:
            assert len(sim.agents) == 2
            assert {a.name for a in sim.agents} == {"bot0", "bot1"}

            bot0 = next(a for a in sim.agents if a.name == "bot0")
            bot1 = next(a for a in sim.agents if a.name == "bot1")

            # Pose
            pos0 = bot0.get_pose().position
            assert abs(pos0[0] - 0.0) < 0.01
            assert abs(pos0[1] - 0.0) < 0.01
            assert abs(pos0[2] - 0.05) < 0.1

            pos1 = bot1.get_pose().position
            assert abs(pos1[0] - 2.0) < 0.01
            assert abs(pos1[1] - 0.0) < 0.01

            # Motion mode
            from pybullet_fleet.types import MotionMode

            assert bot0.motion_mode == MotionMode.OMNIDIRECTIONAL
            assert bot1.motion_mode == MotionMode.DIFFERENTIAL


# ---------------------------------------------------------------------------
# spawn_robots_from_config() method
# ---------------------------------------------------------------------------


class TestSpawnRobotsFromConfig:
    """Direct calls to spawn_robots_from_config()."""

    def test_spawn_from_list(self, sim_core):
        agents = sim_core.spawn_robots_from_config([_robot("a"), _robot("b", pose=[3, 0, 0.05])])
        assert len(agents) == 2
        assert all(isinstance(a, Agent) for a in agents)
        # Agents are registered in sim_core
        assert len(sim_core.agents) == 2

    def test_spawn_empty_list(self, sim_core):
        agents = sim_core.spawn_robots_from_config([])
        assert agents == []


# ---------------------------------------------------------------------------
# load_world() method
# ---------------------------------------------------------------------------


class TestLoadWorld:
    """Direct calls to load_world()."""

    def test_empty_config_returns_empty(self, sim_core):
        objs = sim_core.load_world({})
        assert objs == []

    def test_nonexistent_mesh_dir_returns_empty(self, sim_core):
        objs = sim_core.load_world({"mesh_dir": "/does/not/exist"})
        assert objs == []

    def test_mesh_dir_loads_obj_files(self, sim_core, tmp_path):
        """load_world with mesh_dir loads .obj files as SimObjects."""
        from pybullet_fleet.sim_object import SimObject

        # Minimal valid OBJ: a single triangle
        obj_content = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"
        (tmp_path / "wall.obj").write_text(obj_content)
        (tmp_path / "floor.obj").write_text(obj_content)
        # Non-obj file should be ignored
        (tmp_path / "readme.txt").write_text("ignored")

        objs = sim_core.load_world({"mesh_dir": str(tmp_path)})
        assert len(objs) == 2
        assert all(isinstance(o, SimObject) for o in objs)
        names = {o.name for o in objs}
        assert names == {"floor", "wall"}
        # Objects should be registered in sim_core
        assert len(sim_core.sim_objects) >= 2


# ---------------------------------------------------------------------------
# SimulationParams.from_config — nested support
# ---------------------------------------------------------------------------


class TestSimulationParamsFromConfig:
    """SimulationParams.from_config() handles nested layout."""

    def test_nested_simulation_section(self):
        config = {
            "simulation": {**_HEADLESS_SIM, "physics": True, "timestep": 0.01},
            "entities": [{"name": "r0"}],  # ignored by SimulationParams
        }
        with _yaml_file(config) as path:
            params = SimulationParams.from_config(path)
            assert params.gui is False
            assert params.physics is True
            assert params.timestep == 0.01

    def test_flat_config_still_works(self):
        config = {**_HEADLESS_SIM, "physics": True, "timestep": 0.01}
        with _yaml_file(config) as path:
            params = SimulationParams.from_config(path)
            assert params.physics is True
            assert params.timestep == 0.01


# ---------------------------------------------------------------------------
# Grid expansion in spawn_robots_from_config()
# ---------------------------------------------------------------------------


class TestGridExpansion:
    """spawn_robots_from_config() grid expansion."""

    def test_grid_spawns_correct_count(self, sim_core):
        """Grid key should expand to exactly count entities."""
        entities = sim_core.spawn_robots_from_config(
            [_robot(pose=None, motion_mode="omnidirectional", grid={"count": 4, "spacing": [3.0, 3.0]})]
        )
        assert len(entities) == 4
        assert len(sim_core.agents) == 4

    def test_grid_names_are_sequential(self, sim_core):
        """Grid-spawned entities get indexed names."""
        entities = sim_core.spawn_robots_from_config([_robot("bot", pose=None, grid={"count": 3, "spacing": [2, 2]})])
        names = [e.name for e in entities]
        assert names == ["bot_0", "bot_1", "bot_2"]

    def test_grid_positions_are_spaced(self, sim_core):
        """Grid-spawned entities are placed on a grid with correct spacing."""
        entities = sim_core.spawn_robots_from_config(
            [_robot(pose=None, grid={"count": 4, "spacing": [10.0, 20.0], "columns": 2})]
        )
        # 4 entities in 2 columns: (0,0), (1,0), (0,1), (1,1)
        positions = [e.get_pose().position for e in entities]
        # col=0,row=0 → x=0, y=0; col=1,row=0 → x=10, y=0
        # col=0,row=1 → x=0, y=20; col=1,row=1 → x=10, y=20
        assert abs(positions[0][0] - 0.0) < 0.01
        assert abs(positions[1][0] - 10.0) < 0.01
        assert abs(positions[2][1] - 20.0) < 0.01
        assert abs(positions[3][0] - 10.0) < 0.01
        assert abs(positions[3][1] - 20.0) < 0.01

    def test_grid_offset_shifts_origin(self, sim_core):
        """Grid offset shifts all entities."""
        entities = sim_core.spawn_robots_from_config(
            [_robot(pose=None, grid={"count": 1, "spacing": [5.0, 5.0], "offset": [100.0, 200.0, 0.05]})]
        )
        pos = entities[0].get_pose().position
        assert abs(pos[0] - 100.0) < 0.01
        assert abs(pos[1] - 200.0) < 0.01
        assert abs(pos[2] - 0.05) < 0.01

    def test_grid_mixed_with_individual(self, sim_core):
        """Grid and individual entries can be mixed in the same config."""
        entities = sim_core.spawn_robots_from_config(
            [
                _robot("solo"),
                _robot("fleet", pose=None, grid={"count": 3, "spacing": [2, 2]}),
            ]
        )
        assert len(entities) == 4
        assert entities[0].name == "solo"
        assert entities[1].name == "fleet_0"

    def test_grid_via_from_dict(self):
        """Grid spawning works end-to-end via from_dict."""
        config = {
            "simulation": _HEADLESS_SIM,
            "entities": [_robot(pose=None, grid={"count": 6, "spacing": [2, 2]})],
        }
        with _make_sim(config) as sim:
            assert len(sim.agents) == 6

    def test_grid_range_based(self, sim_core):
        """Range-based grid (x_min/x_max/y_min/y_max) works."""
        entities = sim_core.spawn_robots_from_config(
            [
                _robot(
                    pose=None,
                    grid={"x_min": 0, "x_max": 2, "y_min": 0, "y_max": 1, "spacing": [5.0, 5.0, 0.0], "offset": [0, 0, 0.0]},
                ),
            ]
        )
        # 3 columns x 2 rows = 6 entities
        assert len(entities) == 6

    def test_grid_range_positions(self, sim_core):
        """Range-based grid places entities at correct positions."""
        entities = sim_core.spawn_robots_from_config(
            [
                _robot(
                    pose=None,
                    grid={"x_min": 0, "x_max": 1, "y_min": 0, "y_max": 1, "spacing": [10.0, 20.0, 0.0], "offset": [0, 0, 0.0]},
                ),
            ]
        )
        positions = [e.get_pose().position for e in entities]
        # X→Y order: (0,0), (1,0), (0,1), (1,1)
        assert abs(positions[0][0] - 0.0) < 0.01
        assert abs(positions[1][0] - 10.0) < 0.01
        assert abs(positions[2][1] - 20.0) < 0.01
        assert abs(positions[3][0] - 10.0) < 0.01
        assert abs(positions[3][1] - 20.0) < 0.01

    def test_grid_registers_manager(self, sim_core):
        """Grid spawning auto-registers a SimObjectManager."""
        assert len(sim_core._registered_managers) == 0
        sim_core.spawn_robots_from_config([_robot(pose=None, grid={"count": 3, "spacing": [2, 2]})])
        assert len(sim_core._registered_managers) == 1
        mgr = sim_core._registered_managers[0]
        assert len(mgr.objects) == 3

    def test_grid_manager_syncs_on_remove(self, sim_core):
        """Removing a grid-spawned entity also removes it from the manager."""
        entities = sim_core.spawn_robots_from_config([_robot(pose=None, grid={"count": 3, "spacing": [2, 2]})])
        mgr = sim_core._registered_managers[0]
        assert len(mgr.objects) == 3
        sim_core.remove_object(entities[0])
        assert len(mgr.objects) == 2


# ---------------------------------------------------------------------------
# Type dispatch in spawn_robots_from_config()
# ---------------------------------------------------------------------------


class TestTypeDispatch:
    """spawn_robots_from_config() entity type dispatch via entity_registry."""

    def test_default_type_is_agent(self, sim_core):
        """When type is omitted, entities are spawned as Agents."""
        entities = sim_core.spawn_robots_from_config([_robot()])
        assert len(entities) == 1
        assert isinstance(entities[0], Agent)

    def test_explicit_agent_type(self, sim_core):
        """type: agent produces Agent instances."""
        entities = sim_core.spawn_robots_from_config([_robot(type="agent")])
        assert isinstance(entities[0], Agent)

    def test_sim_object_type(self, sim_core):
        """type: sim_object produces SimObject instances (not Agent)."""
        from pybullet_fleet.sim_object import SimObject

        entities = sim_core.spawn_robots_from_config([_robot(urdf="robots/simple_cube.urdf", type="sim_object")])
        assert len(entities) == 1
        assert type(entities[0]) is SimObject

    def test_unknown_type_raises(self, sim_core):
        """Unknown entity type raises KeyError with available types."""
        with pytest.raises(KeyError, match="Unknown entity type.*nonexistent"):
            sim_core.spawn_robots_from_config([_robot(pose=None, type="nonexistent")])

    def test_custom_registered_type(self, sim_core):
        """Custom entity types registered via entity_registry work."""
        from pybullet_fleet.entity_registry import ENTITY_REGISTRY

        # Register a custom type (just an alias for Agent in this test)
        ENTITY_REGISTRY["test_forklift"] = Agent
        try:
            entities = sim_core.spawn_robots_from_config([_robot(type="test_forklift")])
            assert isinstance(entities[0], Agent)
        finally:
            del ENTITY_REGISTRY["test_forklift"]

    def test_type_with_grid(self, sim_core):
        """type dispatch works together with grid expansion."""
        entities = sim_core.spawn_robots_from_config([_robot(pose=None, type="agent", grid={"count": 3, "spacing": [2, 2]})])
        assert len(entities) == 3
        assert all(isinstance(e, Agent) for e in entities)


# ---------------------------------------------------------------------------
# Manager registration & sync
# ---------------------------------------------------------------------------


class TestManagerRegistration:
    """register_manager / unregister_manager and auto-sync."""

    def test_register_manager(self, sim_core):
        """register_manager adds a manager and sets sim_core."""
        from pybullet_fleet.agent_manager import AgentManager

        mgr = AgentManager()
        sim_core.register_manager(mgr)
        assert mgr.sim_core is sim_core
        assert mgr in sim_core._registered_managers

    def test_register_idempotent(self, sim_core):
        """Registering the same manager twice is a no-op."""
        from pybullet_fleet.agent_manager import AgentManager

        mgr = AgentManager()
        sim_core.register_manager(mgr)
        sim_core.register_manager(mgr)
        assert sim_core._registered_managers.count(mgr) == 1

    def test_unregister_manager(self, sim_core):
        """unregister_manager removes the manager."""
        from pybullet_fleet.agent_manager import AgentManager

        mgr = AgentManager()
        sim_core.register_manager(mgr)
        assert sim_core.unregister_manager(mgr) is True
        assert mgr not in sim_core._registered_managers

    def test_unregister_unknown_returns_false(self, sim_core):
        """Unregistering a never-registered manager returns False."""
        from pybullet_fleet.agent_manager import AgentManager

        mgr = AgentManager()
        assert sim_core.unregister_manager(mgr) is False


class TestManagerSyncOnRemove:
    """Removing objects from sim_core auto-syncs registered managers."""

    def test_remove_object_syncs_manager(self, sim_core):
        """When sim_core.remove_object() is called, the manager is synced."""
        from pybullet_fleet.agent_manager import AgentManager

        mgr = AgentManager()
        sim_core.register_manager(mgr)

        # Spawn via manager
        sp = _spawn_params()
        agents = mgr.spawn_objects_batch([sp])
        assert len(mgr.objects) == 1
        assert len(sim_core.agents) == 1

        agent = agents[0]
        sim_core.remove_object(agent)

        # Manager should be synced
        assert len(mgr.objects) == 0
        assert agent.object_id not in mgr.object_ids
        assert agent.body_id not in mgr.body_ids
        # sim_core should also be cleared
        assert len(sim_core.agents) == 0

    def test_remove_only_syncs_registered_managers(self, sim_core):
        """An unregistered manager is not synced on remove_object."""
        from pybullet_fleet.agent_manager import AgentManager

        mgr_registered = AgentManager()
        mgr_unregistered = AgentManager(sim_core=sim_core)
        sim_core.register_manager(mgr_registered)

        sp = _spawn_params()
        # Spawn through both managers
        agents_reg = mgr_registered.spawn_objects_batch([sp])
        agents_unreg = mgr_unregistered.spawn_objects_batch([sp])

        # Remove the registered one's agent
        sim_core.remove_object(agents_reg[0])
        assert len(mgr_registered.objects) == 0

        # Remove the unregistered one's agent — manager NOT synced
        sim_core.remove_object(agents_unreg[0])
        assert len(mgr_unregistered.objects) == 1  # still tracked (stale)

    def test_remove_nonexistent_in_manager_is_safe(self, sim_core):
        """Removing an object not tracked by the manager doesn't crash."""
        from pybullet_fleet.agent_manager import AgentManager

        mgr = AgentManager()
        sim_core.register_manager(mgr)

        # Spawn directly (not through manager)
        sp = _spawn_params()
        agent = Agent.from_params(sp, sim_core=sim_core)
        assert agent.object_id not in mgr.object_ids

        # Should not raise
        sim_core.remove_object(agent)
        assert len(sim_core.agents) == 0


class TestManagerSyncOnReset:
    """sim_core.reset() clears registered managers."""

    def test_reset_clears_managers(self, sim_core):
        """reset() calls clear() on all registered managers."""
        from pybullet_fleet.agent_manager import AgentManager

        mgr = AgentManager()
        sim_core.register_manager(mgr)

        sp = _spawn_params()
        mgr.spawn_objects_batch([sp, sp])
        assert len(mgr.objects) == 2

        sim_core.reset()

        assert len(mgr.objects) == 0
        assert len(mgr.object_ids) == 0
        assert len(mgr.body_ids) == 0
        # Manager is still registered after reset
        assert mgr in sim_core._registered_managers


class TestSimObjectManagerRemoveAndClear:
    """Unit tests for SimObjectManager.remove_object() and clear()."""

    def test_remove_object_from_manager(self, sim_core):
        """remove_object removes from objects/body_ids/object_ids."""
        from pybullet_fleet.agent_manager import SimObjectManager
        from pybullet_fleet.sim_object import SimObjectSpawnParams, ShapeParams
        from pybullet_fleet.geometry import Pose

        mgr = SimObjectManager(sim_core=sim_core)
        sp = SimObjectSpawnParams(
            visual_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            initial_pose=Pose.from_xyz(0, 0, 0.05),
            name="cube0",
        )
        objs = mgr.spawn_objects_batch([sp])
        obj = objs[0]

        result = mgr.remove_object(obj)
        assert result is True
        assert obj not in mgr.objects
        assert obj.object_id not in mgr.object_ids
        assert obj.body_id not in mgr.body_ids

    def test_remove_nonexistent_returns_false(self, sim_core):
        """remove_object on unknown object returns False."""
        from pybullet_fleet.agent_manager import SimObjectManager
        from pybullet_fleet.sim_object import SimObject, SimObjectSpawnParams, ShapeParams
        from pybullet_fleet.geometry import Pose

        mgr = SimObjectManager(sim_core=sim_core)
        sp = SimObjectSpawnParams(
            visual_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            initial_pose=Pose.from_xyz(0, 0, 0.05),
            name="cube1",
        )
        obj = SimObject.from_params(sp, sim_core=sim_core)
        # obj NOT added to manager
        assert mgr.remove_object(obj) is False

    def test_clear_empties_all_tracking(self, sim_core):
        """clear() empties objects, body_ids, object_ids."""
        from pybullet_fleet.agent_manager import SimObjectManager
        from pybullet_fleet.sim_object import SimObjectSpawnParams, ShapeParams
        from pybullet_fleet.geometry import Pose

        mgr = SimObjectManager(sim_core=sim_core)
        sp = SimObjectSpawnParams(
            visual_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            initial_pose=Pose.from_xyz(0, 0, 0.05),
            name="cube2",
        )
        mgr.spawn_objects_batch([sp, sp])
        assert len(mgr.objects) == 2

        mgr.clear()
        assert len(mgr.objects) == 0
        assert len(mgr.body_ids) == 0
        assert len(mgr.object_ids) == 0


# ---------------------------------------------------------------------------
# Entity class registration from config
# ---------------------------------------------------------------------------


class TestEntityClassesFromConfig:
    """entity_classes config section: dynamic import and registration."""

    @_VIA
    def test_registers_entity_class(self, via):
        """entity_classes registers classes before spawning."""
        from pybullet_fleet.entity_registry import ENTITY_REGISTRY

        alias = f"alias_{via}"
        config = {
            "simulation": _HEADLESS_SIM,
            "entity_classes": {alias: "pybullet_fleet.agent.Agent"},
            "entities": [_robot(type=alias)],
        }
        try:
            with _make_sim(config, via=via) as sim:
                assert alias in ENTITY_REGISTRY
                assert len(sim.agents) == 1
                assert isinstance(sim.agents[0], Agent)
        finally:
            ENTITY_REGISTRY.pop(alias, None)

    def test_without_entity_classes_still_works(self):
        """Config without entity_classes section works as before."""
        config = {
            "simulation": _HEADLESS_SIM,
            "entities": [_robot()],
        }
        with _make_sim(config) as sim:
            assert len(sim.agents) == 1

    def test_bad_dotted_path_raises(self):
        """Invalid dotted path in entity_classes raises on from_dict."""
        config = {
            "simulation": _HEADLESS_SIM,
            "entity_classes": {"bad": "no_such_module.NoClass"},
        }
        with pytest.raises(ImportError):
            MultiRobotSimulationCore.from_dict(config)
