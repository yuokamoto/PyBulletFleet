# tests/test_sdf_loader.py
"""Tests for SimObject.from_sdf() SDF loading."""

import pybullet as p
import pybullet_data
import pytest

from pybullet_fleet import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import SimObject
from pybullet_fleet.types import CollisionMode


@pytest.fixture
def sim_core():
    """Headless sim_core for SDF tests."""
    sim = MultiRobotSimulationCore(SimulationParams(gui=False, monitor=False))
    sim.initialize_simulation()
    yield sim
    try:
        p.disconnect(sim.client)
    except p.error:
        pass


class TestFromSdf:
    """Tests for SimObject.from_sdf()."""

    def test_load_kiva_shelf(self, sim_core):
        """Load pybullet_data kiva_shelf SDF."""
        objects = SimObject.from_sdf("kiva_shelf/model.sdf", sim_core=sim_core)
        assert len(objects) >= 1
        assert all(isinstance(o, SimObject) for o in objects)
        assert all(o.object_id >= 0 for o in objects)

    def test_returns_list(self, sim_core):
        """Return type is always List[SimObject]."""
        result = SimObject.from_sdf("kiva_shelf/model.sdf", sim_core=sim_core)
        assert isinstance(result, list)

    def test_objects_registered_in_sim_core(self, sim_core):
        """All loaded objects are registered in sim_core."""
        objects = SimObject.from_sdf("kiva_shelf/model.sdf", sim_core=sim_core)
        for obj in objects:
            assert obj in sim_core.sim_objects

    def test_objects_have_names(self, sim_core):
        """Each loaded object has a name."""
        objects = SimObject.from_sdf("kiva_shelf/model.sdf", sim_core=sim_core)
        for obj in objects:
            assert obj.name is not None
            assert len(obj.name) > 0

    def test_global_scaling(self, sim_core):
        """global_scaling parameter is accepted."""
        objects = SimObject.from_sdf("kiva_shelf/model.sdf", sim_core=sim_core, global_scaling=2.0)
        assert len(objects) >= 1

    def test_collision_mode(self, sim_core):
        """collision_mode is applied to all loaded objects."""
        objects = SimObject.from_sdf(
            "kiva_shelf/model.sdf",
            sim_core=sim_core,
            collision_mode=CollisionMode.DISABLED,
        )
        for obj in objects:
            assert obj.collision_mode == CollisionMode.DISABLED

    def test_name_prefix(self, sim_core):
        """name_prefix overrides default naming."""
        objects = SimObject.from_sdf("kiva_shelf/model.sdf", sim_core=sim_core, name_prefix="shelf")
        for obj in objects:
            assert obj.name is not None

    def test_use_fixed_base_static(self, sim_core):
        """use_fixed_base=True makes objects static (mass=0)."""
        objects = SimObject.from_sdf("kiva_shelf/model.sdf", sim_core=sim_core, use_fixed_base=True)
        for obj in objects:
            assert obj.mass == 0.0

    def test_invalid_path_raises(self, sim_core):
        """Invalid SDF path raises error."""
        with pytest.raises((FileNotFoundError, RuntimeError)):
            SimObject.from_sdf("nonexistent_model.sdf", sim_core=sim_core)

    def test_without_sim_core(self):
        """from_sdf works without sim_core (standalone physics client)."""
        cid = p.connect(p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=cid)
        try:
            objects = SimObject.from_sdf("kiva_shelf/model.sdf")
            assert len(objects) >= 1
        finally:
            p.disconnect(cid)
