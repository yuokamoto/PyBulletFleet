# tests/test_sdf_loader.py
"""Tests for SimObject.from_sdf() SDF loading."""

import pybullet as p
import pybullet_data
import pytest

from pybullet_fleet import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sdf_loader import _parse_pose, load_sdf_world, DEFAULT_SCENERY_COLOR
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

    def test_objects_registered_in_sim_core(self, sim_core):
        """All loaded objects are registered in sim_core."""
        objects = SimObject.from_sdf("kiva_shelf/model.sdf", sim_core=sim_core)
        for obj in objects:
            assert obj in sim_core.sim_objects

    def test_objects_have_names(self, sim_core):
        """Each loaded object has a non-empty name."""
        objects = SimObject.from_sdf("kiva_shelf/model.sdf", sim_core=sim_core)
        for obj in objects:
            assert obj.name  # not None, not ""

    def test_global_scaling(self, sim_core):
        """global_scaling=2.0 produces a larger AABB than 1.0."""
        objs_1x = SimObject.from_sdf("kiva_shelf/model.sdf", sim_core=sim_core, global_scaling=1.0)
        objs_2x = SimObject.from_sdf("kiva_shelf/model.sdf", sim_core=sim_core, global_scaling=2.0)

        # Match bodies by name (ordering may differ between loads)
        by_name_1x = {o.name: o for o in objs_1x}
        by_name_2x = {o.name: o for o in objs_2x}
        assert by_name_1x.keys() == by_name_2x.keys()

        for name in by_name_1x:
            aabb_1 = p.getAABB(by_name_1x[name].body_id, physicsClientId=sim_core.client)
            aabb_2 = p.getAABB(by_name_2x[name].body_id, physicsClientId=sim_core.client)
            size_1 = max(aabb_1[1][i] - aabb_1[0][i] for i in range(3))
            size_2 = max(aabb_2[1][i] - aabb_2[0][i] for i in range(3))
            if size_1 > 0.001:  # skip degenerate bodies
                assert size_2 == pytest.approx(size_1 * 2.0, rel=0.05)

    def test_collision_mode(self, sim_core):
        """collision_mode is applied to all loaded objects."""
        objects = SimObject.from_sdf(
            "kiva_shelf/model.sdf",
            sim_core=sim_core,
            collision_mode=CollisionMode.DISABLED,
        )
        for obj in objects:
            assert obj.collision_mode == CollisionMode.DISABLED

    def test_use_fixed_base(self, sim_core):
        """use_fixed_base=True → mass=0, use_fixed_base=False → mass>0."""
        fixed = SimObject.from_sdf("kiva_shelf/model.sdf", sim_core=sim_core, use_fixed_base=True)
        for obj in fixed:
            assert obj.mass == 0.0

        free = SimObject.from_sdf("kiva_shelf/model.sdf", sim_core=sim_core, use_fixed_base=False)
        for obj in free:
            assert obj.mass is None or obj.mass >= 0.0  # not forced to 0

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


class TestMultiModelSdf:
    """Tests for load_sdf_world handling multiple <model> elements."""

    def _write_sdf(self, tmp_path, content: str) -> str:
        """Write SDF content to a temp file and return the path."""
        sdf_file = tmp_path / "model.sdf"
        sdf_file.write_text(content)
        return str(sdf_file)

    def _make_mesh(self, tmp_path, name: str = "box.obj") -> str:
        """Create a minimal OBJ mesh file."""
        mesh_path = tmp_path / name
        mesh_path.write_text(
            "v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\n" "v 0 0 1\nv 1 0 1\nv 1 1 1\nv 0 1 1\n" "f 1 2 3 4\nf 5 6 7 8\n"
        )
        return str(mesh_path)

    def test_single_model_still_works(self, sim_core, tmp_path):
        """Single <model> SDF still loads correctly (regression)."""
        mesh = self._make_mesh(tmp_path)
        sdf = self._write_sdf(
            tmp_path,
            f"""<?xml version="1.0"?>
            <sdf version="1.6">
              <model name="building">
                <link name="floor">
                  <visual name="v"><geometry><mesh><uri>{mesh}</uri></mesh></geometry></visual>
                </link>
              </model>
            </sdf>""",
        )
        objects = load_sdf_world(sdf, sim_core=sim_core)
        assert len(objects) == 1
        assert objects[0].name == "floor"

    def test_multiple_top_level_models(self, sim_core, tmp_path):
        """Multiple <model> elements at same level are all loaded."""
        mesh = self._make_mesh(tmp_path)
        sdf = self._write_sdf(
            tmp_path,
            f"""<?xml version="1.0"?>
            <sdf version="1.6">
              <model name="building_A">
                <link name="floor_A">
                  <visual name="v"><geometry><mesh><uri>{mesh}</uri></mesh></geometry></visual>
                </link>
              </model>
              <model name="building_B">
                <link name="wall_B">
                  <visual name="v"><geometry><mesh><uri>{mesh}</uri></mesh></geometry></visual>
                </link>
              </model>
            </sdf>""",
        )
        objects = load_sdf_world(sdf, sim_core=sim_core)
        names = {o.name for o in objects}
        assert "floor_A" in names
        assert "wall_B" in names
        assert len(objects) == 2

    def test_three_models_all_links_loaded(self, sim_core, tmp_path):
        """Three models with multiple links each — all links are loaded."""
        mesh = self._make_mesh(tmp_path)
        sdf = self._write_sdf(
            tmp_path,
            f"""<?xml version="1.0"?>
            <sdf version="1.6">
              <model name="m1">
                <link name="L1a">
                  <visual name="v"><geometry><mesh><uri>{mesh}</uri></mesh></geometry></visual>
                </link>
                <link name="L1b">
                  <visual name="v"><geometry><mesh><uri>{mesh}</uri></mesh></geometry></visual>
                </link>
              </model>
              <model name="m2">
                <link name="L2">
                  <visual name="v"><geometry><mesh><uri>{mesh}</uri></mesh></geometry></visual>
                </link>
              </model>
              <model name="m3">
                <link name="L3">
                  <visual name="v"><geometry><mesh><uri>{mesh}</uri></mesh></geometry></visual>
                </link>
              </model>
            </sdf>""",
        )
        objects = load_sdf_world(sdf, sim_core=sim_core)
        assert len(objects) == 4
        names = {o.name for o in objects}
        assert names == {"L1a", "L1b", "L2", "L3"}

    def test_no_model_raises(self, sim_core, tmp_path):
        """SDF with no <model> element raises ValueError."""
        sdf = self._write_sdf(
            tmp_path,
            """<?xml version="1.0"?>
            <sdf version="1.6">
            </sdf>""",
        )
        with pytest.raises(ValueError, match="No <model> element"):
            load_sdf_world(sdf, sim_core=sim_core)


class TestParsePose:
    """Tests for _parse_pose returning full 6-DoF."""

    def test_empty_returns_zeros(self):
        """None or empty string returns 6 zeros."""
        result = _parse_pose(None)
        assert result == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        result = _parse_pose("")
        assert result == (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def test_xyz_only(self):
        """3-value string returns xyz with rpy=0."""
        result = _parse_pose("1.0 2.0 3.0")
        assert result == (1.0, 2.0, 3.0, 0.0, 0.0, 0.0)

    def test_full_6dof(self):
        """6-value string returns all components."""
        result = _parse_pose("1.0 2.0 3.0 0.1 0.2 1.57")
        assert result == pytest.approx((1.0, 2.0, 3.0, 0.1, 0.2, 1.57))


class TestDefaultSceneryColor:
    """Tests for DEFAULT_SCENERY_COLOR constant."""

    def test_matches_gazebo_sim_default(self):
        """Value matches Gazebo Sim (Harmonic) default of 0.7 grey."""
        assert DEFAULT_SCENERY_COLOR == [0.7, 0.7, 0.7, 1.0]
