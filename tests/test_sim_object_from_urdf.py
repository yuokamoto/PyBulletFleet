"""Tests for SimObject.from_urdf() URDF loading.

The counterpart of ``tests/test_sdf_loader.py``: SDF already had a
non-agent loader, URDF did not, so a URDF-defined body with no behaviour
had to be created as an ``Agent`` and then visited by the per-step update
loop for the life of the process.
"""

import pybullet as p
import pytest

from pybullet_fleet import Agent, AgentSpawnParams, MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.geometry import Pose
from pybullet_fleet.sim_object import ShapeParams, SimObject, SimObjectSpawnParams
from pybullet_fleet.types import CollisionMode

URDF = "cube_small.urdf"  # ships with pybullet_data


@pytest.fixture
def sim_core():
    sim = MultiRobotSimulationCore(SimulationParams(gui=False, monitor=False, enable_monitor_gui=False))
    sim.initialize_simulation()
    yield sim
    try:
        p.disconnect(sim.client)
    except p.error:
        pass


class TestFromUrdf:
    def test_loads_and_registers(self, sim_core):
        obj = SimObject.from_urdf(URDF, sim_core=sim_core)
        assert isinstance(obj, SimObject)
        assert not isinstance(obj, Agent)
        assert obj.object_id >= 0
        assert obj in sim_core.sim_objects

    def test_honours_the_pose_it_is_given(self, sim_core):
        obj = SimObject.from_urdf(URDF, pose=Pose.from_xyz(1.0, 2.0, 3.0), sim_core=sim_core)
        assert list(obj.get_pose().position) == pytest.approx([1.0, 2.0, 3.0])

    def test_defaults_to_static_kinematic_scenery(self, sim_core):
        """The case this exists for: static structure, walls and fixtures."""
        obj = SimObject.from_urdf(URDF, sim_core=sim_core, collision_mode=CollisionMode.STATIC)
        assert obj.mass == 0.0
        assert obj.is_kinematic is True
        assert obj.pickable is False
        assert obj.is_static is True

    def test_mass_none_keeps_the_urdf_values(self, sim_core):
        obj = SimObject.from_urdf(URDF, sim_core=sim_core, mass=None, use_fixed_base=False)
        assert obj.mass > 0.0

    def test_name_defaults_to_the_urdf_robot_name(self, sim_core):
        assert SimObject.from_urdf(URDF, sim_core=sim_core).name
        assert SimObject.from_urdf(URDF, sim_core=sim_core, name="shelf_a").name == "shelf_a"

    def test_it_stays_out_of_the_per_step_update_loop(self, sim_core):
        """The whole point. An Agent is visited every step to walk an empty
        action queue, check joints it has none of and sweep plugins it does
        not own; a SimObject is skipped by _needs_update."""
        scenery = SimObject.from_urdf(URDF, sim_core=sim_core)
        agent = Agent.from_params(AgentSpawnParams(urdf_path=URDF, name="bot", use_fixed_base=True), sim_core=sim_core)
        assert scenery._needs_update is False
        assert agent._needs_update is True

    def test_a_missing_file_is_reported_as_such(self, sim_core):
        with pytest.raises(FileNotFoundError, match="Failed to load URDF"):
            SimObject.from_urdf("/nonexistent/definitely_not_here.urdf", sim_core=sim_core)


class TestSpawnParamsUrdfPath:
    def test_from_params_routes_a_urdf_to_from_urdf(self, sim_core):
        obj = SimObject.from_params(
            SimObjectSpawnParams(
                urdf_path=URDF,
                name="tile",
                initial_pose=Pose.from_xyz(4.0, 0.0, 0.0),
                collision_mode=CollisionMode.STATIC,
            ),
            sim_core=sim_core,
        )
        assert obj.name == "tile"
        assert obj.collision_mode is CollisionMode.STATIC
        assert list(obj.get_pose().position) == pytest.approx([4.0, 0.0, 0.0])

    def test_from_dict_accepts_urdf_path(self, sim_core):
        obj = SimObject.from_dict({"name": "tile", "urdf_path": URDF, "pose": [1.0, 1.0, 0.0]}, sim_core=sim_core)
        assert list(obj.get_pose().position) == pytest.approx([1.0, 1.0, 0.0])

    def test_the_shape_path_is_unchanged(self, sim_core):
        """No urdf_path means the existing from_mesh route, exactly as before."""
        obj = SimObject.from_params(
            SimObjectSpawnParams(
                visual_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
                name="box",
                initial_pose=Pose.from_xyz(2.0, 0.0, 0.0),
            ),
            sim_core=sim_core,
        )
        assert obj.name == "box"
        assert list(obj.get_pose().position) == pytest.approx([2.0, 0.0, 0.0])


def test_field_order_is_unchanged_for_the_subclasses():
    """urdf_path is declared last on SimObjectSpawnParams on purpose.

    AgentSpawnParams and ElevatorParams inherit these fields, so declaring it
    anywhere else would silently change what the third positional argument
    means for all three dataclasses.
    """
    import dataclasses

    from pybullet_fleet.devices.elevator import ElevatorParams

    assert [f.name for f in dataclasses.fields(SimObjectSpawnParams)][-1] == "urdf_path"
    for cls in (AgentSpawnParams, ElevatorParams):
        names = [f.name for f in dataclasses.fields(cls)]
        assert names[:10] == [
            "visual_shape",
            "collision_shape",
            "initial_pose",
            "mass",
            "pickable",
            "name",
            "visual_frame_pose",
            "collision_frame_pose",
            "collision_mode",
            "user_data",
        ]
        assert names[10] == "urdf_path"
