"""An AgentManager holding a plain SimObject stays usable.

``spawn_from_config()`` dispatches on each entry's ``type``, and a config may
mix ``agent`` and ``sim_object``, so a mixed manager is a supported
arrangement. Everything the subclass adds is agent-only, though -- the
fleet-controller defaults read ``controller_params``, the batch controller
reads ``_batch_controller`` -- and those paths used to run over every object
regardless, raising AttributeError as soon as either controller was attached.
"""

import pytest

from pybullet_fleet import Agent, AgentManager, AgentSpawnParams, MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.geometry import Pose
from pybullet_fleet.sim_object import ShapeParams, SimObject, SimObjectSpawnParams

URDF = "cube_small.urdf"

AGENT_DEF = {"type": "agent", "name": "r0", "urdf_path": URDF, "pose": [0, 0, 0.05]}
OBJECT_DEF = {
    "type": "sim_object",
    "name": "box0",
    "visual_shape": {"shape_type": "box", "half_extents": [0.3, 0.3, 0.1]},
    "collision_shape": {"shape_type": "box", "half_extents": [0.3, 0.3, 0.1]},
    "pose": [3, 0, 0.1],
}


@pytest.fixture
def sim_core():
    import pybullet as p

    sim = MultiRobotSimulationCore(SimulationParams(gui=False, monitor=False, enable_monitor_gui=False))
    sim.initialize_simulation()
    yield sim
    try:
        p.disconnect(sim.client)
    except p.error:
        pass


def _sim_object(sim_core, name):
    return SimObject.from_params(
        SimObjectSpawnParams(
            visual_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            name=name,
            initial_pose=Pose.from_xyz(0.0, 0.0, 0.0),
        ),
        sim_core=sim_core,
    )


def _agent(sim_core, name):
    return Agent.from_params(AgentSpawnParams(urdf_path=URDF, name=name), sim_core=sim_core)


class TestBatchControllerWithMixedObjects:
    def test_enable_batch_skips_a_sim_object_already_held(self, sim_core):
        manager = AgentManager(sim_core=sim_core)
        manager.add_object(_sim_object(sim_core, "wall"))
        agent = _agent(sim_core, "bot")
        manager.add_object(agent)

        bc = manager.enable_batch("batch_omni")

        assert agent._batch_controller is bc
        assert manager.get_object_count() == 2

    def test_a_sim_object_can_be_added_after_enable_batch(self, sim_core):
        manager = AgentManager(sim_core=sim_core)
        manager.enable_batch("batch_omni")
        manager.add_object(_sim_object(sim_core, "wall"))
        assert manager.get_object_count() == 1

    def test_a_sim_object_can_be_added_under_a_fleet_controller(self, sim_core):
        manager = AgentManager(sim_core=sim_core, fleet_controller={"max_linear_vel": [1.0, 1.0, 0.0]})
        manager.add_object(_sim_object(sim_core, "wall"))
        assert manager.get_object_count() == 1

    def test_disable_batch_leaves_a_sim_object_alone(self, sim_core):
        manager = AgentManager(sim_core=sim_core)
        agent = _agent(sim_core, "bot")
        manager.add_object(agent)
        manager.add_object(_sim_object(sim_core, "wall"))
        manager.enable_batch("batch_omni")

        manager.disable_batch()

        assert agent._batch_controller is None
        assert manager.batch_controller is None

    def test_removing_a_sim_object_under_batch_mode_works(self, sim_core):
        manager = AgentManager(sim_core=sim_core)
        manager.enable_batch("batch_omni")
        obj = _sim_object(sim_core, "wall")
        manager.add_object(obj)
        assert manager.remove_object(obj) is True
        assert manager.get_object_count() == 0

    def test_a_mixed_config_spawn_can_then_enable_batch(self, sim_core):
        """The path that makes a mixed manager in the first place."""
        manager = AgentManager(sim_core=sim_core)
        spawned = manager.spawn_from_config([AGENT_DEF, OBJECT_DEF])
        assert [type(o).__name__ for o in spawned] == ["Agent", "SimObject"]

        bc = manager.enable_batch("batch_omni")

        assert spawned[0]._batch_controller is bc
        assert not hasattr(spawned[1], "_batch_controller")
