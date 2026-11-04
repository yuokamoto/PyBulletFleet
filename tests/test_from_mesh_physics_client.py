"""A body built from shapes lands on its simulation's physics client, not PyBullet's default one.

PyBullet calls without ``physicsClientId`` go to client 0. These tests keep a
second client connected so that the simulation's own client is not 0, which is
what happens with two simulations in one process.
"""

import os

import pybullet as p
import pytest

from pybullet_fleet.agent import Agent
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.geometry import Pose
from pybullet_fleet.sim_object import ShapeParams, SimObject

MESH_PATH = os.path.join(os.path.dirname(__file__), "..", "pybullet_fleet", "mesh", "cube.obj")


@pytest.fixture
def two_clients():
    """A bystander client and a simulation on another one; the simulation's client is never 0."""
    bystander = p.connect(p.DIRECT)
    sim = MultiRobotSimulationCore(SimulationParams(gui=False, monitor=False, enable_floor=False, log_level="warning"))
    if sim.client == 0:  # the bystander was not first; take 0 out of the way
        extra = p.connect(p.DIRECT)
        p.disconnect(sim.client)
        sim = MultiRobotSimulationCore(SimulationParams(gui=False, monitor=False, enable_floor=False, log_level="warning"))
        p.disconnect(extra)
    assert sim.client != 0
    yield sim
    for client in (sim.client, bystander):
        if p.isConnected(client):
            p.disconnect(client)


def _mesh():
    return ShapeParams(shape_type="mesh", mesh_path=MESH_PATH, mesh_scale=[0.2, 0.2, 0.2])


def _box():
    return ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1])


def _bodies(client):
    return p.getNumBodies(physicsClientId=client)


class TestAgentFromMesh:
    @pytest.mark.parametrize("shape", [_box, _mesh], ids=["box", "mesh"])
    def test_creates_its_body_on_the_simulations_client(self, two_clients, shape):
        sim = two_clients
        default_before = _bodies(0) if p.isConnected(0) else None
        before = _bodies(sim.client)
        agent = Agent.from_mesh(
            visual_shape=shape(),
            collision_shape=shape(),
            pose=Pose.from_xyz(1.0, 2.0, 0.5),
            sim_core=sim,
        )
        assert _bodies(sim.client) == before + 1
        if default_before is not None:
            assert _bodies(0) == default_before
        position, _ = p.getBasePositionAndOrientation(agent.body_id, physicsClientId=sim.client)
        assert position == pytest.approx((1.0, 2.0, 0.5))


class TestTheSharedShapeCache:
    def test_does_not_hand_one_clients_shapes_to_another(self, two_clients):
        sim = two_clients
        other = p.connect(p.DIRECT)
        try:
            on_other = SimObject.create_shared_shapes(visual_shape=_mesh(), physics_client_id=other)
            on_sim = SimObject.create_shared_shapes(visual_shape=_mesh(), physics_client_id=sim.client)
            # A shape id is only meaningful on the client that made it: the
            # second call must have built its own, which its client can draw.
            assert p.getVisualShapeData(
                p.createMultiBody(baseVisualShapeIndex=on_sim[0], physicsClientId=sim.client), physicsClientId=sim.client
            )
            assert SimObject.create_shared_shapes(visual_shape=_mesh(), physics_client_id=other) == on_other
            assert SimObject.create_shared_shapes(visual_shape=_mesh(), physics_client_id=sim.client) == on_sim
            assert len(SimObject._shared_shapes) == 2
        finally:
            p.disconnect(other)
