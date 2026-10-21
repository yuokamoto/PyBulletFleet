"""PathRequest: the one request object Agent.request_path and the controllers take."""

from __future__ import annotations

from dataclasses import replace

import pybullet as p
import pytest

from pybullet_fleet import (
    Agent,
    AgentSpawnParams,
    MotionMode,
    MultiRobotSimulationCore,
    PathRequest,
    PathRequestState,
    Pose,
    SimulationParams,
)
from pybullet_fleet.controller import Controller
from pybullet_fleet.types import MovementDirection


@pytest.fixture
def sim():
    sc = MultiRobotSimulationCore(
        SimulationParams(gui=False, monitor=False, physics=False, timestep=0.05, target_rtf=0, monitor_log_interval=0)
    )
    sc.initialize_simulation()
    yield sc
    p.disconnect(sc.client)


def _agent(sim, batch=False):
    agent = Agent.from_params(
        AgentSpawnParams(
            urdf_path="robots/simple_cube.urdf",
            initial_pose=Pose.from_xyz(0.0, 0.0, 0.1),
            motion_mode=MotionMode.OMNIDIRECTIONAL,
            controller={"max_linear_vel": 2.0, "max_linear_accel": 4.0},
        ),
        sim_core=sim,
    )
    if batch:
        from pybullet_fleet.agent_manager import AgentManager

        AgentManager(sim_core=sim, fleet_controller={"type": "batch_omni"}).add_object(agent)
    return agent


def test_fields_and_conversion():
    request = PathRequest(path=[Pose.from_xyz(1, 0, 0)], direction="backward")
    assert request.direction is MovementDirection.BACKWARD
    assert isinstance(request.path, tuple)
    assert request.final_orientation_align is True and request.auto_approach is True
    with pytest.raises(ValueError):
        PathRequest(path=[Pose.from_xyz(1, 0, 0)], direction="sideways")


def test_identity_survives_copies_but_not_equal_values():
    request = PathRequest(path=[Pose.from_xyz(1, 0, 0)])
    copy = replace(request, path=[Pose.from_xyz(0, 0, 0), Pose.from_xyz(1, 0, 0)])
    twin = PathRequest(path=[Pose.from_xyz(1, 0, 0)])
    assert request.same_request(copy)
    assert not request.same_request(twin)  # equal fields, another request
    assert not request.same_request(None)


def test_record_round_trip():
    request = PathRequest(path=[Pose.from_xyz(1, 2, 0)], final_orientation_align=False, direction="auto")
    back = PathRequest.from_record(request.to_record())
    assert back.to_record() == request.to_record()
    assert back.direction is MovementDirection.AUTO


@pytest.mark.parametrize("batch", [False, True], ids=["per_agent", "batch"])
def test_request_path_keeps_identity_through_the_approach_waypoint(sim, batch):
    agent = _agent(sim, batch)
    request = PathRequest(path=[Pose.from_xyz(3.0, 0.0, 0.1)], final_orientation_align=False)  # approach added
    assert agent.request_path(request) is True
    assert agent.path_request_state(request) is PathRequestState.ACTIVE
    for _ in range(200):
        sim.step_once()
    assert agent.path_request_state(request) is PathRequestState.FINISHED
    other = PathRequest(path=[Pose.from_xyz(0.0, 0.0, 0.1)])
    assert agent.path_request_state(other) is PathRequestState.GONE


def test_set_path_and_set_goal_pose_go_through_request_path(sim, monkeypatch):
    agent = _agent(sim)
    seen = []
    original = agent.request_path
    monkeypatch.setattr(agent, "request_path", lambda request, **kw: seen.append(request) or original(request, **kw))
    agent.set_path([Pose.from_xyz(1, 0, 0.1)], auto_approach=False, direction="forward")
    agent.set_goal_pose(Pose.from_xyz(2, 0, 0.1))
    assert [r.auto_approach for r in seen] == [False, True]
    assert seen[0].direction is MovementDirection.FORWARD


def test_a_controller_written_against_set_path_still_works(sim):
    class Legacy(Controller):
        def __init__(self):
            super().__init__()
            self.paths = []

        def compute(self, agent, dt):
            return False

        def set_path(self, agent, path, final_orientation_align=True, direction=None, preempt=False):
            self.paths.append((list(path), final_orientation_align))
            return True

    agent = _agent(sim)
    legacy = Legacy()
    agent.set_controller(legacy)
    goal = Pose.from_xyz(1.0, 0.0, 0.1)
    assert agent.request_path(PathRequest(path=[goal], final_orientation_align=False, auto_approach=False)) is True
    assert legacy.paths == [([goal], False)]
    assert agent.path_request_state(PathRequest(path=[goal])) is PathRequestState.UNKNOWN
