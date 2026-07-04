from __future__ import annotations

from dataclasses import dataclass, field
from typing import cast

import pytest

from pybullet_fleet.events import EventBus
from pybullet_fleet.fleet_api import (
    FLEET_COMMAND_EVENT,
    FleetCommandDispatcher,
    FleetStateProvider,
    RobotGoalCommand2D,
    RobotGoalCommand3D,
    RobotJointCommand,
)
from pybullet_fleet.geometry import Pose


@dataclass
class FakeAgent:
    name: str
    object_id: int
    pose: Pose = field(default_factory=lambda: Pose.from_xyz(0.0, 0.0, 0.0))
    velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    angular_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    is_moving: bool = False
    battery_soc: float | None = None
    is_charging: bool = False

    def __post_init__(self) -> None:
        self.goal_calls = []
        self.stop_calls = 0
        self.joint_calls = []

    def get_pose(self) -> Pose:
        return self.pose

    def set_goal_pose(self, pose: Pose) -> None:
        self.goal_calls.append(pose)

    def stop(self) -> None:
        self.stop_calls += 1

    def set_all_joints_targets(self, positions, max_force=500.0):
        self.joint_calls.append(("all", list(positions), max_force))

    def set_joints_targets_by_name(self, positions_by_name, max_force=500.0):
        self.joint_calls.append(("named", dict(positions_by_name), max_force))


class FakeSim:
    def __init__(self, agents, sim_time=12.5):
        self.agents = agents
        self.sim_time = sim_time
        self.events = EventBus()


def test_fleet_state_provider_returns_3d_and_2d_snapshots():
    sim = FakeSim(
        [
            FakeAgent(
                "robot0",
                7,
                pose=Pose.from_yaw(1.0, 2.0, 0.3, 1.57),
                velocity=(0.4, 0.5, 0.0),
                angular_velocity=0.2,
                is_moving=True,
                battery_soc=0.75,
            )
        ]
    )

    provider = FleetStateProvider(sim)

    states_3d = provider.get_states()
    assert len(states_3d) == 1
    assert states_3d[0].name == "robot0"
    assert states_3d[0].object_id == 7
    assert states_3d[0].position == (1.0, 2.0, 0.3)
    assert states_3d[0].linear_velocity == (0.4, 0.5, 0.0)
    assert states_3d[0].angular_velocity == (0.0, 0.0, 0.2)
    assert states_3d[0].battery_soc == 0.75

    states_2d = provider.get_states_2d(names=["robot0"])
    assert len(states_2d) == 1
    assert states_2d[0].x == 1.0
    assert states_2d[0].y == 2.0
    assert states_2d[0].linear_velocity == (0.4, 0.5)
    assert states_2d[0].angular_velocity == 0.2


def test_dispatcher_navigate_accepts_many_and_emits_before_mutation():
    robot0 = FakeAgent("robot0", 1)
    robot1 = FakeAgent("robot1", 2)
    sim = FakeSim([robot0, robot1])
    emitted = []
    sim.events.on(FLEET_COMMAND_EVENT, lambda command_event: emitted.append((command_event, len(robot0.goal_calls))))

    dispatcher = FleetCommandDispatcher(sim)
    ack = dispatcher.navigate(
        [
            RobotGoalCommand2D("robot0", x=1.0, y=2.0, yaw=0.5),
            RobotGoalCommand3D("robot1", position=(3.0, 4.0, 0.0)),
        ],
        source="test",
        command_id="cmd-1",
    )

    assert ack.ok
    assert ack.accepted_names == ("robot0", "robot1")
    assert ack.rejected == {}
    assert robot0.goal_calls[0].x == 1.0
    assert robot0.goal_calls[0].yaw == pytest.approx(0.5)
    assert robot1.goal_calls[0].position == [3.0, 4.0, 0.0]
    assert dispatcher.command_events[0].command_type == "navigate"
    assert emitted[0][0] == dispatcher.command_events[0]
    assert emitted[0][1] == 0


def test_dispatcher_rejects_unknown_duplicate_and_ambiguous_names():
    robot0 = FakeAgent("robot0", 1)
    duplicate_a = FakeAgent("dup", 2)
    duplicate_b = FakeAgent("dup", 3)
    sim = FakeSim([robot0, duplicate_a, duplicate_b])

    dispatcher = FleetCommandDispatcher(sim)
    ack = dispatcher.navigate(
        [
            RobotGoalCommand2D("robot0", x=1.0, y=0.0),
            RobotGoalCommand2D("robot0", x=2.0, y=0.0),
            RobotGoalCommand2D("missing", x=3.0, y=0.0),
            RobotGoalCommand2D("dup", x=4.0, y=0.0),
        ],
        command_id="cmd-2",
    )

    assert ack.accepted_names == ()
    assert ack.rejected == {
        "robot0": "duplicate target",
        "missing": "unknown robot",
        "dup": "ambiguous robot name",
    }
    assert robot0.goal_calls == []
    assert duplicate_a.goal_calls == []
    assert duplicate_b.goal_calls == []


def test_state_provider_handles_minimal_unnamed_agent_stub():
    class MinimalAgent:
        def get_pose(self):
            return Pose.from_xyz(0.0, 0.0, 0.0)

    states = FleetStateProvider(FakeSim([MinimalAgent()])).get_states()

    assert states[0].name == "agent_unknown"
    assert states[0].object_id == -1


def test_dispatcher_joint_command_and_stop():
    robot0 = FakeAgent("robot0", 1)
    robot1 = FakeAgent("robot1", 2)
    dispatcher = FleetCommandDispatcher(FakeSim([robot0, robot1]))

    joint_ack = dispatcher.joint_command(
        [
            RobotJointCommand("robot0", positions=(0.1, 0.2), max_force=10.0),
            RobotJointCommand("robot1", positions_by_name={"joint": 1.5}),
        ],
        command_id="cmd-3",
    )
    stop_ack = dispatcher.stop(["robot0", "missing"], command_id="cmd-4")

    assert joint_ack.ok
    assert robot0.joint_calls == [("all", [0.1, 0.2], 10.0)]
    assert robot1.joint_calls == [("named", {"joint": 1.5}, 500.0)]
    assert stop_ack.accepted_names == ("robot0",)
    assert stop_ack.rejected == {"missing": "unknown robot"}
    assert robot0.stop_calls == 1


def test_command_ack_and_event_rejected_maps_are_immutable():
    dispatcher = FleetCommandDispatcher(FakeSim([]))

    ack = dispatcher.stop(["missing"], command_id="cmd-5")
    event = dispatcher.command_events[0]

    assert ack.rejected == {"missing": "unknown robot"}
    assert event.rejected == {"missing": "unknown robot"}
    with pytest.raises(TypeError):
        cast(dict[str, str], ack.rejected)["other"] = "mutated"
    with pytest.raises(TypeError):
        cast(dict[str, str], event.rejected)["other"] = "mutated"


def test_robot_joint_command_requires_one_target_form():
    with pytest.raises(ValueError, match="requires positions"):
        RobotJointCommand("robot0")

    with pytest.raises(ValueError, match="either positions or positions_by_name"):
        RobotJointCommand("robot0", positions=(0.1,), positions_by_name={"joint": 0.1})
