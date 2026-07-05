"""Tests for fleet-level ROS wrappers.

These tests require generated ``pybullet_fleet_msgs`` Python modules, so they
run in the ROS workspace and skip in a plain Python environment.
"""

from dataclasses import dataclass, field
from unittest.mock import MagicMock

import pytest

pytest.importorskip("pybullet_fleet_msgs.msg", reason="pybullet_fleet_msgs not generated")

from pybullet_fleet.geometry import Pose
from pybullet_fleet_ros.fleet_ros_interface import FleetRosInterface
from pybullet_fleet_ros.interface_config import FleetApiConfig
from pybullet_fleet_msgs.msg import FleetJointCommand, FleetNavigate, RobotGoal2D, RobotNamedJointPositionsCommand
from pybullet_fleet_msgs.srv import FleetJointCommand as FleetJointCommandSrv
from pybullet_fleet_msgs.srv import FleetNavigate as FleetNavigateSrv


@dataclass
class FakeAgent:
    name: str
    object_id: int
    pose: Pose = field(default_factory=lambda: Pose.from_xyz(0.0, 0.0, 0.0))
    velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    angular_velocity: float = 0.0
    is_moving: bool = False

    def __post_init__(self):
        self.goal_calls = []
        self.joint_calls = []

    def get_pose(self):
        return self.pose

    def set_goal_pose(self, pose):
        self.goal_calls.append(pose)

    def set_joints_targets_by_name(self, positions):
        self.joint_calls.append(dict(positions))


class FakeSim:
    def __init__(self, agents):
        self.agents = agents
        self.sim_time = 12.5


def _node():
    node = MagicMock()
    node.get_logger.return_value = MagicMock()
    node.create_publisher.return_value = MagicMock()
    node.create_subscription.return_value = MagicMock()
    node.create_service.return_value = MagicMock()
    return node


def test_fleet_state_publisher_uses_provider():
    node = _node()
    agent = FakeAgent("robot0", 7, pose=Pose.from_yaw(1.0, 2.0, 0.3, 1.57), velocity=(0.4, 0.5, 0.0))
    interface = FleetRosInterface(node, FakeSim([agent]), FleetApiConfig(enabled=True, states=True))

    interface.post_step()

    published = node.create_publisher.return_value.publish.call_args.args[0]
    assert published.robots[0].name == "robot0"
    assert published.robots[0].object_id == 7
    assert published.robots[0].pose.position.x == pytest.approx(1.0)
    assert published.robots[0].twist.linear.x == pytest.approx(0.4)


def test_fleet_state_publisher_applies_frame_offset():
    node = _node()
    node.rmf_frame_offset = (100.0, 200.0)
    agent = FakeAgent("robot0", 7, pose=Pose.from_xyz(1.0, 2.0, 0.3))
    interface = FleetRosInterface(node, FakeSim([agent]), FleetApiConfig(enabled=True, states=True))

    interface.post_step()

    published = node.create_publisher.return_value.publish.call_args.args[0]
    assert published.robots[0].pose.position.x == pytest.approx(101.0)
    assert published.robots[0].pose.position.y == pytest.approx(202.0)


def test_fleet_navigate_service_dispatches_2d_goal():
    node = _node()
    agent = FakeAgent("robot0", 1)
    interface = FleetRosInterface(node, FakeSim([agent]), FleetApiConfig(enabled=True, navigate=True))
    request = FleetNavigateSrv.Request()
    request.command_id = "cmd-1"
    request.source = "test"
    request.goals_2d = [RobotGoal2D(name="robot0", position=[1.0, 2.0], yaw=0.5, z=0.0)]
    response = FleetNavigateSrv.Response()

    result = interface._on_navigate_service(request, response)

    assert result.ack.command_id == "cmd-1"
    assert result.ack.accepted_names == ["robot0"]
    assert agent.goal_calls[0].x == pytest.approx(1.0)
    assert agent.goal_calls[0].yaw == pytest.approx(0.5)


def test_fleet_navigate_service_applies_frame_offset():
    node = _node()
    node.rmf_frame_offset = (100.0, 200.0)
    agent = FakeAgent("robot0", 1)
    interface = FleetRosInterface(node, FakeSim([agent]), FleetApiConfig(enabled=True, navigate=True))
    request = FleetNavigateSrv.Request()
    request.goals_2d = [RobotGoal2D(name="robot0", position=[101.0, 202.0], yaw=0.5, z=0.0)]
    response = FleetNavigateSrv.Response()

    interface._on_navigate_service(request, response)

    assert agent.goal_calls[0].x == pytest.approx(1.0)
    assert agent.goal_calls[0].y == pytest.approx(2.0)


def test_named_joint_service_rejects_mismatched_arrays():
    node = _node()
    interface = FleetRosInterface(
        node,
        FakeSim([FakeAgent("robot0", 1)]),
        FleetApiConfig(enabled=True, joint_command=True),
    )
    request = FleetJointCommandSrv.Request()
    request.command_id = "cmd-2"
    request.named_joint_position_commands = [
        RobotNamedJointPositionsCommand(name="robot0", joint_names=["joint1", "joint2"], positions=[1.0])
    ]
    response = FleetJointCommandSrv.Response()

    result = interface._on_joint_command_service(request, response)

    assert result.ack.command_id == "cmd-2"
    assert result.ack.accepted_names == []
    assert result.ack.rejected_names == ["request"]
    assert "2 joint names" in result.ack.reject_reasons[0]
