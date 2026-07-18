#!/usr/bin/env python3
"""ROS 2 bridge integration checks for docker/test_integration.sh."""

from __future__ import annotations

import sys

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from pybullet_fleet_msgs.msg import (
    FleetState,
    RobotAttachCommand,
    RobotGoal2D,
    RobotJointPositionsCommand,
)
from pybullet_fleet_msgs.srv import FleetAttach, FleetJointCommand, FleetNavigate, FleetStop
from rosgraph_msgs.msg import Clock
from simulation_interfaces.msg import Result
from simulation_interfaces.srv import (
    DeleteEntity,
    GetEntities,
    GetEntityState,
    GetSimulatorFeatures,
    SetEntityState,
    SpawnEntity,
)

from ros_check_utils import RosCheckNode

EXPECTED_TOPICS = [
    "/clock",
    "/robot0/odom",
    "/robot0/cmd_vel",
    "/robot0/joint_states",
    "/robot1/odom",
    "/robot2/odom",
    "/tf",
    "/fleet/states",
    "/fleet/navigate",
    "/fleet/stop",
    "/fleet/attach",
    "/fleet/joint_command",
]
EXPECTED_SERVICES = [
    "/sim/get_simulator_features",
    "/sim/spawn_entity",
    "/sim/delete_entity",
    "/sim/get_entity_state",
    "/sim/set_entity_state",
    "/sim/get_entities",
    "/sim/step_simulation",
    "/sim/get_simulation_state",
    "/fleet/navigate",
    "/fleet/stop",
    "/fleet/attach",
    "/fleet/joint_command",
]
EXPECTED_ROBOTS = {"robot0", "robot1", "robot2"}
READY_TIMEOUT = 20.0
CALL_TIMEOUT = 10.0


class BridgeIntegrationCheck(RosCheckNode):
    def __init__(self) -> None:
        super().__init__("pybullet_fleet_bridge_integration_check")
        self.clock_seen = False
        self.odom_seen = False
        self.fleet_names: set[str] = set()
        self.create_subscription(Clock, "/clock", self._on_clock, 10)
        self.create_subscription(Odometry, "/robot0/odom", self._on_odom, 10)
        self.create_subscription(FleetState, "/fleet/states", self._on_fleet_state, 10)
        self.cmd_vel = self.create_publisher(Twist, "/robot0/cmd_vel", 10)
        self.delete_entity = self.create_client(DeleteEntity, "/sim/delete_entity")
        self.get_entities = self.create_client(GetEntities, "/sim/get_entities")
        self.get_entity_state = self.create_client(GetEntityState, "/sim/get_entity_state")
        self.get_features = self.create_client(GetSimulatorFeatures, "/sim/get_simulator_features")
        self.set_entity_state = self.create_client(SetEntityState, "/sim/set_entity_state")
        self.spawn_entity = self.create_client(SpawnEntity, "/sim/spawn_entity")
        self.fleet_nav = self.create_client(FleetNavigate, "/fleet/navigate")
        self.fleet_stop = self.create_client(FleetStop, "/fleet/stop")
        self.fleet_attach = self.create_client(FleetAttach, "/fleet/attach")
        self.fleet_joint = self.create_client(FleetJointCommand, "/fleet/joint_command")

    def _on_clock(self, _msg: Clock) -> None:
        self.clock_seen = True

    def _on_odom(self, _msg: Odometry) -> None:
        self.odom_seen = True

    def _on_fleet_state(self, msg: FleetState) -> None:
        self.fleet_names.update(robot.name for robot in msg.robots)


def _check_names(label: str, present: set[str], expected: list[str]) -> bool:
    ok = True
    for name in expected:
        if name in present:
            print(f"  OK {label} {name}")
        else:
            print(f"  MISSING {label} {name}")
            ok = False
    return ok


def _result_ok(response) -> bool:
    return response.result.result == Result.RESULT_OK


def _result_message(response) -> str:
    return getattr(response.result, "error_message", "")


def _has_entity(node: BridgeIntegrationCheck, name: str) -> bool:
    response = node.call_service(node.get_entities, GetEntities.Request(), CALL_TIMEOUT)
    return name in set(response.entities)


def _check_sim_crud(node: BridgeIntegrationCheck) -> bool:
    name = "smoke_spawned"

    if _has_entity(node, name):
        cleanup_req = DeleteEntity.Request()
        cleanup_req.entity = name
        node.call_service(node.delete_entity, cleanup_req, CALL_TIMEOUT)

    spawn_req = SpawnEntity.Request()
    spawn_req.name = name
    spawn_req.uri = "robots/simple_cube.urdf"
    spawn_req.initial_pose.pose.position.x = 2.0
    spawn_req.initial_pose.pose.position.y = 0.0
    spawn_req.initial_pose.pose.position.z = 0.05
    spawn_req.initial_pose.pose.orientation.w = 1.0
    spawn_resp = node.call_service(node.spawn_entity, spawn_req, CALL_TIMEOUT)
    if not _result_ok(spawn_resp) or spawn_resp.entity_name != name:
        print(f"  FAIL /sim/spawn_entity: {spawn_resp.entity_name} {_result_message(spawn_resp)}")
        return False

    get_req = GetEntityState.Request()
    get_req.entity = name
    get_resp = node.call_service(node.get_entity_state, get_req, CALL_TIMEOUT)
    if not _result_ok(get_resp):
        print(f"  FAIL /sim/get_entity_state after spawn: {_result_message(get_resp)}")
        return False

    set_req = SetEntityState.Request()
    set_req.entity = name
    if hasattr(set_req, "set_pose"):
        set_req.set_pose = True
    set_req.state.pose.position.x = 3.0
    set_req.state.pose.position.y = 1.0
    set_req.state.pose.position.z = 0.05
    set_req.state.pose.orientation.w = 1.0
    set_resp = node.call_service(node.set_entity_state, set_req, CALL_TIMEOUT)
    if not _result_ok(set_resp):
        print(f"  FAIL /sim/set_entity_state: {_result_message(set_resp)}")
        return False

    moved = node.call_service(node.get_entity_state, get_req, CALL_TIMEOUT)
    if not _result_ok(moved):
        print(f"  FAIL /sim/get_entity_state after set: {_result_message(moved)}")
        return False
    pos = moved.state.pose.position
    if abs(pos.x - 3.0) > 1e-3 or abs(pos.y - 1.0) > 1e-3:
        print(f"  FAIL /sim/set_entity_state pose mismatch: x={pos.x:.3f}, y={pos.y:.3f}")
        return False

    delete_req = DeleteEntity.Request()
    delete_req.entity = name
    delete_resp = node.call_service(node.delete_entity, delete_req, CALL_TIMEOUT)
    if not _result_ok(delete_resp):
        print(f"  FAIL /sim/delete_entity: {_result_message(delete_resp)}")
        return False
    if _has_entity(node, name):
        print(f"  FAIL /sim/delete_entity: {name} still listed")
        return False

    print("  OK spawn/get/set/delete entity services")
    return True


def main() -> int:
    rclpy.init()
    node = BridgeIntegrationCheck()
    try:
        print("--- Checking topics ---")
        if not node.spin_until(lambda: set(EXPECTED_TOPICS).issubset(node.present_topics()), READY_TIMEOUT):
            _check_names("topic", node.present_topics(), EXPECTED_TOPICS)
            return 1
        _check_names("topic", node.present_topics(), EXPECTED_TOPICS)

        print("--- Checking services ---")
        if not node.spin_until(lambda: set(EXPECTED_SERVICES).issubset(node.present_services()), READY_TIMEOUT):
            _check_names("service", node.present_services(), EXPECTED_SERVICES)
            return 1
        _check_names("service", node.present_services(), EXPECTED_SERVICES)

        print("--- Checking publishers ---")
        ready = node.spin_until(
            lambda: node.clock_seen and node.odom_seen and EXPECTED_ROBOTS.issubset(node.fleet_names),
            READY_TIMEOUT,
        )
        if not ready:
            print(
                "  FAIL publisher readiness: "
                f"clock={node.clock_seen}, odom={node.odom_seen}, fleet={sorted(node.fleet_names)}"
            )
            return 1
        print("  OK /clock, /robot0/odom, and /fleet/states publishing")

        print("--- Testing cmd_vel -> controller path ---")
        twist = Twist()
        twist.linear.x = 1.0
        node.cmd_vel.publish(twist)
        node.spin_for(0.5)
        print("  OK /robot0/cmd_vel publish path is available")

        print("--- Testing simulation services ---")
        entities = node.call_service(node.get_entities, GetEntities.Request(), CALL_TIMEOUT)
        if len(entities.entities) == 0:
            print("  FAIL /sim/get_entities returned no entities")
            return 1
        features = node.call_service(node.get_features, GetSimulatorFeatures.Request(), CALL_TIMEOUT)
        if len(features.features.features) == 0:
            print("  FAIL /sim/get_simulator_features returned no features")
            return 1
        print(f"  OK simulation services returned {len(entities.entities)} entities")
        if not _check_sim_crud(node):
            return 1

        print("--- Testing fleet services ---")
        nav_req = FleetNavigate.Request()
        nav_req.command_id = "smoke-nav"
        nav_req.source = "smoke"
        nav_req.goals_2d = [RobotGoal2D(name="robot0", position=[1.0, 0.0], yaw=0.0, z=0.05)]
        nav_resp = node.call_service(node.fleet_nav, nav_req, CALL_TIMEOUT)
        if "robot0" not in nav_resp.ack.accepted_names:
            print(f"  FAIL /fleet/navigate ack: {nav_resp.ack}")
            return 1

        stop_req = FleetStop.Request()
        stop_req.command_id = "smoke-stop"
        stop_req.source = "smoke"
        stop_req.names = ["robot0"]
        stop_resp = node.call_service(node.fleet_stop, stop_req, CALL_TIMEOUT)
        if "robot0" not in stop_resp.ack.accepted_names:
            print(f"  FAIL /fleet/stop ack: {stop_resp.ack}")
            return 1

        attach_req = FleetAttach.Request()
        attach_req.command_id = "smoke-attach"
        attach_req.source = "smoke"
        attach_req.commands = [RobotAttachCommand(name="robot0", attach=False)]
        attach_resp = node.call_service(node.fleet_attach, attach_req, CALL_TIMEOUT)
        if "robot0" not in attach_resp.ack.rejected_names:
            print(f"  FAIL /fleet/attach reject ack: {attach_resp.ack}")
            return 1

        joint_req = FleetJointCommand.Request()
        joint_req.command_id = "smoke-joint"
        joint_req.source = "smoke"
        joint_req.joint_position_commands = [RobotJointPositionsCommand(name="robot0", positions=[])]
        joint_resp = node.call_service(node.fleet_joint, joint_req, CALL_TIMEOUT)
        if "robot0" not in joint_resp.ack.accepted_names:
            print(f"  FAIL /fleet/joint_command ack: {joint_resp.ack}")
            return 1
        print(
            "  OK /fleet/navigate, /fleet/stop, /fleet/attach, "
            "and /fleet/joint_command responded"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
