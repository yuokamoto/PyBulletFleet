"""Concrete validation profile for the mobile-manipulator recording example.

This module belongs to the example, not the reusable PBF recorder. It declares
which PBF state can be restored and rejects unhandled execution state.
"""

from __future__ import annotations

import hashlib
import math
import time
from pathlib import Path
from typing import Any, Callable

import pybullet as p

from pybullet_fleet.agent import Agent, AgentSpawnParams
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.geometry import Pose
from pybullet_fleet.sim_object import ShapeParams, SimObject, SimObjectSpawnParams
from pybullet_fleet.types import CollisionMode
from pybullet_fleet.replay.state_recording import _keys, _vector, _pose_record, _pose_from_record
from pybullet_fleet.replay.state_recording import ResultPlayback, restore_supported_simulation

PROFILE = "pbf.validation.kinematic_manipulation.v1"
VERSION = 1


def _validate_checkpoint(manifest: dict, state: dict) -> None:
    """Reject malformed or unsupported V1 state before constructing a world."""
    _keys(
        manifest,
        {
            "profile",
            "version",
            "complete",
            "construction",
            "records",
            "checkpoint_every_steps",
            "last_completed_step",
            "coverage",
            "checkpoint_sha256",
            "frames_sha256",
            "inputs_sha256",
        },
        "manifest",
    )
    if manifest["profile"] != PROFILE or manifest["version"] != VERSION or manifest["complete"] is not True:
        raise ValueError("Unsupported or incomplete recording")
    c = _keys(
        manifest["construction"],
        {
            "timestep",
            "physics",
            "enable_floor",
            "collision_check_frequency",
            "robot_name",
            "joint_name",
            "box_name",
            "robot_asset",
            "robot_asset_sha256",
            "controller",
            "robot_mass",
            "use_fixed_base",
            "robot_collision_mode",
            "initial_pose",
        },
        "construction",
    )
    dt = c["timestep"]
    if isinstance(dt, bool) or not isinstance(dt, (int, float)) or not math.isfinite(dt) or dt <= 0:
        raise ValueError("Invalid timestep")
    if (
        c["physics"] is not False
        or c["enable_floor"] is not False
        or c["collision_check_frequency"] != 0
        or c["robot_mass"] != 0
        or c["use_fixed_base"] is not False
        or c["robot_collision_mode"] != CollisionMode.NORMAL_3D.value
    ):
        raise ValueError("Unsupported simulation construction")
    if len({c["robot_name"], c["box_name"]}) != 2 or any(
        not isinstance(c[key], str) or not c[key] for key in ("robot_name", "joint_name", "box_name", "robot_asset")
    ):
        raise ValueError("Invalid or duplicate entity identity")
    controller = c["controller"]
    if (
        not isinstance(controller, dict)
        or controller.get("type") != "omni"
        or controller.get("navigation_2d") is not True
        or not all(
            isinstance(controller.get(key), (int, float)) and controller[key] > 0
            for key in ("max_linear_vel", "max_linear_accel")
        )
    ):
        raise ValueError("Unsupported controller construction")
    _validate_pose(c["initial_pose"])
    asset = Path(c["robot_asset"])
    if not asset.is_file() or hashlib.sha256(asset.read_bytes()).hexdigest() != c["robot_asset_sha256"]:
        raise ValueError("Robot asset is missing or changed")
    step = state["sim"]["step"]
    if type(step) is not int or step < 1 or step > manifest["last_completed_step"]:
        raise ValueError("Invalid checkpoint step")
    if not math.isclose(state["sim"]["elapsed_time"], step * dt, rel_tol=0, abs_tol=dt * 1e-9):
        raise ValueError("Checkpoint step/time mismatch")
    if state["sim"]["type"] != "kinematic" or state["sim"]["version"] != 1:
        raise ValueError("Unsupported simulation state type or version")
    robot_entry = state["agents"][c["robot_name"]]
    if robot_entry["type"] != "mobile_manipulator" or robot_entry["version"] != 1:
        raise ValueError("Unsupported agent state type or version")
    robot = _keys(
        robot_entry["state"],
        {"key", "pose", "velocity", "angular_velocity", "moving", "controller", "joints"},
        "robot",
    )
    if robot["key"] != c["robot_name"] or type(robot["moving"]) is not bool:
        raise ValueError("Robot checkpoint identity or movement flag is invalid")
    _validate_pose(robot["pose"])
    _vector(robot["velocity"], 3, "velocity")
    if not isinstance(robot["angular_velocity"], (float, int)) or not math.isfinite(robot["angular_velocity"]):
        raise ValueError("Invalid angular velocity")
    joints = robot["joints"]
    if not isinstance(joints, list) or not joints:
        raise ValueError("Invalid joint checkpoint")
    joint_names = set()
    for entry in joints:
        joint = _keys(entry, {"name", "position", "target"}, "joint")
        name = joint["name"]
        if not isinstance(name, str) or not name or name in joint_names:
            raise ValueError("Invalid or duplicate joint name")
        joint_names.add(name)
        for key in ("position", "target"):
            value = joint[key]
            if value is not None or key == "position":
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                    raise ValueError("Invalid joint position or target")
    if c["joint_name"] not in joint_names:
        raise ValueError("Declared scenario joint is missing from checkpoint")
    controller_entry = _keys(robot["controller"], {"type", "version", "state"}, "controller")
    if controller_entry["type"] != "omni" or controller_entry["version"] != 1:
        raise ValueError("Unsupported controller state type or version")
    if robot["moving"] != (controller_entry["state"] is not None):
        raise ValueError("Moving flag and controller state disagree")
    if controller_entry["state"] is not None:
        navigation = _keys(
            controller_entry["state"],
            {
                "path",
                "waypoint_index",
                "goal",
                "align_final_orientation",
                "final_target_orientation",
                "final_alignment_in_progress",
                "phase",
                "forward",
                "rotation",
            },
            "navigation",
        )
        path = navigation["path"]
        if not isinstance(path, list) or any(_validate_pose(pose) is not None for pose in path):
            raise ValueError("Invalid navigation path")
        _validate_pose(navigation["goal"])
        index = navigation["waypoint_index"]
        final_alignment = navigation["final_alignment_in_progress"]
        if (
            type(index) is not int
            or index < 0
            or type(final_alignment) is not bool
            or type(navigation["align_final_orientation"]) is not bool
            or (not final_alignment and (not path or index >= len(path) or navigation["goal"] != path[index]))
            or (final_alignment and path)
        ):
            raise ValueError("Invalid navigation waypoint state")
        target = navigation["final_target_orientation"]
        if target is not None:
            _vector(target, 4, "final target orientation")
        if navigation["align_final_orientation"] and target is None:
            raise ValueError("Final alignment target is missing")
        phase = navigation["phase"]
        if phase not in ("forward", "rotate"):
            raise ValueError("Invalid navigation phase")
        if phase == "forward":
            forward = _keys(navigation["forward"], {"origin", "t0", "vmax", "accel", "orientation_after_rotation"}, "forward")
            if forward["origin"] is not None:
                _vector(forward["origin"], 3, "forward origin")
            elif not final_alignment:
                raise ValueError("Forward origin is missing")
            if forward["orientation_after_rotation"] is not None:
                _vector(forward["orientation_after_rotation"], 4, "forward orientation")
            if navigation["rotation"] is not None:
                raise ValueError("Forward phase has active rotation")
            trajectory = forward
        else:
            if navigation["forward"] is not None:
                raise ValueError("Rotation phase has active forward trajectory")
            rotation = navigation["rotation"]
            if rotation is not None:
                rotation = _keys(rotation, {"start_orientation", "target_orientation", "t0", "vmax", "accel"}, "rotation")
                _vector(rotation["start_orientation"], 4, "rotation start")
                _vector(rotation["target_orientation"], 4, "rotation target")
            trajectory = rotation
        if trajectory is not None:
            for key in ("t0", "vmax", "accel"):
                value = trajectory[key]
                if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                    raise ValueError("Invalid navigation trajectory")
            if trajectory["vmax"] <= 0 or trajectory["accel"] <= 0:
                raise ValueError("Invalid navigation limits")
    if set(state["agents"]) != {c["robot_name"]} or set(state["objects"]) - {c["box_name"]}:
        raise ValueError("Checkpoint contains undeclared entities")
    box_entry = state["objects"].get(c["box_name"])
    if box_entry is not None and (box_entry["type"] != "kinematic_box" or box_entry["version"] != 1):
        raise ValueError("Unsupported object state type or version")
    box = box_entry["state"] if box_entry is not None else None
    if box is not None:
        box = _keys(box, {"key", "pose", "attachment", "construction"}, "box")
        if box["key"] != c["box_name"]:
            raise ValueError("Box checkpoint identity mismatch")
        _validate_pose(box["pose"])
        shape = _keys(
            box["construction"],
            {"visual_half_extents", "collision_half_extents", "visual_color", "pickable"},
            "box construction",
        )
        for key in ("visual_half_extents", "collision_half_extents"):
            if any(item <= 0 for item in _vector(shape[key], 3, key)):
                raise ValueError("Invalid box dimensions")
        _vector(shape["visual_color"], 4, "visual_color")
        if type(shape["pickable"]) is not bool:
            raise ValueError("Invalid pickable flag")
        if box["attachment"] is not None:
            attachment = _keys(
                box["attachment"],
                {"parent_name", "parent_link", "relative_position", "relative_orientation"},
                "attachment",
            )
            if attachment["parent_name"] != c["robot_name"] or not isinstance(attachment["parent_link"], str):
                raise ValueError("Invalid attachment parent")
            _vector(attachment["relative_position"], 3, "relative_position")
            relative_orientation = _vector(attachment["relative_orientation"], 4, "relative_orientation")
            if not math.isclose(sum(part * part for part in relative_orientation), 1.0, abs_tol=1e-5):
                raise ValueError("Invalid attachment quaternion")


def _validate_pose(value: Any) -> None:
    pose = _keys(value, {"position", "orientation"}, "pose")
    _vector(pose["position"], 3, "position")
    orientation = _vector(pose["orientation"], 4, "orientation")
    if not math.isclose(sum(part * part for part in orientation), 1.0, abs_tol=1e-5):
        raise ValueError("Invalid pose quaternion")


class KinematicManipulationProfile:
    """Validation example's supported omni robot, joints, and box profile."""

    profile_id = PROFILE
    version = VERSION
    coverage = {
        "inputs": "FleetCommandDispatcher navigate/joint/attach/stop and box spawn/remove",
        "results": "each completed step of this manipulation example",
        "checkpoints": "omni robot, joints, box and required scenario data at configured cadence",
        "unsupported": "other controllers/entities, Actions, plugins, physics and ROS/RMF",
    }

    def __init__(
        self,
        *,
        robot_name: str,
        joint_name: str,
        box_name: str,
        robot_asset: str,
        controller: dict,
        initial_pose: Pose,
        allowed_event_handlers: tuple[Callable, ...] = (),
    ) -> None:
        self.robot_name = robot_name
        self.joint_name = joint_name
        self.box_name = box_name
        self.robot_asset = str(Path(robot_asset).resolve())
        self.controller = dict(controller)
        self.initial_pose = initial_pose
        self.allowed_event_handlers = allowed_event_handlers

    @classmethod
    def from_manifest(cls, manifest: dict) -> "KinematicManipulationProfile":
        c = manifest["construction"]
        return cls(
            robot_name=c["robot_name"],
            joint_name=c["joint_name"],
            box_name=c["box_name"],
            robot_asset=c["robot_asset"],
            controller=c["controller"],
            initial_pose=_pose_from_record(c["initial_pose"]),
        )

    def validate_checkpoint(self, manifest: dict, state: dict) -> None:
        _validate_checkpoint(manifest, state)

    def spawn_record(self, obj: SimObject, params: SimObjectSpawnParams) -> dict | None:
        if obj.name != self.box_name:
            raise ValueError("Spawned object is outside the recording profile")
        if params.visual_shape is None or params.collision_shape is None:
            raise ValueError("Spawn lacks supported shape parameters")
        return {
            "key": obj.name,
            "pose": _pose_record(params.initial_pose),
            "visual_half_extents": list(params.visual_shape.half_extents),
            "collision_half_extents": list(params.collision_shape.half_extents),
        }

    def construction(self, sim: MultiRobotSimulationCore) -> dict:
        if sim.params.physics or sim.params.collision_check_frequency != 0:
            raise ValueError("This profile requires physics and collision checks disabled")
        if len(sim.agents) != 1 or type(sim.agents[0]) is not Agent or sim.agents[0].name != self.robot_name:
            raise ValueError("Recording must start with the declared robot")
        robot = sim.agents[0]
        from pybullet_fleet.controller import OmniController

        if (
            type(robot.controller) is not OmniController
            or Path(robot.urdf_path).resolve() != Path(self.robot_asset)
            or _pose_record(robot.get_pose()) != _pose_record(self.initial_pose)
            or robot.controller.params.navigation_2d != self.controller.get("navigation_2d")
            or robot.controller.params.max_linear_vel != self.controller.get("max_linear_vel")
            or robot.controller.params.max_linear_accel != self.controller.get("max_linear_accel")
        ):
            raise ValueError("Declared recording construction differs from the live robot")
        asset = Path(self.robot_asset)
        if not asset.is_file():
            raise ValueError("Robot asset is missing")
        return {
            "timestep": sim.params.timestep,
            "physics": sim.params.physics,
            "enable_floor": sim.params.enable_floor,
            "collision_check_frequency": sim.params.collision_check_frequency,
            "robot_name": self.robot_name,
            "joint_name": self.joint_name,
            "box_name": self.box_name,
            "robot_asset": self.robot_asset,
            "robot_asset_sha256": hashlib.sha256(asset.read_bytes()).hexdigest(),
            "controller": self.controller,
            "robot_mass": robot.mass,
            "use_fixed_base": robot.use_fixed_base,
            "robot_collision_mode": robot.collision_mode.value,
            "initial_pose": _pose_record(self.initial_pose),
        }

    def capture(self, sim: MultiRobotSimulationCore) -> dict:
        if sim._plugins or sim._callbacks or sim._behavior_tree_callbacks:
            raise ValueError("Undeclared simulation plugins/callbacks are not checkpointable")
        allowed = {id(handler) for handler in self.allowed_event_handlers}
        if any(id(handler) not in allowed for handlers in sim.events._handlers.values() for _, handler in handlers):
            raise ValueError("Undeclared event handler is not checkpointable")
        if len(sim.agents) != 1 or type(sim.agents[0]) is not Agent or sim.agents[0].name != self.robot_name:
            raise ValueError("Checkpoint requires exactly the declared robot")
        robot = sim.agents[0]
        if robot.plugins or robot.callbacks or not robot.is_action_queue_empty() or len(robot.controllers) != 1:
            raise ValueError("Agent plugins, Actions or controller chains are not checkpointable")
        if robot._events is not None and robot._events._handlers:
            raise ValueError("Agent event handlers are outside the checkpoint profile")
        from pybullet_fleet.controller import OmniController

        if type(robot.controller) is not OmniController:
            raise ValueError("Checkpoint requires a built-in omni controller")
        boxes = [obj for obj in sim.sim_objects if obj is not robot]
        if len(boxes) > 1 or (boxes and (type(boxes[0]) is not SimObject or boxes[0].name != self.box_name)):
            raise ValueError("Checkpoint requires at most the declared box")
        step, elapsed_time = sim.get_completed_step_boundary()
        pose = robot.get_pose()
        controller_state = robot.controller.capture_navigation_state() if robot.is_moving else None
        box_state = None
        if boxes:
            box = boxes[0]
            if box.callbacks:
                raise ValueError("Box callbacks are outside the checkpoint profile")
            if box._events is not None and box._events._handlers:
                raise ValueError("Box event handlers are outside the checkpoint profile")
            if box.mass != 0 or box.collision_mode != CollisionMode.DISABLED:
                raise ValueError("Unsupported box physics/collision mode")
            params = getattr(box, "_checkpoint_spawn_params", None)
            if (
                params is None
                or params.visual_shape is None
                or params.collision_shape is None
                or params.visual_shape.shape_type != "box"
                or params.collision_shape.shape_type != "box"
            ):
                raise ValueError("Box lacks supported construction parameters")
            box_state = {
                "key": self.box_name,
                "pose": _pose_record(box.get_pose()),
                "attachment": box.get_kinematic_attachment_state(),
                "construction": {
                    "visual_half_extents": list(params.visual_shape.half_extents),
                    "collision_half_extents": list(params.collision_shape.half_extents),
                    "visual_color": list(params.visual_shape.rgba_color),
                    "pickable": params.pickable,
                },
            }
        state = {
            "sim": {"type": "kinematic", "version": 1, "step": step, "elapsed_time": elapsed_time},
            "agents": {
                self.robot_name: {
                    "type": "mobile_manipulator",
                    "version": 1,
                    "state": {
                        "key": self.robot_name,
                        "pose": _pose_record(pose),
                        "velocity": robot.get_velocity().tolist(),
                        "angular_velocity": float(robot.angular_velocity),
                        "moving": robot.is_moving,
                        "controller": {"type": "omni", "version": 1, "state": controller_state},
                        "joints": robot.capture_kinematic_joint_execution(),
                    },
                }
            },
            "objects": (
                {self.box_name: {"type": "kinematic_box", "version": 1, "state": box_state}} if box_state is not None else {}
            ),
        }
        return state

    def restore(self, manifest: dict, state: dict, *, gui: bool, target_rtf: float) -> MultiRobotSimulationCore:
        return _restore_manipulation_simulation(manifest, state, gui=gui, target_rtf=target_rtf)

    def play_gui(self, playback: ResultPlayback, *, rtf: float, hold: bool) -> None:
        """Render this validation profile's recorded poses without stepping."""
        manifest = playback.manifest
        checkpoint_steps = sorted(int(step) for step in manifest["checkpoint_sha256"])
        if not checkpoint_steps:
            raise ValueError("GUI playback requires a checkpoint")
        first_step = checkpoint_steps[0]
        first_path = playback.path / "checkpoints" / f"{first_step:09d}.json"
        from pybullet_fleet.replay.state_recording import _read_json, _sha256

        first_state = _read_json(first_path)
        if _sha256(first_path) != manifest["checkpoint_sha256"].get(str(first_step)):
            raise ValueError("First checkpoint integrity check failed")
        sim = restore_supported_simulation(manifest, first_state, profile=self, gui=True)
        robot = sim.get_unique_agent_by_name(self.robot_name)
        boxes = sim.find_objects_by_name(self.box_name)
        box = boxes[0] if boxes else None
        try:
            sim.setup_camera(
                camera_config={
                    "camera_mode": "manual",
                    "camera_distance": 3.0,
                    "camera_yaw": 45,
                    "camera_pitch": -35,
                    "camera_target": [0.5, 0, 0.7],
                }
            )
            for frame in playback.frames:
                robot_state = frame["agents"][self.robot_name]["state"]
                robot.set_pose(_pose_from_record(robot_state["pose"]))
                robot.restore_kinematic_joint_execution(robot_state["joints"])
                box_entry = frame["objects"].get(self.box_name)
                box_state = box_entry["state"] if box_entry is not None else None
                if box_state is None and box is not None:
                    sim.remove_object(box)
                    box = None
                elif box_state is not None:
                    if box is None:
                        raise ValueError("Playback profile does not support respawn after deletion")
                    box.set_pose(_pose_from_record(box_state["pose"]))
                time.sleep(manifest["construction"]["timestep"] / rtf)
            if hold:
                while p.isConnected(sim.client):
                    time.sleep(0.1)
        finally:
            if p.isConnected(sim.client):
                p.disconnect(sim.client)


def _restore_manipulation_simulation(
    manifest: dict, state: dict, *, gui: bool = False, target_rtf: float = 0.0
) -> MultiRobotSimulationCore:
    """Reconstruct the concrete manipulation example's supported world."""
    _validate_checkpoint(manifest, state)
    c = manifest["construction"]
    sim = MultiRobotSimulationCore(
        SimulationParams(
            gui=gui,
            monitor=False,
            enable_monitor_gui=False,
            physics=c["physics"],
            enable_floor=c["enable_floor"],
            timestep=c["timestep"],
            target_rtf=target_rtf,
            collision_check_frequency=c["collision_check_frequency"],
            log_level="warning",
        )
    )
    try:
        robot = Agent.from_params(
            AgentSpawnParams(
                name=c["robot_name"],
                urdf_path=c["robot_asset"],
                initial_pose=_pose_from_record(c["initial_pose"]),
                mass=c["robot_mass"],
                use_fixed_base=c["use_fixed_base"],
                collision_mode=CollisionMode(c["robot_collision_mode"]),
                controller=c["controller"],
            ),
            sim,
        )
        sim.initialize_simulation()
        robot_state = state["agents"][c["robot_name"]]["state"]
        if robot_state["key"] != c["robot_name"] or c["joint_name"] not in {joint["name"] for joint in robot_state["joints"]}:
            raise ValueError("Checkpoint identity mismatch")
        box_entry = state["objects"].get(c["box_name"])
        box_state = box_entry["state"] if box_entry is not None else None
        box = None
        if box_state is not None:
            if box_state["key"] != c["box_name"]:
                raise ValueError("Checkpoint box identity mismatch")
            box_construction = box_state["construction"]
            box = SimObject.from_params(
                SimObjectSpawnParams(
                    name=c["box_name"],
                    initial_pose=_pose_from_record(box_state["pose"]),
                    mass=0.0,
                    pickable=box_construction["pickable"],
                    collision_mode=CollisionMode.DISABLED,
                    visual_shape=ShapeParams(
                        shape_type="box",
                        half_extents=box_construction["visual_half_extents"],
                        rgba_color=box_construction["visual_color"],
                    ),
                    collision_shape=ShapeParams(shape_type="box", half_extents=box_construction["collision_half_extents"]),
                ),
                sim,
            )
        robot.restore_kinematic_joint_execution(robot_state["joints"])
        pose = _pose_from_record(robot_state["pose"])
        robot.restore_motion_state(
            pose, robot_state["velocity"], robot_state["angular_velocity"], moving=robot_state["moving"]
        )
        if robot_state["controller"]["state"] is not None:
            robot.controller.restore_navigation_state(robot_state["controller"]["state"], pose.orientation)
        elif robot_state["moving"]:
            raise ValueError("Moving robot is missing controller execution state")
        if box_state is not None and box_state["attachment"] is not None:
            if box is None:
                raise ValueError("Attached checkpoint box was not reconstructed")
            attachment = box_state["attachment"]
            if attachment["parent_name"] != robot.name:
                raise ValueError("Attachment parent identity mismatch")
            link_names = {
                p.getJointInfo(robot.body_id, index, physicsClientId=sim.client)[12].decode("utf-8")
                for index in range(p.getNumJoints(robot.body_id, physicsClientId=sim.client))
            }
            if attachment["parent_link"] != "base_link" and attachment["parent_link"] not in link_names:
                raise ValueError("Attachment link is absent from the restored robot")
            if not robot.attach_object(
                box,
                parent_link_index=attachment["parent_link"],
                relative_pose=Pose(position=attachment["relative_position"], orientation=attachment["relative_orientation"]),
            ):
                raise ValueError("Attachment restoration failed")
        sim.restore_completed_step_boundary(state["sim"]["step"], state["sim"]["elapsed_time"])
        return sim
    except BaseException:
        if p.isConnected(sim.client):
            p.disconnect(sim.client)
        raise
