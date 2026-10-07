"""Inspect, record, restore and play back a supported mobile-manipulator run.

The default observation run remains separate from the opt-in omni recording
profile. Scenario stages are application state, not a general PBF task model.
"""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import pybullet as p

from pybullet_fleet.agent import Agent, AgentSpawnParams
from pybullet_fleet.commands import RobotAttachCommand, RobotGoalCommand2D, RobotNamedJointPositionsCommand
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.events import SimEvents
from pybullet_fleet.fleet_api import FleetCommandDispatcher, FleetStateProvider
from pybullet_fleet.geometry import Pose
from pybullet_fleet.sim_object import ShapeParams, SimObject, SimObjectSpawnParams
from pybullet_fleet.types import CollisionMode
from pybullet_fleet.examples.validation.manipulation_recording_profile import KinematicManipulationProfile

_ROBOT_NAME = "mobile-arm"
_BOX_ENTITY_ID = "box-001"  # Driver-owned identity; runtime object_id is not durable.
_JOINT = "shoulder_to_elbow"
_PICK_TARGET = 0.8
_CARRY_TARGET = -0.6
_BOX_OFFSET = Pose.from_xyz(0, 0, 0.07)


def _hold_final_gui(sim: MultiRobotSimulationCore) -> None:
    print("Simulation finished. Close the GUI window or press Ctrl+C to exit.")
    try:
        while p.isConnected(sim.client):
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass


def run_scenario(
    *,
    gui: bool = False,
    view_rtf: float = 1.0,
    hold_gui: bool = False,
    record_output: str | None = None,
    omni_profile: bool = False,
) -> dict:
    """Run explicit stages using existing PBF APIs and return in-memory facts."""
    if gui and (not math.isfinite(view_rtf) or view_rtf <= 0):
        raise ValueError("view_rtf must be finite and positive in GUI mode")
    if hold_gui and not gui:
        raise ValueError("hold_gui requires gui=True")

    dt = 0.1
    sim = MultiRobotSimulationCore(
        SimulationParams(
            gui=gui,
            monitor=False,
            enable_monitor_gui=False,
            physics=False,
            enable_floor=False,
            timestep=dt,
            target_rtf=view_rtf if gui else 0,
            collision_check_frequency=0,
            log_level="warning",
        )
    )
    controller = (
        {"type": "omni", "navigation_2d": True, "max_linear_vel": 1.0, "max_linear_accel": 2.0}
        if record_output is not None or omni_profile
        else {
            "type": "differential",
            "max_linear_vel": 1.0,
            "max_linear_accel": 2.0,
            "max_angular_vel": 1.5,
            "max_angular_accel": 3.0,
        }
    )
    try:
        robot = Agent.from_params(
            AgentSpawnParams(
                name=_ROBOT_NAME,
                urdf_path=str(Path(__file__).resolve().parents[2] / "robots" / "mobile_manipulator.urdf"),
                initial_pose=Pose.from_xyz(0, 0, 0.3),
                mass=0.0,
                use_fixed_base=False,
                controller=controller,
            ),
            sim,
        )
        return _run_scenario_with_core(
            sim,
            dt=dt,
            gui=gui,
            hold_gui=hold_gui,
            robot=robot,
            box=None,
            controller=controller,
            application_state={},
            record_output=record_output,
            duration=5.0 if record_output is not None or omni_profile else 3.0,
            resume=False,
            record_initial_observation=True,
            omni_profile=omni_profile,
        )
    finally:
        if p.isConnected(sim.client):
            p.disconnect(sim.client)


def _run_scenario_with_core(
    sim: MultiRobotSimulationCore,
    *,
    dt: float,
    gui: bool,
    hold_gui: bool,
    robot: Agent,
    box: SimObject | None,
    controller: dict | None,
    application_state: dict,
    duration: float,
    resume: bool,
    record_initial_observation: bool,
    omni_profile: bool,
    record_output: str | None = None,
) -> dict:
    """Run the stage logic using a freshly constructed or restored scenario state."""
    if gui:
        sim.setup_camera(
            camera_config={
                "camera_mode": "manual",
                "camera_distance": 3.0,
                "camera_yaw": 45,
                "camera_pitch": -35,
                "camera_target": [0.5, 0, 0.7],
            }
        )

    dispatcher = FleetCommandDispatcher(sim)
    provider = FleetStateProvider(sim)
    application = application_state
    stage = application.get("stage", "spawn")
    observations: dict[str, dict] = dict(application.get("observations", {}))
    lifecycle: list[dict] = list(application.get("lifecycle", []))
    identity: dict[str, int] = {_BOX_ENTITY_ID: box.object_id} if box is not None else {}
    trajectory: list[dict] = []
    callback_error: list[Exception] = []
    callback_phase = "outside_step"

    def lifecycle_record(operation: str, obj: SimObject) -> dict:
        completed_step = sim.step_count + (callback_phase == "POST_STEP")
        return {
            "operation": operation,
            "phase": callback_phase,
            "counter_during_callback": sim.step_count,
            "completed_step": completed_step,
            "sim_time": round(completed_step * dt, 10),
            "object_id": obj.object_id,
        }

    def observe(label: str, step: int) -> None:
        present = box is not None and box in sim.sim_objects
        state = provider.get_states_3d([_ROBOT_NAME])[0]
        robot_pose = robot.get_pose()
        box_pose = box.get_pose() if present and box is not None else None
        observations[label] = {
            "step": step,
            "sim_time": round(step * dt, 10),
            "robot_object_id": robot.object_id,
            "base_position": list(robot_pose.position),
            "base_is_moving": state.is_moving,
            "joint_position": robot.get_joint_state_by_name(_JOINT)[0],
            "joint_reported_velocity": robot.get_joint_state_by_name(_JOINT)[1],
            "box_present": present,
            "box_runtime_object_id": box.object_id if present and box is not None else None,
            "box_position": list(box_pose.position) if box_pose is not None else None,
            "box_attached": box.is_attached() if present and box is not None else False,
            "attached_children": [child.object_id for child in robot.get_attached_objects()],
        }

    def on_spawn(*, obj: SimObject) -> None:
        if obj.name == _BOX_ENTITY_ID:
            lifecycle.append(lifecycle_record("spawn", obj))

    def on_remove(*, obj: SimObject) -> None:
        if obj.name == _BOX_ENTITY_ID:
            lifecycle.append(lifecycle_record("delete", obj))

    def require_ack(ack, action: str) -> None:
        if _ROBOT_NAME not in ack.accepted_names:
            raise RuntimeError(f"{action} rejected: {dict(ack.rejected)}")

    def before_step(**_: object) -> None:
        nonlocal box, stage, callback_phase
        if callback_error or sim.step_count != 0:
            return
        try:
            callback_phase = "PRE_STEP"
            shape = ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05])
            box = SimObject.from_params(
                SimObjectSpawnParams(
                    name=_BOX_ENTITY_ID,
                    initial_pose=Pose.from_xyz(1.0, 0, 0.7),
                    mass=0.0,
                    pickable=True,
                    collision_mode=CollisionMode.DISABLED,
                    visual_shape=shape,
                    collision_shape=shape,
                ),
                sim,
            )
            identity[_BOX_ENTITY_ID] = box.object_id
            stage = "base_motion"
        except Exception as exc:
            callback_error.append(exc)

    def after_step(**_: object) -> None:
        nonlocal box, stage, callback_phase
        if callback_error:
            return
        try:
            callback_phase = "POST_STEP"
            # POST_STEP has the new physical state but core counters still
            # identify the preceding completed step until this callback ends.
            step = sim.step_count + 1
            if stage == "base_motion":
                if step == 1:
                    require_ack(
                        dispatcher.navigate(
                            [
                                RobotGoalCommand2D(
                                    name=_ROBOT_NAME,
                                    position=(0.6, 0),
                                    yaw=0.4 if record_output is not None or omni_profile else 0.0,
                                    z=0.3,
                                )
                            ],
                            command_id="base-to-box",
                        ),
                        "base navigate",
                    )
                elif step == 3:
                    observe("CP1", step)
                elif step > 3 and not robot.is_moving:
                    require_ack(
                        dispatcher.joint_command(
                            [RobotNamedJointPositionsCommand(_ROBOT_NAME, {_JOINT: _PICK_TARGET})],
                            command_id="arm-to-pick",
                        ),
                        "arm target",
                    )
                    stage = "arm_motion"
            elif stage == "arm_motion":
                if "CP2" not in observations:
                    observe("CP2", step)
                elif robot.are_joints_at_targets_by_name({_JOINT: _PICK_TARGET}, tolerance=0.01):
                    require_ack(
                        dispatcher.attach(
                            [
                                RobotAttachCommand(
                                    _ROBOT_NAME,
                                    attach=True,
                                    object_name=_BOX_ENTITY_ID,
                                    parent_link="end_effector",
                                    offset=_BOX_OFFSET,
                                )
                            ],
                            command_id="attach-box",
                        ),
                        "attach",
                    )
                    require_ack(
                        dispatcher.joint_command(
                            [RobotNamedJointPositionsCommand(_ROBOT_NAME, {_JOINT: _CARRY_TARGET})],
                            command_id="arm-carry",
                        ),
                        "carry target",
                    )
                    stage = "attached_motion"
            elif stage == "attached_motion":
                if "CP3" not in observations:
                    observe("CP3", step)
                elif robot.are_joints_at_targets_by_name({_JOINT: _CARRY_TARGET}, tolerance=0.01):
                    require_ack(
                        dispatcher.attach(
                            [RobotAttachCommand(_ROBOT_NAME, attach=False, object_name=_BOX_ENTITY_ID)],
                            command_id="detach-box",
                        ),
                        "detach",
                    )
                    observe("CP4", step)
                    stage = "delete"
            elif stage == "delete":
                if box is None:
                    raise RuntimeError("box missing before deletion")
                sim.remove_object(box)
                if box in sim.sim_objects or box.is_attached() or box in robot.get_attached_objects():
                    raise RuntimeError("removed box remains live or attached")
                box = None
                observe("after_delete", step)
                stage = "done"
                if hold_gui:
                    _hold_final_gui(sim)
            current_box = box if box is not None and box in sim.sim_objects else None
            trajectory.append(
                {
                    "step": step,
                    "sim_time": round(step * dt, 10),
                    "base_position": list(robot.get_pose().position),
                    "base_orientation": list(robot.get_pose().orientation),
                    "base_is_moving": robot.is_moving,
                    "joint_position": robot.get_joint_state_by_name(_JOINT)[0],
                    "box_position": list(current_box.get_pose().position) if current_box is not None else None,
                    "box_attached": current_box.is_attached() if current_box is not None else False,
                    "stage": stage,
                }
            )
        except Exception as exc:
            callback_error.append(exc)

    sim.events.on(SimEvents.OBJECT_SPAWNED, on_spawn)
    sim.events.on(SimEvents.OBJECT_REMOVED, on_remove)
    sim.events.on(SimEvents.PRE_STEP, before_step)
    sim.events.on(SimEvents.POST_STEP, after_step)
    if record_initial_observation:
        observe("before_spawn", 0)
    state_recorder = None
    if record_output is not None:
        from pybullet_fleet.replay.state_recording import DataRecord

        def capture_application_state(_context) -> dict:
            # EventBus logs handler exceptions. A durable recording must fail
            # visibly instead of marking that incomplete execution as complete.
            if callback_error:
                raise RuntimeError("Scenario callback failed before checkpoint") from callback_error[0]
            return {"stage": stage, "observations": observations, "lifecycle": lifecycle}

        state_recorder = sim.configure_state_recording(
            output=record_output,
            profile=KinematicManipulationProfile(
                robot_name=_ROBOT_NAME,
                joint_name=_JOINT,
                box_name=_BOX_ENTITY_ID,
                robot_asset=str(Path(__file__).resolve().parents[2] / "robots" / "mobile_manipulator.urdf"),
                controller=controller,
                initial_pose=Pose.from_xyz(0, 0, 0.3),
                allowed_event_handlers=(on_spawn, on_remove, before_step, after_step),
            ),
            records=(DataRecord("scenario", 1, capture_application_state, required_for_restore=True),),
        )
    sim.run_simulation(
        duration=duration,
        resume=resume,
    )
    if callback_error:
        raise RuntimeError("scenario callback failed") from callback_error[0]
    if stage != "done":
        raise RuntimeError(f"scenario did not finish: {stage}")
    return {
        "observations": observations,
        "lifecycle": lifecycle,
        "identity": identity,
        "trajectory": trajectory,
        "final_stage": stage,
        "recording_path": str(state_recorder.path) if state_recorder is not None else None,
    }


def resume_scenario(
    directory: str, *, at_or_before: float, gui: bool = False, view_rtf: float = 1.0, hold_gui: bool = False
) -> dict:
    """Restore a supported run through PBF tooling, then resume its application."""
    from pybullet_fleet.replay.state_recording import (
        load_recording_manifest,
        load_supported_checkpoint,
        restore_supported_simulation,
    )

    manifest = load_recording_manifest(directory)
    profile = KinematicManipulationProfile.from_manifest(manifest)
    manifest, state = load_supported_checkpoint(directory, at_or_before=at_or_before, profile=profile)
    sim = restore_supported_simulation(manifest, state, profile=profile, gui=gui, target_rtf=view_rtf if gui else 0.0)
    robot = sim.get_unique_agent_by_name(_ROBOT_NAME)
    boxes = sim.find_objects_by_name(_BOX_ENTITY_ID)
    if len(boxes) > 1:
        raise LookupError(f"Expected at most one object named {_BOX_ENTITY_ID!r}; found {len(boxes)}")
    box = boxes[0] if boxes else None
    application = state["records"]["scenario"]["value"]
    try:
        return _run_scenario_with_core(
            sim,
            dt=manifest["construction"]["timestep"],
            gui=gui,
            hold_gui=hold_gui,
            robot=robot,
            box=box,
            controller=None,
            application_state=application,
            duration=5.0,
            resume=True,
            record_initial_observation=False,
            omni_profile=True,
        )
    finally:
        if p.isConnected(sim.client):
            p.disconnect(sim.client)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gui", action="store_true", help="Show the PyBullet GUI")
    parser.add_argument("--rtf", type=float, default=1.0, help="GUI viewing real-time factor")
    parser.add_argument("--hold-gui", action="store_true", help="Keep the final GUI frame open")
    operations = parser.add_mutually_exclusive_group()
    operations.add_argument(
        "--record", nargs="?", const="", metavar="DIR", help="Record a supported run (omit DIR for an automatic name)"
    )
    operations.add_argument("--restore", metavar="DIR", help="Resume a supported checkpoint in this recording")
    operations.add_argument("--playback", metavar="DIR", help="Read recorded results without executing the simulation")
    parser.add_argument("--time", type=float, help="Select the latest checkpoint at or before this simulated time")
    args = parser.parse_args()
    if args.playback is not None:
        from pybullet_fleet.replay.state_recording import ResultPlayback, load_recording_manifest

        profile = KinematicManipulationProfile.from_manifest(load_recording_manifest(args.playback))
        playback = ResultPlayback(args.playback, profile=profile)
        if args.gui:
            playback.play_gui(rtf=args.rtf, hold=args.hold_gui)
        else:
            for frame in playback:
                print(
                    f"step={frame['step']} time={frame['elapsed_time']:.1f}s "
                    f"box_present={_BOX_ENTITY_ID in frame['objects']}"
                )
        return
    if args.restore is not None:
        if args.time is None:
            raise ValueError("--restore requires --time in this supported profile")
        result = resume_scenario(args.restore, at_or_before=args.time, gui=args.gui, view_rtf=args.rtf, hold_gui=args.hold_gui)
    else:
        output = args.record
        if output is None:
            result = run_scenario(gui=args.gui, view_rtf=args.rtf, hold_gui=args.hold_gui)
        else:
            result = run_scenario(gui=args.gui, view_rtf=args.rtf, hold_gui=args.hold_gui, record_output=output)
            print(f"State recording: {result['recording_path']}")
    for label, observation in result["observations"].items():
        print(
            f"{label}: step={observation['step']} time={observation['sim_time']:.1f}s "
            f"box_present={observation['box_present']} attached={observation['box_attached']}"
        )


if __name__ == "__main__":
    main()
