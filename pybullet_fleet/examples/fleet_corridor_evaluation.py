"""Synthetic corridor experiment driven by an external fleet-management app.

Run ``python -m pybullet_fleet.examples.fleet_corridor_evaluation OUTPUT_DIR``.
The policies, task ledger and verdict-free metrics live here, not in PBF core.
This is not a replay, historical incident, or warehouse delivery workload.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from dataclasses import asdict, dataclass
from itertools import combinations
from pathlib import Path
from uuid import uuid4

import pybullet as p

from pybullet_fleet.agent import Agent, AgentSpawnParams
from pybullet_fleet.commands import RobotGoalCommand2D
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.events import SimEvents
from pybullet_fleet.fleet_api import FleetCommandDispatcher, FleetStateProvider
from pybullet_fleet.geometry import Pose
from pybullet_fleet.sim_object import ShapeParams, SimObject, SimObjectSpawnParams
from pybullet_fleet.types import CollisionDetectionMethod, CollisionMode


@dataclass(frozen=True)
class CorridorConfig:
    timestep: float = 0.1
    cutoff: float = 300.0
    margin: float = 0.02
    speed: float = 1.0
    position_tolerance: float = 0.02

    def __post_init__(self) -> None:
        for label in ("timestep", "cutoff", "speed", "position_tolerance"):
            value = getattr(self, label)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{label} must be finite and positive")
        if not math.isfinite(self.margin) or self.margin < 0:
            raise ValueError("margin must be finite and nonnegative")
        steps = self.cutoff / self.timestep
        if not math.isclose(steps, round(steps), rel_tol=0, abs_tol=1e-8):
            raise ValueError("cutoff must be an integer number of steps")

    @property
    def steps(self) -> int:
        return round(self.cutoff / self.timestep)


def _quantiles(values: list[float]) -> dict:
    if not values:
        return {"count": 0, "min": None, "median": None, "p90": None, "max": None}
    ordered = sorted(values)
    p90_index = math.ceil(0.9 * len(ordered)) - 1
    return {
        "count": len(ordered),
        "min": ordered[0],
        "median": statistics.median(ordered),
        "p90": ordered[p90_index],
        "max": ordered[-1],
    }


def _workload() -> list[dict]:
    tasks = []
    for side in ("a", "b"):
        for index in range(10):
            name = f"{side}{index:02d}"
            lane_y = -0.075 if index % 2 == 0 else 0.075
            offset = 0.24 * index
            start_x = (-5.0 - offset) if side == "a" else (5.0 + offset)
            goal_x = (4.0 + offset) if side == "a" else (-4.0 - offset)
            for leg in (0, 1):
                tasks.append(
                    {
                        "task_id": f"{name}-leg{leg + 1}",
                        "robot_id": name,
                        "leg": leg,
                        "direction": "a_to_b" if (side == "a") == (leg == 0) else "b_to_a",
                        "start": [start_x, lane_y] if leg == 0 else [goal_x, lane_y],
                        "destination": [goal_x, lane_y] if leg == 0 else [start_x, lane_y],
                    }
                )
    return tasks


def _make_sim(
    config: CorridorConfig, *, gui: bool = False
) -> tuple[MultiRobotSimulationCore, dict[int, tuple[str, SimObject]]]:
    sim = MultiRobotSimulationCore(
        SimulationParams(
            gui=gui,
            monitor=False,
            enable_monitor_gui=False,
            enable_floor=False,
            physics=False,
            target_rtf=0,
            timestep=config.timestep,
            collision_check_frequency=None,
            collision_detection_method=CollisionDetectionMethod.CLOSEST_POINTS,
            collision_margin=config.margin,
            ignore_static_collision=False,
            log_level="warning",
        )
    )
    entities: dict[int, tuple[str, SimObject]] = {}
    model = Path(__file__).resolve().parents[1] / "robots" / "simple_cube.urdf"
    with sim.batch_spawn():
        for side in ("a", "b"):
            for index in range(10):
                name = f"{side}{index:02d}"
                x = (-5.0 - 0.24 * index) if side == "a" else (5.0 + 0.24 * index)
                y = -0.075 if index % 2 == 0 else 0.075
                robot = Agent.from_params(
                    AgentSpawnParams(
                        name=name,
                        urdf_path=str(model),
                        initial_pose=Pose.from_xyz(x, y, 0.1),
                        mass=0.0,
                        pickable=False,
                        collision_mode=CollisionMode.NORMAL_2D,
                        controller={
                            "type": "omni",
                            "navigation_2d": True,
                            "max_linear_vel": config.speed,
                            "max_linear_accel": 2.0,
                            "max_angular_vel": 2.0,
                            "max_angular_accel": 4.0,
                        },
                    ),
                    sim,
                )
                entities[robot.object_id] = (name, robot)
                if gui:
                    color = [0.15, 0.45, 0.95, 1.0] if side == "a" else [0.95, 0.45, 0.12, 1.0]
                    p.changeVisualShape(robot.body_id, -1, rgbaColor=color, physicsClientId=sim.client)
        for side, y in (("north", 0.22), ("south", -0.22)):
            shape = ShapeParams(shape_type="box", half_extents=[3.0, 0.05, 0.25])
            wall = SimObject.from_params(
                SimObjectSpawnParams(
                    name=f"wall-{side}",
                    initial_pose=Pose.from_xyz(0, y, 0.1),
                    mass=0.0,
                    pickable=False,
                    visual_shape=shape,
                    collision_shape=shape,
                    collision_mode=CollisionMode.STATIC,
                ),
                sim,
            )
            entities[wall.object_id] = (f"wall-{side}", wall)
    if gui:
        sim.setup_camera(
            camera_config={
                "camera_mode": "manual",
                "camera_distance": 18.0,
                "camera_yaw": 0,
                "camera_pitch": -89,
                "camera_target": [0, 0, 0.1],
            }
        )
    return sim, entities


def _category(distance: float, margin: float) -> str | None:
    if distance <= 0:
        return "geometric_overlap"
    if distance <= margin:
        return "margin_only"
    return None


def _observe_collisions(
    sim: MultiRobotSimulationCore,
    entities: dict[int, tuple[str, SimObject]],
    active: dict[tuple[str, str], dict],
    episodes: list[dict],
    margin: float,
    active_tasks: dict[str, dict] | None = None,
    observation_step: int | None = None,
) -> None:
    sample_step = sim.step_count if observation_step is None else observation_step
    observed: dict[tuple[str, str], tuple[str, float, bool, dict]] = {}
    # The core's AABB broadphase does not expand by collision_margin. Query all
    # pairs here so a positive-gap margin intrusion is not silently omitted.
    for (_, (name_a, obj_a)), (_, (name_b, obj_b)) in combinations(entities.items(), 2):
        if name_a.startswith("wall-") and name_b.startswith("wall-"):
            continue
        points = p.getClosestPoints(obj_a.body_id, obj_b.body_id, distance=margin, physicsClientId=sim.client)
        if not points:
            continue
        distance = min(float(point[8]) for point in points)
        category = _category(distance, margin)
        if category is not None:
            x_mid = (obj_a.get_pose().x + obj_b.get_pose().x) / 2
            observed[tuple(sorted((name_a, name_b)))] = (
                category,
                distance,
                -3.0 <= x_mid <= 3.0,
                {name_a: [obj_a.get_pose().x, obj_a.get_pose().y], name_b: [obj_b.get_pose().x, obj_b.get_pose().y]},
            )

    for pair, episode in tuple(active.items()):
        if pair not in observed or observed[pair][0] != episode["category"]:
            episode["end_step"] = sample_step
            episode["end_time"] = sample_step * sim.params.timestep
            episodes.append(episode)
            del active[pair]
    for pair, (category, distance, in_corridor, positions) in sorted(observed.items()):
        if pair not in active:
            active[pair] = {
                "entities": list(pair),
                "category": category,
                "start_step": sample_step,
                "start_time": sample_step * sim.params.timestep,
                "min_distance": distance,
                "start_positions_xy": positions,
                "active_task_ids_at_start": [
                    active_tasks[name]["task_id"] for name in pair if active_tasks and name in active_tasks
                ],
                "observed_steps": 0,
                "corridor_observed_steps": 0,
            }
        active[pair]["observed_steps"] += 1
        active[pair]["corridor_observed_steps"] += int(in_corridor)
        active[pair]["min_distance"] = min(active[pair]["min_distance"], distance)


def _ready(tasks: list[dict], active: dict[str, dict], completed: set[str]) -> list[dict]:
    return [
        task
        for task in tasks
        if task["robot_id"] not in active
        and task["task_id"] not in completed
        and task.get("issued_step") is None
        and (task["leg"] == 0 or f"{task['robot_id']}-leg1" in completed)
    ]


def _select(policy: str, ready: list[dict], active: dict[str, dict], turn: str) -> tuple[list[dict], str]:
    if policy == "uncontrolled":
        return ready, turn
    if active:
        direction = next(iter(active.values()))["direction"]
        slots = 2 - len(active)
    else:
        available = {task["direction"] for task in ready}
        if not available:
            return [], turn
        direction = (
            turn if turn in available else next(direction for direction in ("a_to_b", "b_to_a") if direction in available)
        )
        slots = 2
        turn = "b_to_a" if direction == "a_to_b" else "a_to_b"
    return [task for task in ready if task["direction"] == direction][: max(0, slots)], turn


def run_policy(
    policy: str,
    config: CorridorConfig = CorridorConfig(),
    *,
    collect_collision_episodes: bool = True,
    gui: bool = False,
    view_rtf: float = 1.0,
    hold_gui: bool = False,
) -> dict:
    """Run one policy from a fresh, equivalent scenario; never use replay commands."""
    if policy not in ("uncontrolled", "direction_gate"):
        raise ValueError("policy must be uncontrolled or direction_gate")
    if gui and (not math.isfinite(view_rtf) or view_rtf <= 0):
        raise ValueError("view_rtf must be finite and positive in GUI mode")
    if hold_gui and not gui:
        raise ValueError("hold_gui requires gui=True")
    sim, entities = _make_sim(config, gui=gui)
    dispatcher = FleetCommandDispatcher(sim, retain_command_events=False)
    provider = FleetStateProvider(sim)
    tasks = _workload()
    run_id = str(uuid4())
    active: dict[str, dict] = {}
    completed: set[str] = set()
    decisions: list[dict] = []
    episodes: list[dict] = []
    active_episodes: dict[tuple[str, str], dict] = {}
    turn = "a_to_b"
    wall_start = time.perf_counter()
    step_wall = 0.0
    corridor_peak_robots = 0
    corridor_over_capacity_steps = 0
    corridor_robot_steps = 0
    crowding_intervals: list[dict] = []
    active_crowding: dict | None = None
    callback_failure: list[Exception] = []
    step_started_wall = wall_start
    sim.params.target_rtf = view_rtf if gui else 0

    def before_step(**_: object) -> None:
        nonlocal turn, step_started_wall
        if callback_failure:
            return
        step_started_wall = time.perf_counter()
        try:
            step = sim.step_count
            ready = _ready(tasks, active, completed)
            for task in ready:
                task.setdefault("released_step", step)
            selected, turn = _select(policy, ready, active, turn)
            selected_ids = {task["task_id"] for task in selected}
            for task in ready:
                if task["task_id"] not in selected_ids and "first_hold_step" not in task:
                    task["first_hold_step"] = step
                    decisions.append(
                        {
                            "run_id": run_id,
                            "step": step,
                            "sim_time": step * config.timestep,
                            "task_id": task["task_id"],
                            "robot_id": task["robot_id"],
                            "decision": "hold_before_command",
                            "reason": "direction_gate",
                        }
                    )
            for task in selected:
                name = task["robot_id"]
                command_id = task["task_id"]
                ack = dispatcher.navigate(
                    [
                        RobotGoalCommand2D(
                            name=name,
                            position=tuple(task["destination"]),
                            z=0.1,
                            command_id=command_id,
                        )
                    ],
                    source="corridor-example",
                    command_id=command_id,
                )
                task["issued_step"] = step
                task["issued_time"] = step * config.timestep
                task["command_id"] = command_id
                task["ack"] = "accepted" if name in ack.accepted_names else ack.rejected.get(name, "rejected")
                decisions.append(
                    {
                        "run_id": run_id,
                        "step": step,
                        "sim_time": step * config.timestep,
                        "task_id": task["task_id"],
                        "robot_id": name,
                        "command_id": command_id,
                        "decision": "navigate",
                        "ack": task["ack"],
                    }
                )
                if name in ack.accepted_names:
                    active[name] = task
        except Exception as exc:
            callback_failure.append(exc)
            sim.params.target_rtf = 0

    def after_step(**_: object) -> None:
        nonlocal step_wall, corridor_peak_robots, corridor_over_capacity_steps, corridor_robot_steps, active_crowding
        if callback_failure:
            return
        step_wall += time.perf_counter() - step_started_wall
        try:
            # POST_STEP is emitted before the core increments step_count.
            observed_step = sim.step_count + 1
            if collect_collision_episodes:
                _observe_collisions(
                    sim, entities, active_episodes, episodes, config.margin, active, observation_step=observed_step
                )
            states = {state.name: state for state in provider.get_states_2d()}
            corridor_robots = sum(
                -3.0 <= state.position[0] <= 3.0 and -0.17 <= state.position[1] <= 0.17 for state in states.values()
            )
            corridor_peak_robots = max(corridor_peak_robots, corridor_robots)
            corridor_over_capacity_steps += int(corridor_robots > 3)
            corridor_robot_steps += corridor_robots
            if corridor_robots > 3:
                if active_crowding is None:
                    active_crowding = {
                        "start_step": observed_step,
                        "start_time": observed_step * config.timestep,
                        "peak_robots": corridor_robots,
                        "observed_steps": 0,
                    }
                active_crowding["peak_robots"] = max(active_crowding["peak_robots"], corridor_robots)
                active_crowding["observed_steps"] += 1
            elif active_crowding is not None:
                active_crowding["end_step"] = observed_step
                active_crowding["end_time"] = observed_step * config.timestep
                crowding_intervals.append(active_crowding)
                active_crowding = None
            for name, task in tuple(active.items()):
                state = states[name]
                if state.is_moving and "first_movement_step" not in task:
                    task["first_movement_step"] = observed_step
                    task["first_movement_time"] = observed_step * config.timestep
                goal = task["destination"]
                if math.dist(state.position, goal) <= config.position_tolerance and not state.is_moving:
                    task["completed_step"] = observed_step
                    task["completed_time"] = observed_step * config.timestep
                    completed.add(task["task_id"])
                    del active[name]
        except Exception as exc:
            callback_failure.append(exc)
            sim.params.target_rtf = 0

    sim.events.on(SimEvents.PRE_STEP, before_step)
    sim.events.on(SimEvents.POST_STEP, after_step)
    try:
        sim.run_simulation(duration=config.cutoff)
        if callback_failure:
            raise RuntimeError("corridor scenario callback failed") from callback_failure[0]
        if sim.step_count != config.steps:
            raise RuntimeError("simulation ended before the fixed simulated-time cutoff; no report was saved")
        for episode in active_episodes.values():
            episode["end_step"] = sim.step_count
            episode["end_time"] = sim.step_count * config.timestep
            episode["censored_at_end"] = True
            episodes.append(episode)
        if active_crowding is not None:
            active_crowding["end_step"] = sim.step_count
            active_crowding["end_time"] = sim.step_count * config.timestep
            active_crowding["censored_at_end"] = True
            crowding_intervals.append(active_crowding)
        elapsed_sim = sim.step_count * config.timestep
        elapsed_wall = time.perf_counter() - wall_start
        finished = [task for task in tasks if "completed_step" in task]
        issued = [task for task in tasks if "issued_step" in task]
        rejected = [task for task in tasks if "ack" in task and task["ack"] != "accepted"]
        for task in tasks:
            task["status"] = (
                "completed"
                if "completed_step" in task
                else "rejected" if "ack" in task and task["ack"] != "accepted" else "unfinished"
            )
        command_durations = [(task["completed_step"] - task["issued_step"]) * config.timestep for task in finished]
        admission_delays = [(task["issued_step"] - task["released_step"]) * config.timestep for task in issued]
        release_to_arrival = [(task["completed_step"] - task["released_step"]) * config.timestep for task in finished]
        report = {
            "schema_version": 1,
            "scenario_id": "pbf.synthetic_corridor.v1",
            "run_id": run_id,
            "policy": policy,
            "execution": {"gui": gui, "target_view_rtf": view_rtf if gui else None},
            "conditions": {
                **asdict(config),
                "robots": 20,
                "tasks": 40,
                "corridor_x": [-3.0, 3.0],
                "corridor_inner_y": [-0.17, 0.17],
                "robot_collision_mode": "normal_2d",
                "collision_detection_method": "closest_points",
                "collision_check_frequency": "every_step",
                "physics": False,
            },
            "measurement_contract": {
                "timebase": "simulated seconds; task and decision steps are simulation steps",
                "cutoff_rule": "fixed window; uncompleted tasks are censored, including unreleased second legs",
                "completion_rule": "within position_tolerance of destination and no longer moving",
                "collision_rule": (
                    "post-step signed closest-point distance: margin_only (0,d<=margin), geometric_overlap (d<=0)"
                ),
                "corridor_rule": "robot reference point in x=[-3,3], y=[-0.17,0.17]; over capacity means >3 robots",
                "admission_rule": "external task release to navigate issuance, never PBF internal waiting",
                "rate_rule": "completed tasks divided by fixed simulated-time cutoff",
                "step_wall_rule": "sum from PRE_STEP entry to POST_STEP entry; excludes collection and pacing",
                "units": {"distance": "m", "time": "s", "rate": "tasks/s"},
            },
            "tasks": tasks,
            "decisions": decisions,
            "collision_episodes": episodes,
            "crowding_intervals": crowding_intervals,
            "metrics": {
                "completed": len(completed),
                "rejected": len(rejected),
                "unfinished": len(tasks) - len(completed) - len(rejected),
                "completion_fraction": len(completed) / len(tasks),
                "completed_per_sim_second": len(completed) / elapsed_sim,
                "all_tasks_completed_at_seconds": (
                    max((task["completed_time"] for task in finished), default=None) if len(finished) == len(tasks) else None
                ),
                "completed_by_60_seconds": sum(task["completed_time"] <= 60 for task in finished),
                "command_to_arrival_seconds": _quantiles(command_durations),
                "external_admission_delay_seconds": _quantiles(admission_delays),
                "external_release_to_arrival_seconds": _quantiles(release_to_arrival),
                "released_but_unissued": sum("released_step" in task and "issued_step" not in task for task in tasks),
                "core_margin_entry_count": sim.collision_count,
                "corridor_peak_robots": corridor_peak_robots,
                "corridor_over_capacity_steps": corridor_over_capacity_steps,
                "corridor_robot_steps": corridor_robot_steps,
                "margin_only_episodes": sum(e["category"] == "margin_only" for e in episodes),
                "geometric_overlap_episodes": sum(e["category"] == "geometric_overlap" for e in episodes),
                "corridor_geometric_overlap_episodes": sum(
                    e["category"] == "geometric_overlap" and e["corridor_observed_steps"] > 0 for e in episodes
                ),
                "simulated_seconds": elapsed_sim,
                "wall_seconds": elapsed_wall,
                "step_wall_seconds": step_wall,
                "rtf": elapsed_sim / elapsed_wall if elapsed_wall else None,
                "collision_episode_collection_enabled": collect_collision_episodes,
            },
            "limits": [
                "Collision states are sampled after completed steps; shorter contacts can be missed.",
                "Core margin entries can omit positive-gap near misses because its AABB broadphase is not margin-expanded.",
                "Geometric overlap is not a physics impact.",
                "Admission delay is an external policy measurement, not PBF robot waiting time.",
                "Uncontrolled traffic passes through overlaps; crowding is not a physical queue or warehouse throughput.",
                "Manual GUI pause or single-step interaction can change the policy command schedule and reported outcome.",
            ],
        }
        if hold_gui:
            print("Simulation finished. Close the GUI window or press Ctrl+C to save the report and exit.")
            try:
                while p.isConnected(sim.client):
                    time.sleep(0.1)
            except KeyboardInterrupt:
                pass
        return report
    finally:
        sim.events.off(SimEvents.PRE_STEP, before_step)
        sim.events.off(SimEvents.POST_STEP, after_step)
        if sim.client is not None and p.isConnected(sim.client):
            p.disconnect(sim.client)


def compare_reports(left: dict, right: dict) -> dict:
    """Return parallel measurements without declaring a winning policy."""
    if left["scenario_id"] != right["scenario_id"] or left["conditions"] != right["conditions"]:
        raise ValueError("policy reports require equivalent scenario conditions")
    workload_fields = ("task_id", "robot_id", "leg", "direction", "start", "destination")
    left_workload = [
        tuple(task[field] if field not in ("start", "destination") else tuple(task[field]) for field in workload_fields)
        for task in left["tasks"]
    ]
    right_workload = [
        tuple(task[field] if field not in ("start", "destination") else tuple(task[field]) for field in workload_fields)
        for task in right["tasks"]
    ]
    if left_workload != right_workload:
        raise ValueError("policy reports require equivalent workload definitions")
    if left["policy"] == right["policy"]:
        raise ValueError("comparison requires two different policies")
    return {
        "schema_version": 1,
        "scenario_id": left["scenario_id"],
        "conditions": left["conditions"],
        "policies": {left["policy"]: left["metrics"], right["policy"]: right["metrics"]},
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path, help="Directory for independent policy reports")
    parser.add_argument("--dt", type=float, default=0.1, help="Simulation timestep in seconds")
    parser.add_argument("--cutoff", type=float, default=300.0, help="Fixed simulated-time cutoff")
    parser.add_argument("--policy", choices=("both", "uncontrolled", "direction_gate"), default="both")
    parser.add_argument("--gui", action="store_true", help="Observe one policy in the PyBullet GUI")
    parser.add_argument("--rtf", type=float, default=1.0, help="GUI target real-time factor (default: 1)")
    args = parser.parse_args()
    if args.gui and args.policy == "both":
        parser.error("--gui requires --policy uncontrolled or --policy direction_gate")
    if args.gui and (not math.isfinite(args.rtf) or args.rtf <= 0):
        parser.error("--rtf must be finite and positive with --gui")
    config = CorridorConfig(timestep=args.dt, cutoff=args.cutoff)
    args.output.mkdir(parents=True, exist_ok=False)
    policies = ("uncontrolled", "direction_gate") if args.policy == "both" else (args.policy,)
    reports = [run_policy(policy, config, gui=args.gui, view_rtf=args.rtf, hold_gui=args.gui) for policy in policies]
    for report in reports:
        path = args.output / f"{report['policy']}.json"
        path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
        print(report["policy"], report["metrics"])
    if len(reports) == 2:
        (args.output / "comparison.json").write_text(json.dumps(compare_reports(*reports), indent=2, sort_keys=True) + "\n")


if __name__ == "__main__":
    main()
