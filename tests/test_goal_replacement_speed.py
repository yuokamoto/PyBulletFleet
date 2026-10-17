"""A goal replaced mid-motion continues the motion instead of restarting from rest.

Every new goal or path used to build its trajectory from rest, and
``Agent.set_path`` put a zero-length approach waypoint (the current pose) in
front of it. A robot cruising at full speed that received a new goal straight
ahead dropped to 0 in one step and accelerated again. The omni controllers --
per-agent and batched -- now start from the agent's speed toward the new goal,
and brake as hard as needed rather than overshoot a goal closer than the
braking distance.
"""

from __future__ import annotations

import numpy as np
import pybullet as p
import pytest

from pybullet_fleet import Agent, AgentSpawnParams, MotionMode, MultiRobotSimulationCore, Pose, SimulationParams
from pybullet_fleet.types import CollisionMode

DT = 0.05
VMAX = 2.0
ACCEL = 4.0
DECEL = 4.0


@pytest.fixture
def sim():
    sc = MultiRobotSimulationCore(
        SimulationParams(
            gui=False,
            monitor=False,
            physics=False,
            timestep=DT,
            target_rtf=0,
            collision_check_frequency=0,
            monitor_log_interval=0,
            log_level="warning",
        )
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
            collision_mode=CollisionMode.NORMAL_3D,
            controller={"max_linear_vel": VMAX, "max_linear_accel": ACCEL, "max_linear_decel": DECEL},
        ),
        sim_core=sim,
    )
    if batch:
        from pybullet_fleet.agent_manager import AgentManager

        AgentManager(sim_core=sim, fleet_controller={"type": "batch_omni"}).add_object(agent)
    return agent


def _goal(x, y=0.0):
    return Pose.from_xyz(x, y, 0.1)


def _run(sim, agent, steps):
    """Per-step (x, y, speed) samples."""
    out = []
    for _ in range(steps):
        sim.step_once()
        x, y, _ = agent.get_pose().position
        out.append((x, y, float(np.linalg.norm(agent.velocity[:2]))))
    return out


def _cruise(sim, agent, far=100.0):
    agent.set_goal_pose(_goal(far))
    _run(sim, agent, int(2 * VMAX / ACCEL / DT) + 5)
    assert np.linalg.norm(agent.velocity[:2]) == pytest.approx(VMAX, rel=1e-3)


@pytest.fixture(params=[False, True], ids=["per_agent", "batch"])
def batch(request):
    return request.param


class TestGoalReplacedMidMotion:
    def test_keeps_its_speed_toward_a_goal_further_ahead(self, sim, batch):
        agent = _agent(sim, batch)
        _cruise(sim, agent)
        start_x = agent.get_pose().position[0]
        agent.set_goal_pose(_goal(start_x + 20.0))
        samples = _run(sim, agent, 5)
        assert all(speed == pytest.approx(VMAX, rel=1e-3) for _, _, speed in samples)
        # Distance covered at full speed, not a restart from rest.
        assert samples[-1][0] - start_x == pytest.approx(5 * DT * VMAX, rel=1e-3)

    def test_speed_never_jumps_more_than_the_ramps_allow(self, sim, batch):
        agent = _agent(sim, batch)
        _cruise(sim, agent)
        x = agent.get_pose().position[0]
        speeds = [VMAX]
        for offset in (6.0, 3.0, 9.0):  # replaced again and again, always ahead
            agent.set_goal_pose(_goal(x + offset))
            samples = _run(sim, agent, 8)
            speeds += [speed for _, _, speed in samples]
            x = samples[-1][0]
        steps = np.abs(np.diff(speeds))
        assert steps.max() <= max(ACCEL, DECEL) * DT + 1e-6

    def test_stops_at_a_goal_within_reach_braking_at_the_configured_rate(self, sim, batch):
        agent = _agent(sim, batch)
        _cruise(sim, agent)
        x0 = agent.get_pose().position[0]
        braking = VMAX * VMAX / (2 * DECEL)
        goal_x = x0 + braking + 0.5
        agent.set_goal_pose(_goal(goal_x))
        samples = _run(sim, agent, int(3 * VMAX / DECEL / DT) + 20)
        xs = [x for x, _, _ in samples]
        speeds = [VMAX] + [speed for _, _, speed in samples]
        assert max(xs) <= goal_x + 1e-6, "overshot the goal"
        assert all(b >= a - 1e-9 for a, b in zip(xs, xs[1:])), "reversed"
        assert np.max(np.diff(speeds)) <= ACCEL * DT + 1e-6
        assert -np.min(np.diff(speeds)) <= DECEL * DT + 1e-6
        assert xs[-1] == pytest.approx(goal_x, abs=1e-6)
        assert all(abs(y) < 1e-9 for _, y, _ in samples)

    def test_a_goal_inside_the_braking_distance_is_not_overshot(self, sim, batch):
        agent = _agent(sim, batch)
        _cruise(sim, agent)
        x0 = agent.get_pose().position[0]
        goal_x = x0 + 0.2  # braking distance is 0.5 m
        agent.set_goal_pose(_goal(goal_x))
        samples = _run(sim, agent, 20)
        xs = [x for x, _, _ in samples]
        assert max(xs) <= goal_x + 1e-6
        assert all(b >= a - 1e-9 for a, b in zip(xs, xs[1:]))
        assert xs[-1] == pytest.approx(goal_x, abs=1e-6)

    def test_a_goal_behind_starts_from_rest(self, sim, batch):
        agent = _agent(sim, batch)
        _cruise(sim, agent)
        x0 = agent.get_pose().position[0]
        agent.set_goal_pose(_goal(x0 - 5.0))
        speed = _run(sim, agent, 1)[0][2]
        assert speed <= ACCEL * DT + 1e-9  # restarted, not carried

    def test_a_goal_across_the_motion_starts_from_rest_along_it(self, sim, batch):
        agent = _agent(sim, batch)
        _cruise(sim, agent)
        x0, y0, _ = agent.get_pose().position
        agent.set_goal_pose(_goal(x0, y0 + 5.0))
        (x, y, speed) = _run(sim, agent, 1)[0]
        assert x == pytest.approx(x0)  # the motion across is dropped at once
        assert speed <= ACCEL * DT + 1e-9

    def test_a_goal_set_on_arrival_starts_from_rest(self, sim, batch):
        """Arrived is at rest, whatever the arrival step's average speed says.

        The next goal is set from the step the agent arrives in, before
        anything has zeroed the reported velocity.
        """
        agent = _agent(sim, batch)
        agent.set_goal_pose(_goal(1.0))
        handed = []

        def next_goal(sim_core, dt):
            if not agent.is_moving and not handed:
                handed.append(np.linalg.norm(agent.velocity[:2]))
                agent.set_goal_pose(_goal(5.0))

        sim.register_callback(next_goal, phase="post_step")
        for _ in range(200):
            sim.step_once()
            if handed:
                break
        assert handed, "never arrived"
        speed = _run(sim, agent, 1)[0][2]
        # From rest the first step is below one step of acceleration; carrying
        # the arrival step's average speed would put it at or above.
        assert speed < ACCEL * DT - 1e-6

    def test_a_path_set_on_arrival_keeps_its_approach_waypoint(self, sim, batch):
        """Arrived is not moving, whatever the arrival step's speed says."""
        agent = _agent(sim, batch)
        agent.set_goal_pose(_goal(1.0))
        paths = []

        def next_path(sim_core, dt):
            if not agent.is_moving and not paths:
                agent.set_goal_pose(_goal(5.0))
                if batch:
                    bc = agent._batch_controller
                    paths.append(bc._paths[bc._agent_index[id(agent)]])
                else:
                    paths.append(agent._controllers[0]._path)

        sim.register_callback(next_path, phase="post_step")
        for _ in range(200):
            sim.step_once()
            if paths:
                break
        assert paths and len(paths[0]) == 2, "approach waypoint + goal"


class TestFromRestUnchanged:
    def test_a_goal_from_rest_ramps_up_from_zero(self, sim, batch):
        agent = _agent(sim, batch)
        agent.set_goal_pose(_goal(10.0))
        speeds = [speed for _, _, speed in _run(sim, agent, 3)]
        assert speeds[0] <= ACCEL * DT + 1e-9
        assert np.max(np.diff([0.0] + speeds)) <= ACCEL * DT + 1e-6

    def test_per_agent_and_batch_follow_the_same_trajectory(self):
        tracks = []
        for batch in (False, True):
            sc = MultiRobotSimulationCore(
                SimulationParams(gui=False, monitor=False, physics=False, timestep=DT, target_rtf=0, log_level="warning")
            )
            sc.initialize_simulation()
            try:
                agent = _agent(sc, batch)
                _cruise(sc, agent)
                x0 = agent.get_pose().position[0]
                agent.set_goal_pose(_goal(x0 + 3.0))
                tracks.append([x - x0 for x, _, _ in _run(sc, agent, 40)])
            finally:
                p.disconnect(sc.client)
        assert tracks[0] == pytest.approx(tracks[1], abs=1e-6)


def test_a_checkpoint_keeps_the_initial_speed(sim):
    """A trajectory continued from speed restores with the same speed."""
    agent = _agent(sim)
    _cruise(sim, agent)
    x0 = agent.get_pose().position[0]
    agent.set_goal_pose(_goal(x0 + 6.0))
    sim.step_once()
    controller = agent._controllers[0]
    state = controller.capture_navigation_state()
    assert state["forward"]["v0"] == pytest.approx(VMAX, rel=1e-3)

    from pybullet_fleet.controller import OmniController

    restored = OmniController(controller.params)
    restored.restore_navigation_state(state, list(agent.get_pose().orientation))
    for t in np.linspace(sim.sim_time, sim.sim_time + 3.0, 7):
        assert restored._tpi_forward.get_point(t)[0] == pytest.approx(controller._tpi_forward.get_point(t)[0])
