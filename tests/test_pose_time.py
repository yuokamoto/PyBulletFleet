"""A pose and its time agree: ``step_end_time`` is when the poses are.

Position controllers evaluate their trajectories at ``step_end_time`` and start
a trajectory there, so the pose a step ends with is the trajectory at the end of
the step, the time ``step_end_time`` reports for it. A goal replaced mid-motion
continues from the speed of the pose in effect when it is set, in every phase,
and the per-agent and batched controllers agree on it.

Uses the motion limits of the goal-replacement tests: 0.05 s steps,
4 m/s^2 acceleration and deceleration.
"""

from __future__ import annotations

import pybullet as p
import pytest

from pybullet_fleet import Agent, AgentSpawnParams, MotionMode, MultiRobotSimulationCore, Pose, SimulationParams
from pybullet_fleet.events import SimEvents
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


def _goal(x):
    return Pose.from_xyz(x, 0.0, 0.1)


FAR = 10.0


def _start(sim, agent):
    assert agent.set_path([_goal(FAR)], final_orientation_align=False, auto_approach=False)


@pytest.mark.parametrize("batch", [False, True], ids=["per-agent", "batch"])
def test_a_pose_is_stamped_with_its_trajectory_time(sim, batch):
    """From rest at 4 m/s^2 the trajectory is x = 2 t^2: every pose a step ends
    with must be that at the time it is stamped with -- e.g. x = 0.08 at
    0.20 s, not at 0.25 s."""
    agent = _agent(sim, batch=batch)
    _start(sim, agent)
    seen = []
    sim.register_callback(
        lambda core, dt: seen.append((core.step_end_time, agent.get_pose().position[0])),
        frequency=None,
        phase="post_step",
    )
    for _ in range(8):
        sim.step_once()
        # Outside the step too: the completed time is the pose's time.
        seen.append((sim.step_end_time, agent.get_pose().position[0]))
    assert len(seen) == 16
    for stamp, x in seen:
        assert x == pytest.approx(0.5 * ACCEL * stamp**2, abs=1e-9)
    assert (pytest.approx(0.2), pytest.approx(0.08)) in [(s, x) for s, x in seen]


def _phase_hooks(sim, phase, action):
    """Run ``action()`` once, from ``phase``, during the next step (or right now)."""
    if phase == "outside":
        action()
        return
    fired = []

    def once(*_args, **_kwargs):
        if not fired:
            fired.append(True)
            action()

    if phase == "pre_step":
        sim.events.on(SimEvents.PRE_STEP, once)
    elif phase == "post_step_event":
        sim.events.on(SimEvents.POST_STEP, once)
    elif phase == "callback":
        sim.register_callback(once, frequency=None)
    elif phase == "post_step_callback":
        sim.register_callback(once, frequency=None, phase="post_step")
    sim.step_once()


PHASES = ["outside", "pre_step", "callback", "post_step_event", "post_step_callback"]


@pytest.mark.parametrize("phase", PHASES)
@pytest.mark.parametrize("margin, accepted", [(0.9, False), (1.1, True)], ids=["inside-braking", "beyond-braking"])
def test_a_replaced_goal_is_judged_on_the_pose_in_effect(sim, phase, margin, accepted):
    """The braking distance is that of the speed at step_end_time when the goal
    is set, per-agent and batched alike. In PRE_STEP that is still the previous
    step's end: a batch row must not use the speed of the step about to run."""
    agents = {name: _agent(sim, batch=(name == "batch")) for name in ("per-agent", "batch")}
    for agent in agents.values():
        _start(sim, agent)
    for _ in range(5):
        sim.step_once()

    results = {}

    def replace():
        speed = ACCEL * sim.step_end_time  # from rest at t0 = 0
        braking = speed**2 / (2 * DECEL)
        for name, agent in agents.items():
            ahead = agent.get_pose().position[0] + margin * braking
            results[name] = agent.set_path([_goal(ahead)], final_orientation_align=False, auto_approach=False)

    _phase_hooks(sim, phase, replace)
    assert results == {"per-agent": accepted, "batch": accepted}


@pytest.mark.parametrize("ahead, expected", [(0.1, False), (0.13, True)])
def test_the_reported_pre_step_case(sim, ahead, expected):
    """After five steps a PRE_STEP goal 0.1 m ahead: the pose in effect is the
    one at 0.25 s, at 1.0 m/s, which needs 0.125 m to stop -- both refuse it,
    and both take one 0.13 m ahead."""
    agents = [_agent(sim), _agent(sim, batch=True)]
    for agent in agents:
        _start(sim, agent)
    for _ in range(5):
        sim.step_once()
    assert sim.step_end_time == pytest.approx(5 * DT)
    decided = []

    def replace():
        decided.extend(
            a.set_path([_goal(a.get_pose().position[0] + ahead)], final_orientation_align=False, auto_approach=False)
            for a in agents
        )

    _phase_hooks(sim, "pre_step", replace)
    assert decided == [expected, expected]
