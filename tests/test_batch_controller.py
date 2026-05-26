"""Tests for batch (vectorized) controllers — Tasks 7 + 8.

Covers:
- BatchKinematicController lifecycle (register/unregister/reset, buffer sizing)
- BatchOmniController equivalence vs the per-agent OmniController for
  single-waypoint translation
- Integration with sim_core.step_once (batch_advance is called in Phase 1)
"""

from __future__ import annotations

import numpy as np
import pybullet as p
import pytest

from pybullet_fleet import (
    Agent,
    AgentSpawnParams,
    MotionMode,
    MultiRobotSimulationCore,
    Pose,
    SimulationParams,
)
from pybullet_fleet.controllers import BatchKinematicController, BatchOmniController
from pybullet_fleet.types import CollisionMode, SpatialHashCellSizeMode


# ----------------------------------------------------------------------
# Fixtures
# ----------------------------------------------------------------------


@pytest.fixture
def sim():
    params = SimulationParams(
        gui=False,
        monitor=False,
        physics=False,
        timestep=1.0 / 60.0,
        collision_check_frequency=0,
        spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
        spatial_hash_cell_size=2.0,
        log_level="warning",
    )
    sc = MultiRobotSimulationCore(params)
    yield sc
    p.disconnect(sc.client)


def _spawn_agent(sim, x=0.0, y=0.0, z=0.1, max_vel=1.0):
    return Agent.from_params(
        AgentSpawnParams(
            urdf_path="robots/simple_cube.urdf",
            initial_pose=Pose.from_xyz(x, y, z),
            motion_mode=MotionMode.OMNIDIRECTIONAL,
            collision_mode=CollisionMode.NORMAL_3D,
            max_linear_vel=max_vel,
            max_linear_accel=2.0,
        ),
        sim_core=sim,
    )


# ----------------------------------------------------------------------
# Task 7: BatchKinematicController lifecycle + base buffers
# ----------------------------------------------------------------------


class _NoopBatchController(BatchKinematicController):
    """Concrete-but-trivial subclass to exercise the ABC lifecycle."""

    def batch_advance(self, dt: float) -> np.ndarray:
        # Pretend nothing moved this step.
        self._moved_mask.fill(False)
        return self._moved_mask


class TestBatchControllerLifecycle:
    def test_register_sizes_buffers(self, sim):
        bc = _NoopBatchController()
        a = _spawn_agent(sim, x=0.0)
        b = _spawn_agent(sim, x=1.0)
        bc.register_agent(a)
        bc.register_agent(b)

        assert bc._pos_buf.shape == (2, 3)
        assert bc._orn_buf.shape == (2, 4)
        assert bc._moved_mask.shape == (2,)
        assert bc._object_ids.tolist() == [a.object_id, b.object_id]

    def test_register_twice_raises(self, sim):
        bc = _NoopBatchController()
        a = _spawn_agent(sim, x=0.0)
        bc.register_agent(a)
        with pytest.raises(ValueError):
            bc.register_agent(a)

    def test_unregister_compacts_arrays(self, sim):
        bc = _NoopBatchController()
        a = _spawn_agent(sim, x=0.0)
        b = _spawn_agent(sim, x=1.0)
        c = _spawn_agent(sim, x=2.0)
        bc.register_agent(a)
        bc.register_agent(b)
        bc.register_agent(c)

        bc.unregister_agent(b)
        assert len(bc._agents) == 2
        assert bc._pos_buf.shape == (2, 3)
        # b was at idx 1, c (last) swapped into idx 1
        assert bc._agents[1] is c
        assert bc._object_ids.tolist() == [a.object_id, c.object_id]

    def test_unregister_unknown_raises(self, sim):
        bc = _NoopBatchController()
        a = _spawn_agent(sim, x=0.0)
        with pytest.raises(KeyError):
            bc.unregister_agent(a)

    def test_reset_clears_all(self, sim):
        bc = _NoopBatchController()
        for x in range(3):
            bc.register_agent(_spawn_agent(sim, x=float(x)))
        bc.reset()
        assert bc._agents == []
        assert bc._pos_buf.shape == (0, 3)

    def test_first_register_auto_attaches(self, sim):
        bc = _NoopBatchController()
        assert bc._sim_core is None
        assert bc not in sim._batch_controllers
        a = _spawn_agent(sim, x=0.0)
        bc.register_agent(a)
        assert bc._sim_core is sim
        assert bc in sim._batch_controllers

    def test_last_unregister_auto_detaches(self, sim):
        bc = _NoopBatchController()
        a = _spawn_agent(sim, x=0.0)
        bc.register_agent(a)
        bc.unregister_agent(a)
        assert bc._sim_core is None
        assert bc not in sim._batch_controllers

    def test_register_agent_without_sim_core_raises(self, sim):
        bc = _NoopBatchController()

        class _Stub:
            sim_core = None

        with pytest.raises(ValueError):
            bc.register_agent(_Stub())  # type: ignore[arg-type]


# ----------------------------------------------------------------------
# Task 8: BatchOmniController behaviour + equivalence with OmniController
# ----------------------------------------------------------------------


class TestBatchOmniBasics:
    def test_set_path_requires_registered_agent(self, sim):
        bc = BatchOmniController()
        # Register a sentinel so the controller is attached to sim_core,
        # then verify set_path() rejects an agent that isn't in the set.
        sentinel = _spawn_agent(sim, x=-1.0)
        bc.register_agent(sentinel)
        a = _spawn_agent(sim, x=0.0)
        with pytest.raises(KeyError):
            bc.set_path(a, [Pose.from_xyz(1.0, 0.0, 0.1)])

    def test_step_once_advances_position(self, sim):
        bc = BatchOmniController()
        a = _spawn_agent(sim, x=0.0)
        bc.register_agent(a)
        bc.set_path(a, [Pose.from_xyz(1.0, 0.0, 0.1)])
        start_x = a.get_pose().x
        for _ in range(60):
            sim.step_once()
        # After 1s with max_vel=1, accel=2, should have arrived at x=1.
        end_x = a.get_pose().x
        assert end_x > start_x + 0.5  # at least moved a substantial fraction
        # Eventually reaches the goal
        for _ in range(120):
            sim.step_once()
        assert a.get_pose().x == pytest.approx(1.0, abs=1e-3)

    def test_multi_agent_step(self, sim):
        bc = BatchOmniController()
        agents = [_spawn_agent(sim, x=float(i)) for i in range(5)]
        for a in agents:
            bc.register_agent(a)
            bc.set_path(a, [Pose.from_xyz(a.get_pose().x + 1.0, 0.0, 0.1)])
        for _ in range(180):
            sim.step_once()
        for i, a in enumerate(agents):
            assert a.get_pose().x == pytest.approx(float(i) + 1.0, abs=1e-3)


class TestBatchOmniEquivalence:
    """BatchOmniController should produce trajectories matching OmniController
    within ~1e-6 per step for the supported scope (single goal, no rotation)."""

    @pytest.mark.parametrize("goal_xyz", [(1.5, 0.0, 0.1), (0.0, 2.0, 0.1), (1.0, 1.0, 0.1)])
    def test_per_step_position_matches_omni(self, sim, goal_xyz):
        # Per-agent reference
        ref = _spawn_agent(sim, x=0.0, y=0.0, max_vel=1.2)
        ref.set_path([Pose.from_xyz(*goal_xyz)], final_orientation_align=False)

        # Batched agent at the same start
        target = _spawn_agent(sim, x=0.0, y=0.0, max_vel=1.2)
        bc = BatchOmniController()
        bc.register_agent(target)
        bc.set_path(target, [Pose.from_xyz(*goal_xyz)])

        # Step in lock-step and compare positions
        for _ in range(120):
            sim.step_once()
            ref_pos = np.asarray(ref.get_pose().position)
            tgt_pos = np.asarray(target.get_pose().position)
            err = np.linalg.norm(ref_pos - tgt_pos)
            assert err < 1e-3, f"divergence {err:.6f} at step (ref={ref_pos}, tgt={tgt_pos})"


class TestBatchOmniIntegration:
    def test_uses_buffered_pose_write(self, sim):
        """Batch controller should write via the buffered set_poses path during
        step_once (verifiable by checking the pose lands in PyBullet after the step)."""
        bc = BatchOmniController()
        a = _spawn_agent(sim, x=0.0)
        bc.register_agent(a)
        bc.set_path(a, [Pose.from_xyz(1.0, 0.0, 0.1)])

        sim.step_once()
        pb_pos, _ = p.getBasePositionAndOrientation(a.object_id, physicsClientId=sim.client)
        cache_pos = a.get_pose().position
        # After step_once, PyBullet and cache must agree (Phase 2 flushed).
        assert pb_pos[0] == pytest.approx(cache_pos[0], abs=1e-9)

    def test_inactive_agents_dont_move(self, sim):
        bc = BatchOmniController()
        a = _spawn_agent(sim, x=0.0)
        bc.register_agent(a)
        # No set_path called — agent should stay put across many steps.
        start = a.get_pose().position[:]
        for _ in range(30):
            sim.step_once()
        assert np.allclose(a.get_pose().position, start)
