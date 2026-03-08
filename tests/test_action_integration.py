"""
Integration tests for Action execution with real Agent + PyBullet.

Unlike test_action.py (unit tests with MockAgent), these tests run actions
through a real Agent.update() loop in a headless PyBullet environment to
verify end-to-end behaviour:

- MoveAction: agent follows path and reaches destination
- PickAction: agent approaches target, attaches object
- DropAction: agent approaches drop pose, detaches object at correct position
- Sequence: Move → Pick → Move → Drop full workflow
"""

import os

import numpy as np
import pybullet as p
import pybullet_data
import pytest

from pybullet_fleet.action import (
    JointAction,
    MoveAction,
    PickAction,
    DropAction,
    WaitAction,
)
from pybullet_fleet.agent import Agent
from pybullet_fleet.geometry import Pose, Path
from pybullet_fleet.sim_object import SimObject, ShapeParams
from pybullet_fleet.types import ActionStatus, MotionMode, MovementDirection


# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

MESH_PATH = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
ARM_URDF = "robots/arm_robot.urdf"


# ---------------------------------------------------------------------------
# Assertion tolerances
# ---------------------------------------------------------------------------

TIGHT_TOL = 0.01  # Final position accuracy (±5 cm)
PROXIMITY_TOL = 0.01  # "Near target" checks
COARSE_TOL = 0.01  # Box placement / waypoint proximity


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


class MockSimCore:
    """Minimal sim_core with advancing time, object registry, and no-op helpers."""

    def __init__(self, dt: float = 1.0 / 240.0):
        self.sim_time = 0.0
        self._dt = dt
        self.sim_objects: list = []
        self._kinematic_objects: set = set()
        self._client = 0  # default PyBullet physics client ID

    @property
    def client(self):
        return self._client

    def add_object(self, obj):
        self.sim_objects.append(obj)

    def remove_object(self, obj):
        if obj in self.sim_objects:
            self.sim_objects.remove(obj)

    def _mark_object_moved(self, object_id):
        pass

    def tick(self, n: int = 1):
        self.sim_time += self._dt * n


def run_until_idle(agent, sim_core, *, max_steps: int = 20_000) -> int:
    """Run agent.update() until the action queue is empty.

    Returns the number of steps executed.
    Raises AssertionError if *max_steps* is exceeded.
    """
    dt = sim_core._dt
    for step in range(max_steps):
        sim_core.tick()
        agent.update(dt)
        if agent.is_action_queue_empty() and not agent.is_moving:
            return step + 1
    raise AssertionError(f"Agent did not finish actions within {max_steps} steps")


def create_agent(sim_core, *, pose=None, motion_mode=MotionMode.OMNIDIRECTIONAL):
    """Create an omnidirectional mesh agent wired to *sim_core*."""
    if pose is None:
        pose = Pose.from_xyz(0, 0, 0)
    return Agent.from_mesh(
        visual_shape=ShapeParams(shape_type="mesh", mesh_path=MESH_PATH, mesh_scale=[0.2, 0.2, 0.2]),
        collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
        pose=pose,
        mass=0.0,
        max_linear_vel=5.0,
        max_linear_accel=20.0,
        max_angular_vel=6.0,
        max_angular_accel=20.0,
        motion_mode=motion_mode,
        sim_core=sim_core,
    )


def create_pickable_box(sim_core, *, pos=(2, 0, 0)):
    """Create a small pickable SimObject registered with *sim_core*."""
    return SimObject.from_mesh(
        visual_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05]),
        collision_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05]),
        pose=Pose.from_xyz(*pos),
        mass=0.0,
        pickable=True,
        sim_core=sim_core,
    )


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def pybullet_env():
    """Headless PyBullet session with ground plane."""
    client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    p.loadURDF("plane.urdf")
    yield client
    p.disconnect()


@pytest.fixture
def sim(pybullet_env):
    """MockSimCore wired to the PyBullet session."""
    return MockSimCore()


# ---------------------------------------------------------------------------
# MoveAction
# ---------------------------------------------------------------------------


class TestMoveActionIntegration:
    """MoveAction executed through Agent.update() loop."""

    def test_single_waypoint(self, sim):
        """Agent reaches a single waypoint and action completes."""
        agent = create_agent(sim)
        goal = [2, 0, 0]
        action = MoveAction(path=Path.from_positions([goal]))
        agent.add_action(action)

        run_until_idle(agent, sim)

        assert action.status is ActionStatus.COMPLETED
        pos = agent.get_pose().position
        assert np.allclose(pos[:2], goal[:2], atol=PROXIMITY_TOL)

    def test_multi_waypoint_path(self, sim):
        """Agent follows an L-shaped path through all intermediate waypoints."""
        agent = create_agent(sim)
        waypoints = [[1, 0, 0], [1, 1, 0], [0, 1, 0]]
        action = MoveAction(path=Path.from_positions(waypoints))
        agent.add_action(action)

        # Record positions during update loop to verify intermediate waypoints
        trajectory: list = []
        dt = sim._dt
        for _ in range(20_000):
            sim.tick()
            agent.update(dt)
            trajectory.append(list(agent.get_pose().position[:2]))
            if agent.is_action_queue_empty() and not agent.is_moving:
                break

        assert action.status is ActionStatus.COMPLETED

        # Final position
        pos = agent.get_pose().position
        assert np.allclose(pos[:2], [0, 1], atol=PROXIMITY_TOL)

        # Verify agent passed near each intermediate waypoint
        for wp in waypoints[:-1]:
            dists = [np.linalg.norm(np.array(p) - np.array(wp[:2])) for p in trajectory]
            assert min(dists) < COARSE_TOL, f"Agent never passed near intermediate waypoint {wp}"

    def test_backward_direction(self, sim):
        """Agent moves backward: reaches goal while facing away from travel direction."""
        agent = create_agent(sim, motion_mode=MotionMode.DIFFERENTIAL)
        goal = [2, 0, 0]
        action = MoveAction(
            path=Path.from_positions([goal]),
            direction=MovementDirection.BACKWARD,
            final_orientation_align=False,
        )
        agent.add_action(action)

        run_until_idle(agent, sim)

        assert action.status is ActionStatus.COMPLETED
        pos = agent.get_pose().position
        assert np.allclose(pos[:2], goal[:2], atol=PROXIMITY_TOL)

        # Backward movement: agent's X-axis should point OPPOSITE to travel direction.
        # Travel was +X, so agent should face -X (yaw ≈ π or -π).
        orn = agent.get_pose().orientation
        _, _, yaw = p.getEulerFromQuaternion(orn)
        assert (
            abs(abs(yaw) - np.pi) < 0.3
        ), f"Backward agent should face away from travel direction (yaw ≈ ±π), got {np.degrees(yaw):.1f}°"

    def test_duration_is_recorded(self, sim):
        """Start and end time are recorded after completion."""
        agent = create_agent(sim)
        action = MoveAction(path=Path.from_positions([[1, 0, 0]]))
        agent.add_action(action)

        run_until_idle(agent, sim)

        assert action.start_time is not None
        assert action.end_time is not None
        assert action.get_duration() >= 0  # type: ignore[operator]

    def test_sequential_moves(self, sim):
        """Two MoveActions execute in sequence; both complete."""
        agent = create_agent(sim)
        a1 = MoveAction(path=Path.from_positions([[1, 0, 0]]))
        a2 = MoveAction(path=Path.from_positions([[1, 1, 0]]))
        agent.add_action_sequence([a1, a2])

        run_until_idle(agent, sim)

        assert a1.status is ActionStatus.COMPLETED
        assert a2.status is ActionStatus.COMPLETED
        pos = agent.get_pose().position
        assert np.allclose(pos[:2], [1, 1], atol=PROXIMITY_TOL)


# ---------------------------------------------------------------------------
# PickAction
# ---------------------------------------------------------------------------


class TestPickActionIntegration:
    """PickAction executed through Agent.update() loop."""

    def test_pick_with_approach(self, sim):
        """Full 5-phase pick: approach → move_to_pick → pick → retreat.

        Verify via trajectory that agent approached the target, then retreated.
        """
        approach_offset = 0.5
        target_pos = (2, 0, 0)

        agent = create_agent(sim)
        box = create_pickable_box(sim, pos=target_pos)
        target_xy = np.array(target_pos[:2])

        action = PickAction(
            target_object_id=box.body_id,
            use_approach=True,
            approach_offset=approach_offset,
            pick_offset=0.0,
        )
        agent.add_action(action)

        # Record distance-to-target over time
        dists: list = []
        dt = sim._dt
        for _ in range(20_000):
            sim.tick()
            agent.update(dt)
            pos = agent.get_pose().position
            dists.append(float(np.linalg.norm(np.array(pos[:2]) - target_xy)))
            if agent.is_action_queue_empty() and not agent.is_moving:
                break

        assert action.status is ActionStatus.COMPLETED
        assert box.is_attached()

        # Agent should have reached near the target (pick_offset=0 → at target)
        min_dist = min(dists)
        assert min_dist < PROXIMITY_TOL, f"Agent should have reached near target (pick_offset=0), closest was {min_dist:.2f}m"

        # After reaching target, distance should have increased (retreat).
        # Find the index of closest approach, then verify distance grew afterward.
        min_idx = dists.index(min_dist)
        final_dist = dists[-1]
        assert final_dist > min_dist + COARSE_TOL, (
            f"Agent should have retreated after pick " f"(closest={min_dist:.2f}m at step {min_idx}, final={final_dist:.2f}m)"
        )

        # Final position should be at approach_offset from target
        assert (
            abs(final_dist - approach_offset) < TIGHT_TOL
        ), f"Agent should end ~{approach_offset}m (approach_offset) from target, but is {final_dist:.2f}m"

    def test_pick_with_approach_and_pick_offset(self, sim):
        """Pick with approach and pick_offset: agent stops at pick_offset, not at target.

        Verify agent approached to pick_offset distance (not to target itself),
        then retreated to approach_offset distance.
        """
        approach_offset = 0.5
        pick_offset = 0.3
        target_pos = (2, 0, 0)

        agent = create_agent(sim)
        box = create_pickable_box(sim, pos=target_pos)
        target_xy = np.array(target_pos[:2])

        action = PickAction(
            target_object_id=box.body_id,
            use_approach=True,
            approach_offset=approach_offset,
            pick_offset=pick_offset,
        )
        agent.add_action(action)

        dists: list = []
        dt = sim._dt
        for _ in range(20_000):
            sim.tick()
            agent.update(dt)
            pos = agent.get_pose().position
            dists.append(float(np.linalg.norm(np.array(pos[:2]) - target_xy)))
            if agent.is_action_queue_empty() and not agent.is_moving:
                break

        assert action.status is ActionStatus.COMPLETED
        assert box.is_attached()

        # Closest distance should be ~pick_offset, NOT at target (0.0m)
        min_dist = min(dists)
        assert (
            abs(min_dist - pick_offset) < PROXIMITY_TOL
        ), f"Agent should approach to ~{pick_offset}m (pick_offset) from target, closest was {min_dist:.2f}m"

        # Final position should be at approach_offset
        final_dist = dists[-1]
        assert (
            abs(final_dist - approach_offset) < TIGHT_TOL
        ), f"Agent should end ~{approach_offset}m (approach_offset) from target, but is {final_dist:.2f}m"

    def test_pick_without_approach(self, sim):
        """No approach/retreat: agent moves to pick_pose and stays there.

        With pick_offset=0.0, agent ends at the target position.
        """
        target_pos = (2, 0, 0)

        agent = create_agent(sim, pose=Pose.from_xyz(1.9, 0, 0))
        box = create_pickable_box(sim, pos=target_pos)

        action = PickAction(
            target_object_id=box.body_id,
            use_approach=False,
            pick_offset=0.0,
        )
        agent.add_action(action)

        run_until_idle(agent, sim)

        assert action.status is ActionStatus.COMPLETED
        assert box in agent.attached_objects

        # No retreat → agent stays at pick_pose = target position (pick_offset=0).
        agent_pos = agent.get_pose().position
        target_xy = np.array(target_pos[:2])
        dist_to_target = np.linalg.norm(np.array(agent_pos[:2]) - target_xy)

        assert dist_to_target < PROXIMITY_TOL, (
            f"Agent should be at target (pick_offset=0, no retreat), "
            f"but is {dist_to_target:.2f}m away (pos={agent_pos[:2]})"
        )

    def test_pick_without_approach_with_pick_offset(self, sim):
        """No approach/retreat with pick_offset: agent stops at pick_offset distance.

        With pick_offset=0.3, agent ends 0.3m from the target.
        """
        pick_offset = 0.3
        target_pos = (2, 0, 0)

        agent = create_agent(sim)
        box = create_pickable_box(sim, pos=target_pos)

        action = PickAction(
            target_object_id=box.body_id,
            use_approach=False,
            pick_offset=pick_offset,
        )
        agent.add_action(action)

        run_until_idle(agent, sim)

        assert action.status is ActionStatus.COMPLETED
        assert box.is_attached()

        # No retreat → agent stays at pick_offset from target.
        agent_pos = agent.get_pose().position
        target_xy = np.array(target_pos[:2])
        dist_to_target = np.linalg.norm(np.array(agent_pos[:2]) - target_xy)
        assert abs(dist_to_target - pick_offset) < TIGHT_TOL, (
            f"Agent should be ~{pick_offset}m (pick_offset) from target, "
            f"but is {dist_to_target:.2f}m away (pos={agent_pos[:2]})"
        )

    def test_pick_with_relative_pose(self, sim):
        """Picked object is attached at the specified relative pose offset."""
        rel_offset = 0.3
        approach_offset = 0.5

        agent = create_agent(sim)
        box = create_pickable_box(sim)

        rel_pose = Pose.from_euler(rel_offset, 0, 0)
        action = PickAction(
            target_object_id=box.body_id,
            use_approach=True,
            approach_offset=approach_offset,
            attach_relative_pose=rel_pose,
        )
        agent.add_action(action)

        run_until_idle(agent, sim)

        assert action.status is ActionStatus.COMPLETED
        assert box.is_attached()

        # Verify relative pose: box should be ~0.3m from agent in the agent's local X direction.
        agent_pos = np.array(agent.get_pose().position)
        box_pos = np.array(box.get_pose().position)
        offset_world = box_pos - agent_pos
        offset_dist = np.linalg.norm(offset_world)
        assert abs(offset_dist - rel_offset) < TIGHT_TOL, (
            f"Box should be ~{rel_offset}m from agent (relative_pose offset), "
            f"but is {offset_dist:.3f}m (offset={offset_world})"
        )

    def test_pick_nonexistent_object_fails(self, sim):
        """PickAction fails if the target body_id is not found."""
        agent = create_agent(sim)

        action = PickAction(target_object_id=9999)
        agent.add_action(action)

        run_until_idle(agent, sim)

        assert action.status is ActionStatus.FAILED
        assert action.error_message is not None

    def test_pick_non_pickable_object_fails(self, sim):
        """PickAction fails if the target object is not pickable."""
        agent = create_agent(sim)
        obj = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05]),
            pose=Pose.from_xyz(2, 0, 0),
            mass=0.0,
            pickable=False,
            sim_core=sim,
        )

        action = PickAction(target_object_id=obj.body_id)
        agent.add_action(action)

        run_until_idle(agent, sim)

        assert action.status is ActionStatus.FAILED

    def test_pick_by_position(self, sim):
        """PickAction can find nearest pickable object by position."""
        target_pos = (2, 0, 0)

        agent = create_agent(sim)
        box = create_pickable_box(sim, pos=target_pos)

        action = PickAction(
            target_position=list(target_pos),
            search_radius=1.0,
            use_approach=True,
            approach_offset=0.5,
        )
        agent.add_action(action)

        run_until_idle(agent, sim)

        assert action.status is ActionStatus.COMPLETED
        assert box.is_attached()

    def test_pick_by_position_offset_within_radius(self, sim):
        """PickAction finds nearest pickable object when position is not exact.

        Box is at (2, 0.4, 0), search from (2, 0, 0) with radius=0.5.
        The box is 0.4m away — within search_radius.
        """
        search_pos = (2, 0, 0)
        search_radius = 0.5

        agent = create_agent(sim)
        box = create_pickable_box(sim, pos=(2, 0.4, 0))

        action = PickAction(
            target_position=list(search_pos),
            search_radius=search_radius,
            use_approach=True,
            approach_offset=0.5,
        )
        agent.add_action(action)

        run_until_idle(agent, sim)

        assert action.status is ActionStatus.COMPLETED
        assert box.is_attached()

    def test_pick_by_position_outside_radius_fails(self, sim):
        """PickAction fails when no pickable object is within search_radius."""
        search_pos = (2, 0, 0)
        search_radius = 0.5

        agent = create_agent(sim)
        # Box at (2, 1, 0) is 1.0m from search position — outside radius
        create_pickable_box(sim, pos=(2, 1, 0))

        action = PickAction(
            target_position=list(search_pos),
            search_radius=search_radius,
        )
        agent.add_action(action)

        run_until_idle(agent, sim)

        assert action.status is ActionStatus.FAILED

    def test_pick_with_explicit_approach_pose(self, sim):
        """PickAction with explicit approach_pose not on the agent-target line.

        Agent at (0,0), target at (2,0), approach_pose at (2,1) — off to the side.
        Agent should go to (2,1) first, then forward to target, then retreat to (2,1).
        """
        agent = create_agent(sim)
        box = create_pickable_box(sim)
        approach = Pose.from_xyz(2, 1, 0)

        action = PickAction(
            target_object_id=box.body_id,
            use_approach=True,
            approach_pose=approach,
            pick_offset=0.0,
        )
        agent.add_action(action)

        # Record trajectory to verify lateral approach
        trajectory: list = []
        dt = sim._dt
        for _ in range(20_000):
            sim.tick()
            agent.update(dt)
            trajectory.append(list(agent.get_pose().position[:2]))
            if agent.is_action_queue_empty() and not agent.is_moving:
                break

        assert action.status is ActionStatus.COMPLETED
        assert box.is_attached()

        # Agent should have passed near approach_pose (2,1), not just gone straight
        dists_to_approach = [np.linalg.norm(np.array(t) - np.array([2, 1])) for t in trajectory]
        assert min(dists_to_approach) < COARSE_TOL, (
            f"Agent should have passed near explicit approach_pose (2,1), " f"closest was {min(dists_to_approach):.2f}m"
        )

        # Final position should be back near approach_pose after retreat
        final_pos = np.array(trajectory[-1])
        dist_to_approach = np.linalg.norm(final_pos - np.array([2, 1]))
        assert dist_to_approach < 0.3, f"Agent should retreat to approach_pose (2,1), but ended at {final_pos}"


# ---------------------------------------------------------------------------
# DropAction
# ---------------------------------------------------------------------------


class TestDropActionIntegration:
    """DropAction executed through Agent.update() loop."""

    def _pick_box(self, agent, box, sim):
        """Helper: pick a box and wait for completion."""
        action = PickAction(
            target_object_id=box.body_id,
            use_approach=False,
        )
        agent.add_action(action)
        run_until_idle(agent, sim)
        assert box.is_attached()

    def test_drop_with_approach(self, sim):
        """Full 5-phase drop: approach → move_to_drop → drop → retreat.

        Verify via trajectory that agent approached the drop pose, then retreated.
        """
        approach_offset = 0.5
        drop_target = [4, 0, 0]

        agent = create_agent(sim, pose=Pose.from_xyz(1.9, 0, 0))
        box = create_pickable_box(sim)
        self._pick_box(agent, box, sim)

        target_xy = np.array(drop_target[:2])
        action = DropAction(
            drop_pose=Pose(position=drop_target),
            use_approach=True,
            approach_offset=approach_offset,
            drop_offset=0.0,
            place_gently=True,
        )
        agent.add_action(action)

        # Record distance-to-drop-pose over time
        dists: list = []
        dt = sim._dt
        for _ in range(20_000):
            sim.tick()
            agent.update(dt)
            pos = agent.get_pose().position
            dists.append(float(np.linalg.norm(np.array(pos[:2]) - target_xy)))
            if agent.is_action_queue_empty() and not agent.is_moving:
                break

        assert action.status is ActionStatus.COMPLETED
        assert not box.is_attached()

        # Box should be at drop position
        box_pos = box.get_pose().position
        assert np.allclose(box_pos[:2], drop_target[:2], atol=COARSE_TOL)

        # Agent should have reached near the drop pose (drop_offset=0 → at target)
        min_dist = min(dists)
        assert (
            min_dist < PROXIMITY_TOL
        ), f"Agent should have reached near drop pose (drop_offset=0), closest was {min_dist:.2f}m"

        # Final position should be at approach_offset from drop pose
        final_dist = dists[-1]
        assert (
            abs(final_dist - approach_offset) < TIGHT_TOL
        ), f"Agent should end ~{approach_offset}m (approach_offset) from drop pose, but is {final_dist:.2f}m"

    def test_drop_with_approach_and_drop_offset(self, sim):
        """Drop with approach and drop_offset: agent stops at drop_offset, not at target.

        Verify agent approached to drop_offset distance (not to target itself),
        then retreated to approach_offset distance.
        """
        approach_offset = 0.5
        drop_offset = 0.3
        drop_target = [4, 0, 0]

        agent = create_agent(sim, pose=Pose.from_xyz(1.9, 0, 0))
        box = create_pickable_box(sim)
        self._pick_box(agent, box, sim)

        target_xy = np.array(drop_target[:2])
        action = DropAction(
            drop_pose=Pose(position=drop_target),
            use_approach=True,
            approach_offset=approach_offset,
            drop_offset=drop_offset,
            place_gently=True,
        )
        agent.add_action(action)

        dists: list = []
        dt = sim._dt
        for _ in range(20_000):
            sim.tick()
            agent.update(dt)
            pos = agent.get_pose().position
            dists.append(float(np.linalg.norm(np.array(pos[:2]) - target_xy)))
            if agent.is_action_queue_empty() and not agent.is_moving:
                break

        assert action.status is ActionStatus.COMPLETED
        assert not box.is_attached()

        # Closest distance should be ~drop_offset, NOT at target (0.0m)
        min_dist = min(dists)
        assert (
            abs(min_dist - drop_offset) < PROXIMITY_TOL
        ), f"Agent should approach to ~{drop_offset}m (drop_offset) from drop pose, closest was {min_dist:.2f}m"

        # Final position should be at approach_offset
        final_dist = dists[-1]
        assert (
            abs(final_dist - approach_offset) < TIGHT_TOL
        ), f"Agent should end ~{approach_offset}m (approach_offset) from drop pose, but is {final_dist:.2f}m"

    def test_drop_without_approach(self, sim):
        """No approach/retreat: agent moves to drop_pose and stays there.

        With drop_offset=0.0, agent ends at the drop position.
        """
        agent = create_agent(sim, pose=Pose.from_xyz(1.9, 0, 0))
        box = create_pickable_box(sim)
        self._pick_box(agent, box, sim)

        drop_target = [3, 0, 0]
        action = DropAction(
            drop_pose=Pose(position=drop_target),
            use_approach=False,
            drop_offset=0.0,
            place_gently=True,
        )
        agent.add_action(action)

        run_until_idle(agent, sim)

        assert action.status is ActionStatus.COMPLETED
        assert not box.is_attached()

        # No retreat → agent stays at drop_pose (drop_offset=0).
        agent_pos = agent.get_pose().position
        target_pos = np.array(drop_target[:2])
        dist_to_target = np.linalg.norm(np.array(agent_pos[:2]) - target_pos)
        assert dist_to_target < PROXIMITY_TOL, (
            f"Agent should be at drop pose (drop_offset=0, no retreat), "
            f"but is {dist_to_target:.2f}m away (pos={agent_pos[:2]})"
        )

        # Box should be at drop position
        box_pos = box.get_pose().position
        assert np.allclose(box_pos[:2], drop_target[:2], atol=COARSE_TOL)

    def test_drop_without_approach_with_drop_offset(self, sim):
        """No approach/retreat with drop_offset: agent stops at drop_offset distance.

        With drop_offset=0.3, agent ends 0.3m from the drop pose.
        """
        drop_offset = 0.3
        drop_target = [4, 0, 0]

        agent = create_agent(sim, pose=Pose.from_xyz(1.9, 0, 0))
        box = create_pickable_box(sim)
        self._pick_box(agent, box, sim)

        action = DropAction(
            drop_pose=Pose(position=drop_target),
            use_approach=False,
            drop_offset=drop_offset,
            place_gently=True,
        )
        agent.add_action(action)

        run_until_idle(agent, sim)

        assert action.status is ActionStatus.COMPLETED
        assert not box.is_attached()

        # No retreat → agent stays at drop_pose offset by drop_offset.
        agent_pos = agent.get_pose().position
        target_xy = np.array(drop_target[:2])
        dist_to_target = np.linalg.norm(np.array(agent_pos[:2]) - target_xy)
        assert abs(dist_to_target - drop_offset) < TIGHT_TOL, (
            f"Agent should be ~{drop_offset}m (drop_offset) from drop pose, "
            f"but is {dist_to_target:.2f}m away (pos={agent_pos[:2]})"
        )

    def test_drop_with_orientation(self, sim):
        """Dropped object receives the specified orientation."""
        agent = create_agent(sim, pose=Pose.from_xyz(1.9, 0, 0))
        box = create_pickable_box(sim)
        self._pick_box(agent, box, sim)

        yaw_quat = list(p.getQuaternionFromEuler([0, 0, np.pi / 2]))
        action = DropAction(
            drop_pose=Pose(position=[3, 0, 0], orientation=yaw_quat),
            use_approach=False,
            place_gently=True,
        )
        agent.add_action(action)

        run_until_idle(agent, sim)

        assert action.status is ActionStatus.COMPLETED
        box_orn = box.get_pose().orientation
        assert np.allclose(box_orn, yaw_quat, atol=TIGHT_TOL)

    def test_drop_no_attached_object_fails(self, sim):
        """DropAction fails if agent has nothing attached."""
        agent = create_agent(sim)
        action = DropAction(drop_pose=Pose(position=[1, 0, 0]), use_approach=False)
        agent.add_action(action)

        run_until_idle(agent, sim)

        assert action.status is ActionStatus.FAILED
        assert action.error_message is not None

    def test_drop_from_height(self, sim):
        """When place_gently=False, object is placed at drop_height above drop_pose."""
        drop_height = 0.5

        agent = create_agent(sim, pose=Pose.from_xyz(1.9, 0, 0))
        box = create_pickable_box(sim)
        self._pick_box(agent, box, sim)

        action = DropAction(
            drop_pose=Pose(position=[2, 0, 0]),
            use_approach=False,
            place_gently=False,
            drop_height=drop_height,
        )
        agent.add_action(action)

        run_until_idle(agent, sim)

        assert action.status is ActionStatus.COMPLETED
        # Object should be at z = 0.0 + 0.5 = 0.5
        box_z = box.get_pose().position[2]
        assert abs(box_z - drop_height) < PROXIMITY_TOL

    def test_drop_with_explicit_approach_pose(self, sim):
        """DropAction with explicit approach_pose not on the agent-target line.

        Agent at (1.9,0), drop pose at (4,0), approach_pose at (4,1) — off to the side.
        Agent should go to (4,1) first, then forward to drop pose, then retreat to (4,1).
        """
        agent = create_agent(sim, pose=Pose.from_xyz(1.9, 0, 0))
        box = create_pickable_box(sim)
        self._pick_box(agent, box, sim)

        approach = Pose.from_xyz(4, 1, 0)
        action = DropAction(
            drop_pose=Pose(position=[4, 0, 0]),
            use_approach=True,
            approach_pose=approach,
            drop_offset=0.0,
            place_gently=True,
        )
        agent.add_action(action)

        # Record trajectory to verify lateral approach
        trajectory: list = []
        dt = sim._dt
        for _ in range(20_000):
            sim.tick()
            agent.update(dt)
            trajectory.append(list(agent.get_pose().position[:2]))
            if agent.is_action_queue_empty() and not agent.is_moving:
                break

        assert action.status is ActionStatus.COMPLETED
        assert not box.is_attached()

        # Box should be at drop position
        box_pos = box.get_pose().position
        assert np.allclose(box_pos[:2], [4, 0], atol=COARSE_TOL)

        # Agent should have passed near approach_pose (4,1), not just gone straight
        dists_to_approach = [np.linalg.norm(np.array(t) - np.array([4, 1])) for t in trajectory]
        assert min(dists_to_approach) < COARSE_TOL, (
            f"Agent should have passed near explicit approach_pose (4,1), " f"closest was {min(dists_to_approach):.2f}m"
        )

        # Final position should be back near approach_pose after retreat
        final_pos = np.array(trajectory[-1])
        dist_to_approach = np.linalg.norm(final_pos - np.array([4, 1]))
        assert dist_to_approach < 0.3, f"Agent should retreat to approach_pose (4,1), but ended at {final_pos}"


# ---------------------------------------------------------------------------
# Full sequence: Move → Pick → Move → Drop
# ---------------------------------------------------------------------------


class TestActionSequenceIntegration:
    """End-to-end action sequence mimicking a warehouse pick-and-place task."""

    def test_move_pick_move_drop(self, sim):
        """Full workflow: move to object, pick, move to drop zone, drop."""
        agent = create_agent(sim)
        box = create_pickable_box(sim, pos=(3, 0, 0))

        move_to_pick = MoveAction(
            path=Path.from_positions([[2.5, 0, 0]]),
            final_orientation_align=False,
        )
        pick = PickAction(
            target_object_id=box.body_id,
            use_approach=True,
            approach_offset=0.5,
        )
        move_to_drop = MoveAction(
            path=Path.from_positions([[3, 3, 0]]),
            final_orientation_align=False,
        )
        drop = DropAction(
            drop_pose=Pose(position=[4, 3, 0]),
            use_approach=False,
            place_gently=True,
        )

        agent.add_action_sequence([move_to_pick, pick, move_to_drop, drop])

        run_until_idle(agent, sim)

        # All four actions should be completed
        assert move_to_pick.status is ActionStatus.COMPLETED
        assert pick.status is ActionStatus.COMPLETED
        assert move_to_drop.status is ActionStatus.COMPLETED
        assert drop.status is ActionStatus.COMPLETED

        # Box should be detached and near drop position
        assert not box.is_attached()
        box_pos = box.get_pose().position
        assert np.allclose(box_pos[:2], [4, 3], atol=COARSE_TOL)

    def test_move_pick_move_drop_with_wait(self, sim):
        """Full workflow with WaitAction interleaved."""
        agent = create_agent(sim)
        box = create_pickable_box(sim)

        actions = [
            MoveAction(path=Path.from_positions([[1.5, 0, 0]]), final_orientation_align=False),
            PickAction(target_object_id=box.body_id, use_approach=True, approach_offset=0.3),
            WaitAction(duration=0.1, action_type="loading"),
            MoveAction(path=Path.from_positions([[2, 2, 0]]), final_orientation_align=False),
            DropAction(drop_pose=Pose(position=[3, 2, 0]), use_approach=False, place_gently=True),
        ]
        agent.add_action_sequence(actions)

        run_until_idle(agent, sim)

        for a in actions:
            assert a.status is ActionStatus.COMPLETED, f"{a.__class__.__name__} not completed: {a.status}"

        assert not box.is_attached()

    def test_queue_size_decreases(self, sim):
        """Action queue size decreases as actions complete."""
        agent = create_agent(sim)

        actions = [
            MoveAction(path=Path.from_positions([[1, 0, 0]]), final_orientation_align=False),
            WaitAction(duration=0.01),
            MoveAction(path=Path.from_positions([[1, 1, 0]]), final_orientation_align=False),
        ]
        agent.add_action_sequence(actions)

        initial_size = agent.get_action_queue_size()
        assert initial_size == 3

        run_until_idle(agent, sim)

        assert agent.get_action_queue_size() == 0
        assert agent.is_action_queue_empty()


# ---------------------------------------------------------------------------
# JointAction
# ---------------------------------------------------------------------------


class PhysicsSimCore:
    """Minimal sim_core that advances PyBullet physics each tick.

    JointAction uses position motor control (p.setJointMotorControl2),
    which only takes effect after p.stepSimulation().
    """

    def __init__(self, dt: float = 1.0 / 240.0):
        self.sim_time = 0.0
        self._dt = dt
        self.sim_objects: list = []
        self._kinematic_objects: set = set()
        self._client = 0

    @property
    def client(self):
        return self._client

    def add_object(self, obj):
        self.sim_objects.append(obj)

    def remove_object(self, obj):
        if obj in self.sim_objects:
            self.sim_objects.remove(obj)

    def _mark_object_moved(self, object_id):
        pass

    def tick(self, n: int = 1):
        for _ in range(n):
            p.stepSimulation()
            self.sim_time += self._dt


def create_arm_agent(sim_core):
    """Create a fixed-base arm robot wired to *sim_core*."""
    return Agent.from_urdf(
        urdf_path=ARM_URDF,
        pose=Pose.from_xyz(0, 0, 0),
        use_fixed_base=True,
        sim_core=sim_core,
    )


def run_arm_until_idle(agent, sim_core, *, max_steps: int = 5000) -> int:
    """Run agent.update() with physics stepping until actions complete."""
    dt = sim_core._dt
    for step in range(max_steps):
        sim_core.tick()
        agent.update(dt)
        if agent.is_action_queue_empty():
            return step + 1
    raise AssertionError(f"Agent did not finish actions within {max_steps} steps")


@pytest.fixture
def physics_sim(pybullet_env):
    """PhysicsSimCore wired to the PyBullet session."""
    return PhysicsSimCore()


class TestJointActionIntegration:
    """Test JointAction execution with a real arm robot + physics."""

    def test_reaches_target_positions(self, physics_sim):
        """JointAction moves all joints to target and completes."""
        agent = create_arm_agent(physics_sim)
        targets = [0.5, 0.3, -0.3, 0.2]

        action = JointAction(target_joint_positions=targets, tolerance=0.05)
        agent.add_action(action)

        run_arm_until_idle(agent, physics_sim)

        assert action.status is ActionStatus.COMPLETED
        assert agent.are_joints_at_targets(targets, tolerance=0.05)

    def test_dict_targets_by_name(self, physics_sim):
        """JointAction accepts dict keyed by joint name."""
        agent = create_arm_agent(physics_sim)
        targets = {"base_to_shoulder": 0.4, "elbow_to_wrist": -0.5}

        action = JointAction(target_joint_positions=targets, tolerance=0.05)
        agent.add_action(action)

        run_arm_until_idle(agent, physics_sim)

        assert action.status is ActionStatus.COMPLETED
        assert agent.are_joints_at_targets(targets, tolerance=0.05)

    def test_sequential_joint_actions(self, physics_sim):
        """Two JointActions execute in sequence, both reach targets."""
        agent = create_arm_agent(physics_sim)
        targets_1 = [0.5, 0.3, -0.3, 0.2]
        targets_2 = [-0.3, 0.5, 0.2, -0.1]

        action1 = JointAction(target_joint_positions=targets_1, tolerance=0.05)
        action2 = JointAction(target_joint_positions=targets_2, tolerance=0.05)
        agent.add_action_sequence([action1, action2])

        run_arm_until_idle(agent, physics_sim)

        assert action1.status is ActionStatus.COMPLETED
        assert action2.status is ActionStatus.COMPLETED
        # Final state should match second action's targets
        assert agent.are_joints_at_targets(targets_2, tolerance=0.05)

    def test_duration_recorded(self, physics_sim):
        """JointAction records non-zero duration after completion."""
        agent = create_arm_agent(physics_sim)
        action = JointAction(target_joint_positions=[0.3, 0.0, 0.0, 0.0], tolerance=0.05)
        agent.add_action(action)

        run_arm_until_idle(agent, physics_sim)

        assert action.status is ActionStatus.COMPLETED
        duration = action.get_duration()
        assert duration is not None and duration > 0, f"Duration should be positive, got {duration}"
