"""
End-to-end tests for PyBulletFleet simulation workflows.

Each test runs a complete scenario through MultiRobotSimulationCore.step_once(),
verifying final state after action sequences complete.

Based on examples/:
- action_system_demo.py (Pick → Drop → Move → Wait)
- pick_drop_mobile_100robots_demo.py (bulk Pick → Move → Drop)
- path_following_demo.py (waypoint path following)
- collision_features_demo.py (multi-mode collision detection)
"""

import numpy as np
import pybullet as p
import pytest

from pybullet_fleet.action import (
    DropAction,
    JointAction,
    MoveAction,
    PickAction,
    WaitAction,
)
from pybullet_fleet.agent import Agent, AgentSpawnParams
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.geometry import Path, Pose
from pybullet_fleet.sim_object import ShapeParams, SimObject, SimObjectSpawnParams
from pybullet_fleet.types import (
    CollisionDetectionMethod,
    CollisionMode,
    MotionMode,
    SpatialHashCellSizeMode,
)

TOL = 0.15  # E2E position tolerance (generous — accounts for kinematic stepping)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def sim():
    """Headless kinematics sim (fast timestep for quick convergence)."""
    params = SimulationParams(
        gui=False,
        physics=False,
        monitor=False,
        timestep=0.1,
        target_rtf=0,  # max speed (no sleep)
        collision_detection_method=CollisionDetectionMethod.CLOSEST_POINTS,
        spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
        spatial_hash_cell_size=2.0,
        ignore_static_collision=True,
        log_level="warning",
    )
    sc = MultiRobotSimulationCore(params)
    sc.set_collision_spatial_hash_cell_size_mode()
    yield sc
    p.disconnect(sc.client)


def make_mobile_agent(sim, position=(0, 0, 0), motion_mode=MotionMode.OMNIDIRECTIONAL):
    """Spawn a mobile robot agent."""
    return Agent.from_params(
        AgentSpawnParams(
            urdf_path="robots/mobile_robot.urdf",
            initial_pose=Pose.from_xyz(*position),
            collision_mode=CollisionMode.NORMAL_3D,
            motion_mode=motion_mode,
        ),
        sim_core=sim,
    )


def make_pickable_box(sim, position=(2, 0, 0)):
    """Spawn a pickable box object."""
    return SimObject.from_params(
        SimObjectSpawnParams(
            visual_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            initial_pose=Pose.from_xyz(*position),
            mass=0.0,
            pickable=True,
        ),
        sim_core=sim,
    )


def run_until_done(sim, agents, *, max_steps=5000):
    """Step sim until all agent action queues are empty."""
    for step in range(max_steps):
        sim.step_once()
        if all(a.is_action_queue_empty() and not a.is_moving for a in agents):
            return step + 1
    raise AssertionError(f"Agents did not finish within {max_steps} steps")


# ============================================================================
# E2E: Pick → Move → Drop
# ============================================================================


class TestPickMoveDropE2E:
    """Full pick-move-drop workflow (based on action_system_demo.py)."""

    def test_pick_move_drop_single_agent(self, sim):
        agent = make_mobile_agent(sim, position=(0, 0, 0))
        box = make_pickable_box(sim, position=(2, 0, 0))

        # Verify initial box position
        box_pos = np.array(box.get_pose().position)
        assert np.linalg.norm(box_pos[:2] - np.array([2, 0])) < TOL, f"Box should start at (2,0), got {box_pos}"

        drop_pose = Pose.from_xyz(5, 3, 0)

        # Note: PickAction.target_object_id expects body_id (PyBullet ID), not object_id
        agent.add_action_sequence(
            [
                PickAction(
                    target_object_id=box.body_id,
                    use_approach=False,
                    pick_offset=0.3,
                    attach_relative_pose=Pose.from_xyz(0, 0, 0.3),
                ),
                MoveAction(
                    path=Path.from_positions([drop_pose.position]),
                    final_orientation_align=False,
                ),
                DropAction(
                    drop_pose=drop_pose,
                    use_approach=False,
                    drop_offset=0.3,
                ),
            ]
        )

        run_until_done(sim, [agent])

        # Box should be near drop position
        box_pos = np.array(box.get_pose().position)
        drop_pos = np.array(drop_pose.position)
        assert (
            np.linalg.norm(box_pos[:2] - drop_pos[:2]) < TOL
        ), f"Box should be near drop pose. box={box_pos}, drop={drop_pos}"

    def test_pick_move_drop_with_approach(self, sim):
        agent = make_mobile_agent(sim, position=(0, 0, 0))
        box = make_pickable_box(sim, position=(3, 0, 0))
        drop_pose = Pose.from_xyz(6, 4, 0)

        agent.add_action_sequence(
            [
                PickAction(
                    target_object_id=box.body_id,
                    use_approach=True,
                    approach_offset=1.0,
                    pick_offset=0.3,
                    attach_relative_pose=Pose.from_xyz(0, 0, 0.3),
                ),
                MoveAction(
                    path=Path.from_positions([drop_pose.position]),
                ),
                DropAction(
                    drop_pose=drop_pose,
                    use_approach=True,
                    approach_offset=1.0,
                    drop_offset=0.3,
                ),
            ]
        )

        run_until_done(sim, [agent])

        box_pos = np.array(box.get_pose().position)
        drop_pos = np.array(drop_pose.position)
        assert np.linalg.norm(box_pos[:2] - drop_pos[:2]) < TOL

    def test_move_then_wait(self, sim):
        agent = make_mobile_agent(sim, position=(0, 0, 0))
        goal = [4, 2, 0]

        agent.add_action_sequence(
            [
                MoveAction(path=Path.from_positions([goal])),
                WaitAction(duration=0.5, action_type="idle"),
            ]
        )

        steps = run_until_done(sim, [agent])

        pos = np.array(agent.get_pose().position)
        assert np.linalg.norm(pos[:2] - np.array(goal[:2])) < TOL
        # Move + Wait(0.5s) with timestep=0.1s → at least 5 steps for wait alone
        min_wait_steps = int(0.5 / sim.params.timestep)
        assert steps >= min_wait_steps, f"Expected ≥{min_wait_steps} steps (wait portion), got {steps}"


# ============================================================================
# E2E: Path Following
# ============================================================================


class TestPathFollowingE2E:
    """Path following to completion (based on path_following_demo.py)."""

    def test_square_path(self, sim):
        agent = make_mobile_agent(sim, position=(0, 0, 0))

        waypoints = [[2, 0, 0], [2, 2, 0], [0, 2, 0], [0, 0, 0]]
        agent.add_action_sequence(
            [
                MoveAction(path=Path.from_positions(waypoints)),
            ]
        )

        run_until_done(sim, [agent])

        pos = np.array(agent.get_pose().position)
        assert np.linalg.norm(pos[:2]) < TOL, f"Agent should return to origin, got {pos}"

    def test_multi_agent_paths(self, sim):
        agents = [make_mobile_agent(sim, position=(i * 3, 0, 0)) for i in range(3)]

        for i, agent in enumerate(agents):
            goal = [i * 3, 5, 0]
            agent.add_action_sequence(
                [
                    MoveAction(path=Path.from_positions([goal])),
                ]
            )

        run_until_done(sim, agents)

        for i, agent in enumerate(agents):
            pos = np.array(agent.get_pose().position)
            expected = np.array([i * 3, 5, 0])
            assert np.linalg.norm(pos[:2] - expected[:2]) < TOL


# ============================================================================
# E2E: Collision Detection During Movement
# ============================================================================


class TestCollisionDuringMovementE2E:
    """Collision detection while agents are moving (based on collision_features_demo.py)."""

    @pytest.fixture
    def sim_with_collision(self):
        params = SimulationParams(
            gui=False,
            physics=False,
            monitor=False,
            timestep=0.1,
            collision_detection_method=CollisionDetectionMethod.CLOSEST_POINTS,
            collision_margin=0.05,
            spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
            spatial_hash_cell_size=2.0,
            ignore_static_collision=False,
            log_level="warning",
        )
        sc = MultiRobotSimulationCore(params)
        sc.set_collision_spatial_hash_cell_size_mode()
        yield sc
        p.disconnect(sc.client)

    def test_collision_detected_during_step(self, sim_with_collision):
        """Two agents converging should produce collision pairs at some step."""
        sc = sim_with_collision
        a1 = make_mobile_agent(sc, position=(0, 0, 0))
        a2 = make_mobile_agent(sc, position=(3, 0, 0))

        a1.set_goal_pose(Pose.from_xyz(3, 0, 0))
        a2.set_goal_pose(Pose.from_xyz(0, 0, 0))

        collision_ever = False
        for _ in range(500):
            sc.step_once()
            if len(sc.get_active_collision_pairs()) > 0:
                collision_ever = True
                break

        assert collision_ever, "Converging agents should collide at some point"

    def test_disabled_mode_no_collision(self, sim_with_collision):
        """Agent with DISABLED collision mode is never in collision pairs."""
        sc = sim_with_collision
        a1 = make_mobile_agent(sc, position=(0, 0, 0))
        a2 = Agent.from_params(
            AgentSpawnParams(
                urdf_path="robots/mobile_robot.urdf",
                initial_pose=Pose.from_xyz(0.3, 0, 0),
                collision_mode=CollisionMode.DISABLED,
            ),
            sim_core=sc,
        )
        sc._moved_this_step.add(a1.object_id)

        sc.step_once()
        pairs = sc.get_active_collision_pairs()
        ids = {obj_id for pair in pairs for obj_id in pair}
        assert a2.object_id not in ids, "DISABLED agent should never appear in collision pairs"


# ============================================================================
# E2E: Multi-Agent Bulk Operations
# ============================================================================


class TestMultiAgentBulkE2E:
    """Multi-agent scenarios (based on pick_drop_mobile_100robots_demo.py)."""

    def test_multiple_agents_pick_and_drop(self, sim):
        """3 agents each pick a box, move, and drop it."""
        num_agents = 3
        agents = []
        boxes = []

        for i in range(num_agents):
            agent = make_mobile_agent(sim, position=(i * 4, 0, 0))
            box = make_pickable_box(sim, position=(i * 4, 1, 0))
            agents.append(agent)
            boxes.append(box)

        drop_y = 5.0
        for i, (agent, box) in enumerate(zip(agents, boxes)):
            drop_pose = Pose.from_xyz(i * 4, drop_y, 0)
            agent.add_action_sequence(
                [
                    PickAction(
                        target_object_id=box.body_id,
                        use_approach=False,
                        pick_offset=0.3,
                        attach_relative_pose=Pose.from_xyz(0, 0, 0.3),
                    ),
                    MoveAction(path=Path.from_positions([drop_pose.position])),
                    DropAction(drop_pose=drop_pose, use_approach=False, drop_offset=0.3),
                ]
            )

        run_until_done(sim, agents, max_steps=10000)

        for i, box in enumerate(boxes):
            box_pos = np.array(box.get_pose().position)
            expected_y = drop_y
            assert abs(box_pos[1] - expected_y) < TOL, f"Box {i} should be near y={expected_y}, got y={box_pos[1]}"


# ============================================================================
# E2E: Arm Robot Pick/Drop (based on pick_drop_arm_action_demo.py)
# ============================================================================


class TestArmPickDropE2E:
    """Robot arm JointAction → Pick → JointAction → Drop workflow."""

    @pytest.fixture(
        params=[
            pytest.param(None, id="mass_none"),  # Use URDF mass values
            pytest.param(0.0, id="mass_default"),  # AgentSpawnParams default (kinematic)
        ]
    )
    def arm_mass(self, request):
        """Mass to use for arm agent. None = URDF values; 0.0 = kinematic."""
        return request.param

    @pytest.fixture
    def arm_sim(self):
        """Physics-enabled sim for arm robot (needs stepSimulation for joints)."""
        params = SimulationParams(
            gui=False,
            physics=True,
            monitor=False,
            timestep=1.0 / 240.0,
            target_rtf=0,
            collision_detection_method=CollisionDetectionMethod.CONTACT_POINTS,
            spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
            spatial_hash_cell_size=2.0,
            ignore_static_collision=True,
            log_level="warning",
        )
        sc = MultiRobotSimulationCore(params)
        sc.set_collision_spatial_hash_cell_size_mode()
        yield sc
        p.disconnect(sc.client)

    def test_arm_joint_pick_joint_drop(self, arm_sim, arm_mass):
        """Arm moves to pick pose, picks box, moves to place pose, drops box."""
        sc = arm_sim

        arm = Agent.from_params(
            AgentSpawnParams(
                urdf_path="robots/arm_robot.urdf",
                initial_pose=Pose.from_xyz(0, 0, 0),
                mass=arm_mass,  # None = URDF values; 0.0 = kinematic
                collision_mode=CollisionMode.DISABLED,
                use_fixed_base=True,
            ),
            sim_core=sc,
        )

        box = SimObject.from_params(
            SimObjectSpawnParams(
                visual_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05]),
                collision_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05]),
                initial_pose=Pose.from_xyz(0.3, 0, 0.1),
                mass=0.0,
                pickable=True,
            ),
            sim_core=sc,
        )

        end_effector_link = arm.get_num_joints() - 1
        pick_joints = [1.5, 1.5, 1.5, 0.0]
        place_joints = [-1.5, 1.5, 1.5, 0.0]
        offset_pose = Pose.from_xyz(0, 0, 0.14)

        arm.add_action_sequence(
            [
                # Move to pick position
                JointAction(target_joint_positions=pick_joints, tolerance=0.05),
                # Pick object
                PickAction(
                    target_object_id=box.body_id,
                    use_approach=False,
                    attach_link=end_effector_link,
                    attach_relative_pose=offset_pose,
                ),
                # Move to place position (box attached)
                JointAction(target_joint_positions=place_joints, tolerance=0.05),
                # Drop object
                DropAction(
                    drop_pose=Pose.from_xyz(-0.3, 0, 0.1),
                    use_approach=False,
                ),
            ]
        )

        # Run simulation
        for _ in range(10000):
            sc.step_once()
            if arm.is_action_queue_empty():
                break
        else:
            raise AssertionError("Arm did not finish actions within 10000 steps")

        # Box should be detached
        assert not box.is_attached(), "Box should be detached after drop"

        # Box should be near drop position
        box_pos = np.array(box.get_pose().position)
        drop_pos = np.array([-0.3, 0, 0.1])
        assert np.linalg.norm(box_pos - drop_pos) < 0.3, f"Box should be near drop pose (-0.3, 0, 0.1), got {box_pos}"

    def test_pick_drop_with_inline_joint_targets(self, arm_sim, arm_mass):
        """PickAction/DropAction with joint_targets parameter (joints move inside action)."""
        sc = arm_sim

        arm = Agent.from_params(
            AgentSpawnParams(
                urdf_path="robots/arm_robot.urdf",
                initial_pose=Pose.from_xyz(0, 0, 0),
                mass=arm_mass,  # None = URDF values; 0.0 = kinematic
                collision_mode=CollisionMode.DISABLED,
                use_fixed_base=True,
            ),
            sim_core=sc,
        )

        box = SimObject.from_params(
            SimObjectSpawnParams(
                visual_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05]),
                collision_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05]),
                initial_pose=Pose.from_xyz(0.3, 0, 0.1),
                mass=0.0,
                pickable=True,
            ),
            sim_core=sc,
        )

        end_effector_link = arm.get_num_joints() - 1
        pick_joints = [1.5, 1.5, 1.5, 0.0]
        place_joints = [-1.5, 1.5, 1.5, 0.0]
        offset_pose = Pose.from_xyz(0, 0, 0.14)

        arm.add_action_sequence(
            [
                # Pick with joint_targets specified inline
                PickAction(
                    target_object_id=box.body_id,
                    use_approach=False,
                    attach_link=end_effector_link,
                    attach_relative_pose=offset_pose,
                    joint_targets=pick_joints,
                    joint_tolerance=0.05,
                ),
                # Drop with joint_targets specified inline
                DropAction(
                    drop_pose=Pose.from_xyz(-0.3, 0, 0.1),
                    use_approach=False,
                    joint_targets=place_joints,
                    joint_tolerance=0.05,
                ),
            ]
        )

        # Run simulation
        for _ in range(10000):
            sc.step_once()
            if arm.is_action_queue_empty():
                break
        else:
            raise AssertionError("Arm did not finish actions within 10000 steps")

        # Joints should have moved to place_joints (last joint action)
        joint_states = arm.get_all_joints_state()
        for i, target in enumerate(place_joints):
            pos = joint_states[i][0]
            assert abs(pos - target) < 0.1, f"Joint {i}: expected ~{target}, got {pos:.3f}"

        # Box should be detached
        assert not box.is_attached(), "Box should be detached after drop"

        # Box should be near drop position
        box_pos = np.array(box.get_pose().position)
        drop_pos = np.array([-0.3, 0, 0.1])
        assert np.linalg.norm(box_pos - drop_pos) < 0.3, f"Box should be near drop pose (-0.3, 0, 0.1), got {box_pos}"
