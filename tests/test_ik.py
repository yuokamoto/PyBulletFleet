"""Tests for IK (Inverse Kinematics) functionality.

Covers:
- Agent._solve_ik() — IK solver wrapper
- Agent._get_end_effector_link_index() — EE link auto-detection
- Agent.move_end_effector() — direct EE position command
- Agent._last_joint_targets / are_joints_at_targets() unification
- IKParams dataclass
"""

import numpy as np
import pybullet as p
import pytest

from pybullet_fleet.agent import Agent
from pybullet_fleet.geometry import Pose
from pybullet_fleet.sim_object import ShapeParams
from tests.conftest import MockSimCore

ARM_URDF = "robots/arm_robot.urdf"
RAIL_ARM_URDF = "robots/rail_arm_robot.urdf"
MOBILE_MANIP_URDF = "robots/mobile_manipulator.urdf"

# ---------------------------------------------------------------------------
# Assertion tolerances
# ---------------------------------------------------------------------------

EE_POS_TOL = 0.05  # EE position accuracy after IK or kinematic steps (m)
IK_REACH_TOL = 0.02  # IK reachability check — tighter for pure revolute arm
IK_REACH_TOL_RAIL = 0.05  # IK reachability check — looser for prismatic chain


@pytest.fixture
def arm_agent(pybullet_env):
    """Kinematic arm agent (mass=0, physics=False)."""
    sim_core = MockSimCore(physics=False)
    sim_core._client = pybullet_env
    agent = Agent.from_urdf(
        urdf_path=ARM_URDF,
        pose=Pose.from_xyz(0, 0, 0),
        use_fixed_base=True,
        mass=0.0,
        sim_core=sim_core,
    )
    return agent, sim_core


# ============================================================
# T1: Agent._get_end_effector_link_index
# ============================================================


class TestGetEndEffectorLinkIndex:
    """Tests for Agent._get_end_effector_link_index()."""

    def test_auto_detect_returns_last_link(self, arm_agent):
        """None → last link index (arm_robot.urdf has 4 joints → index 3)."""
        agent, _ = arm_agent
        ee_idx = agent._get_end_effector_link_index()
        assert ee_idx == len(agent.joint_info) - 1
        assert ee_idx == 3  # arm_robot.urdf: 4 joints

    def test_explicit_int(self, arm_agent):
        """Explicit int is passed through as-is (resolve_link_index identity)."""
        agent, _ = arm_agent
        # Int passthrough: callers with Union[int, str, None] can pass int directly
        assert agent._get_end_effector_link_index(2) == 2

    def test_explicit_str(self, arm_agent):
        """Explicit link name resolves to correct index."""
        agent, _ = arm_agent
        # arm_robot.urdf: end_effector link is child of joint 3
        idx = agent._get_end_effector_link_index("end_effector")
        assert idx == 3

    def test_mesh_agent_returns_negative(self, pybullet_env):
        """Mesh agent (no joints) returns -1."""
        sim_core = MockSimCore(physics=False)
        sim_core._client = pybullet_env
        agent = Agent.from_mesh(
            visual_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 0),
            sim_core=sim_core,
        )
        assert agent._get_end_effector_link_index() == -1


# ============================================================
# T0a: Agent._check_ee_pose (shared FK + pose check core)
# ============================================================


class TestCheckEePose:
    """Tests for Agent._check_ee_pose() — core FK position/orientation check."""

    def test_position_within_tolerance(self, arm_agent):
        """Returns True when EE position is within tolerance."""
        agent, _ = arm_agent
        ee_idx = agent._get_end_effector_link_index()
        link_state = p.getLinkState(agent.body_id, ee_idx, computeForwardKinematics=1, physicsClientId=agent._pid)
        current_pos = list(link_state[0])
        assert agent._check_ee_pose(ee_idx, current_pos) is True

    def test_position_outside_tolerance(self, arm_agent):
        """Returns False when EE position is far from target."""
        agent, _ = arm_agent
        ee_idx = agent._get_end_effector_link_index()
        assert agent._check_ee_pose(ee_idx, [10.0, 10.0, 10.0]) is False

    def test_orientation_match(self, arm_agent):
        """Returns True when both position and orientation match."""
        agent, _ = arm_agent
        ee_idx = agent._get_end_effector_link_index()
        link_state = p.getLinkState(agent.body_id, ee_idx, computeForwardKinematics=1, physicsClientId=agent._pid)
        current_pos = list(link_state[0])
        current_orn = list(link_state[1])
        assert agent._check_ee_pose(ee_idx, current_pos, target_orientation=current_orn) is True

    def test_orientation_mismatch(self, arm_agent):
        """Returns False when position matches but orientation doesn't."""
        agent, _ = arm_agent
        ee_idx = agent._get_end_effector_link_index()
        link_state = p.getLinkState(agent.body_id, ee_idx, computeForwardKinematics=1, physicsClientId=agent._pid)
        current_pos = list(link_state[0])
        wrong_orn = [0.707, 0.0, 0.707, 0.0]
        assert agent._check_ee_pose(ee_idx, current_pos, target_orientation=wrong_orn, orientation_tolerance=0.1) is False

    def test_negative_ee_index_returns_false(self, arm_agent):
        """Negative ee_link_index → False."""
        agent, _ = arm_agent
        assert agent._check_ee_pose(-1, [0.0, 0.0, 0.75]) is False

    def test_non_unit_quaternion_does_not_false_pass(self, arm_agent):
        """Non-unit quaternions should not cause false positive via |dot| > 1."""
        agent, _ = arm_agent
        ee_idx = agent._get_end_effector_link_index()
        link_state = p.getLinkState(agent.body_id, ee_idx, computeForwardKinematics=1, physicsClientId=agent._pid)
        current_pos = list(link_state[0])
        # Scaled quaternion — NOT unit length. A very different orientation
        # but with large magnitude could make un-clamped dot exceed 1.
        non_unit_orn = [2.0, 0.0, 0.0, 2.0]  # magnitude ~2.83, direction ≠ actual
        result = agent._check_ee_pose(ee_idx, current_pos, target_orientation=non_unit_orn, orientation_tolerance=0.15)
        # Should not incorrectly pass — the orientation is wrong
        assert result is False


# ============================================================
# T0b: Agent.are_ee_at_target
# ============================================================


class TestAreEEAtTarget:
    """Tests for Agent.are_ee_at_target()."""

    def test_returns_true_when_ee_near_target(self, arm_agent):
        """Returns True when EE is within tolerance of target position."""
        agent, sim_core = arm_agent
        # Home position — EE should already be here
        ee_idx = agent._get_end_effector_link_index()
        link_state = p.getLinkState(agent.body_id, ee_idx, computeForwardKinematics=1, physicsClientId=agent._pid)
        current_pos = list(link_state[0])
        assert agent.are_ee_at_target(current_pos) is True

    def test_returns_false_when_ee_far_from_target(self, arm_agent):
        """Returns False when EE is far from target position."""
        agent, _ = arm_agent
        assert agent.are_ee_at_target([10.0, 10.0, 10.0]) is False

    def test_with_orientation_near(self, arm_agent):
        """Returns True when both position and orientation match."""
        agent, _ = arm_agent
        ee_idx = agent._get_end_effector_link_index()
        link_state = p.getLinkState(agent.body_id, ee_idx, computeForwardKinematics=1, physicsClientId=agent._pid)
        current_pos = list(link_state[0])
        current_orn = list(link_state[1])
        assert agent.are_ee_at_target(current_pos, target_orientation=current_orn) is True

    def test_with_orientation_mismatch(self, arm_agent):
        """Returns False when position matches but orientation does not."""
        agent, sim_core = arm_agent
        # Move arm to a non-zero configuration first
        agent.set_all_joints_targets([1.0, 0.5, -0.5, 0.3])
        for _ in range(2000):
            sim_core.tick()
            agent.update(sim_core._dt)
        ee_idx = agent._get_end_effector_link_index()
        link_state = p.getLinkState(agent.body_id, ee_idx, computeForwardKinematics=1, physicsClientId=agent._pid)
        current_pos = list(link_state[0])
        # Use a very different orientation
        wrong_orn = [0.707, 0.0, 0.707, 0.0]
        assert agent.are_ee_at_target(current_pos, target_orientation=wrong_orn, orientation_tolerance=0.1) is False

    def test_mesh_agent_returns_false(self, pybullet_env):
        """Mesh agent returns False."""
        sim_core = MockSimCore(physics=False)
        sim_core._client = pybullet_env
        agent = Agent.from_mesh(
            visual_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 0),
            sim_core=sim_core,
        )
        assert agent.are_ee_at_target([0.0, 0.0, 0.0]) is False


# ============================================================
# T1: Agent._solve_ik
# ============================================================


class TestSolveIK:
    """Tests for Agent._solve_ik()."""

    def test_ik_solution_reaches_target(self, arm_agent):
        """Applying IK solution should place EE near the target position."""
        agent, _ = arm_agent
        target = [0.0, 0.0, 0.75]  # Home position — always reachable
        ee_idx = agent._get_end_effector_link_index()
        angles = agent._solve_ik(target, ee_link_index=ee_idx)

        # Apply the solution directly
        for i, angle in enumerate(angles):
            p.resetJointState(agent.body_id, i, angle, physicsClientId=agent._pid)

        # Check EE position
        ee_idx = agent._get_end_effector_link_index()
        link_state = p.getLinkState(agent.body_id, ee_idx, computeForwardKinematics=1, physicsClientId=agent._pid)
        actual_pos = np.array(link_state[0])
        target_pos = np.array(target)
        distance = np.linalg.norm(actual_pos - target_pos)
        assert distance < EE_POS_TOL, f"EE at {actual_pos}, target {target_pos}, distance {distance:.4f}"

    def test_ik_with_orientation(self, arm_agent):
        """_solve_ik with target_orientation returns valid joint angles that achieve the orientation."""
        agent, _ = arm_agent
        ee_idx = agent._get_end_effector_link_index()
        target_pos = [0.0, 0.0, 0.75]
        target_orn = [0.0, 0.0, 0.0, 1.0]  # identity quaternion
        angles = agent._solve_ik(target_pos, target_orientation=target_orn, ee_link_index=ee_idx)
        assert isinstance(angles, list)
        assert len(angles) == agent.get_num_joints()

        # Apply solution and verify position + orientation
        for i, angle in enumerate(angles):
            p.resetJointState(agent.body_id, i, angle, physicsClientId=agent._pid)
        link_state = p.getLinkState(agent.body_id, ee_idx, computeForwardKinematics=1, physicsClientId=agent._pid)
        actual_pos = np.array(link_state[0])
        assert np.linalg.norm(actual_pos - np.array(target_pos)) < EE_POS_TOL
        actual_orn = np.array(link_state[1])
        dot = abs(np.dot(actual_orn, np.array(target_orn)))
        assert dot > 0.95, f"EE orientation mismatch: dot={dot:.3f}, actual={actual_orn}"

    def test_ee_index_negative_returns_empty_list(self, arm_agent):
        """_solve_ik with ee_index < 0 returns empty list (not zeros)."""
        agent, _ = arm_agent
        result = agent._solve_ik([0.0, 0.0, 0.75], ee_link_index=-1)
        assert result == [], f"Expected empty list for invalid ee_index, got {result}"

    def test_solves_with_empty_quartiles(self, arm_agent):
        """_solve_ik works even with no quartile seeds (only current-position seed)."""
        agent, _ = arm_agent
        from pybullet_fleet.agent import IKParams

        sim_core = agent.sim_core
        agent2 = Agent.from_urdf(
            urdf_path=ARM_URDF,
            pose=Pose.from_xyz(0, 0, 0),
            use_fixed_base=True,
            mass=0.0,
            sim_core=sim_core,
            ik_params=IKParams(seed_quartiles=()),  # no quartile seeds
        )
        # Should still work — current-position seed is always present
        target = [0.0, 0.3, 0.3]
        ee_idx = agent2._get_end_effector_link_index()
        angles = agent2._solve_ik(target, ee_link_index=ee_idx)
        assert len(angles) > 0

    @pytest.mark.parametrize(
        "target",
        [
            [0.0, 0.3, 0.3],  # Y+ direction — requires J0 ~+90°
            [0.0, -0.3, 0.3],  # Y- direction — requires J0 ~-90°
            [0.3, 0.0, 0.3],  # X+ direction — another axis
        ],
        ids=["y_positive", "y_negative", "x_positive"],
    )
    def test_ik_converges_for_large_joint_displacement(self, arm_agent, target):
        """_solve_ik should converge even when solution requires large displacement from rest."""
        agent, _ = arm_agent
        ee_idx = agent._get_end_effector_link_index()
        angles = agent._solve_ik(target, ee_link_index=ee_idx)
        assert len(angles) == agent.get_num_joints()
        # Verify the EE actually reaches the target
        reachable = agent._check_ik_reachability(angles, target, ee_idx, tolerance=IK_REACH_TOL)
        assert reachable, f"IK solution {angles} does not place EE within {IK_REACH_TOL}m of target {target}"


# ============================================================
# T1b: Agent._check_ik_reachability
# ============================================================


class TestCheckIKReachability:
    """Tests for Agent._check_ik_reachability()."""

    def test_reachable_target_returns_true(self, arm_agent):
        """IK solution for reachable target → True."""
        agent, _ = arm_agent
        ee_idx = agent._get_end_effector_link_index()
        target = [0.0, 0.0, 0.75]
        angles = agent._solve_ik(target, ee_link_index=ee_idx)
        assert agent._check_ik_reachability(angles, target, ee_idx) is True

    def test_unreachable_target_returns_false(self, arm_agent):
        """IK solution for unreachable target → False."""
        agent, _ = arm_agent
        ee_idx = agent._get_end_effector_link_index()
        target = [100.0, 100.0, 100.0]
        angles = agent._solve_ik(target, ee_link_index=ee_idx)
        assert agent._check_ik_reachability(angles, target, ee_idx) is False

    def test_restores_original_positions(self, arm_agent):
        """Joint positions are restored after reachability check."""
        agent, _ = arm_agent
        # Set known positions
        for i in range(agent.get_num_joints()):
            p.resetJointState(agent.body_id, i, 0.0, physicsClientId=agent._pid)
        saved = [agent.get_joint_state(i)[0] for i in range(agent.get_num_joints())]

        ee_idx = agent._get_end_effector_link_index()
        target = [0.0, 0.0, 0.75]
        angles = agent._solve_ik(target, ee_link_index=ee_idx)
        agent._check_ik_reachability(angles, target, ee_idx)

        after = [agent.get_joint_state(i)[0] for i in range(agent.get_num_joints())]
        for s, a in zip(saved, after):
            assert abs(s - a) < 1e-6, "Joint positions not restored after reachability check"

    def test_negative_ee_returns_false(self, arm_agent):
        """Negative ee_link_index → False."""
        agent, _ = arm_agent
        assert agent._check_ik_reachability([0.0] * 4, [0, 0, 0.75], -1) is False

    def test_orientation_check_pass(self, arm_agent):
        """When target_orientation is provided and satisfied, returns True."""
        agent, _ = arm_agent
        ee_idx = agent._get_end_effector_link_index()
        target_pos = [0.0, 0.0, 0.75]
        target_orn = [0.0, 0.0, 0.0, 1.0]
        angles = agent._solve_ik(target_pos, target_orientation=target_orn, ee_link_index=ee_idx)
        result = agent._check_ik_reachability(angles, target_pos, ee_idx, target_orientation=target_orn)
        assert result is True

    def test_orientation_check_fail(self, arm_agent):
        """When target_orientation doesn't match, returns False even if position matches."""
        agent, _ = arm_agent
        ee_idx = agent._get_end_effector_link_index()
        target_pos = [0.0, 0.0, 0.75]
        # Solve IK for identity orientation
        angles = agent._solve_ik(target_pos, target_orientation=[0.0, 0.0, 0.0, 1.0], ee_link_index=ee_idx)
        # But check reachability against a very different orientation
        result = agent._check_ik_reachability(
            angles,
            target_pos,
            ee_idx,
            target_orientation=[0.707, 0.0, 0.707, 0.0],
            orientation_tolerance=0.1,
        )
        assert result is False


# ============================================================
# T2: Agent.move_end_effector
# ============================================================


class TestMoveEndEffector:
    """Tests for Agent.move_end_effector()."""

    def test_returns_true_for_reachable(self, arm_agent):
        """move_end_effector returns True for reachable target."""
        agent, _ = arm_agent
        result = agent.move_end_effector([0.0, 0.0, 0.75])
        assert result is True

    def test_returns_false_for_unreachable(self, arm_agent):
        """move_end_effector returns False for unreachable target."""
        agent, _ = arm_agent
        result = agent.move_end_effector([100.0, 100.0, 100.0])
        assert result is False
        # Targets still set (best-effort)
        assert len(agent._last_joint_targets) > 0

    def test_returns_true_for_y_positive_target(self, arm_agent):
        """move_end_effector should reach Y+ target (requires large J0 rotation)."""
        agent, _ = arm_agent
        result = agent.move_end_effector([0.0, 0.3, 0.3])
        assert result is True, "Y+ target should be reachable with iterative IK"

    def test_sets_joint_targets(self, arm_agent):
        """move_end_effector should set joint targets."""
        agent, _ = arm_agent
        agent.move_end_effector([0.0, 0.0, 0.75])
        # Targets should be populated
        assert len(agent._last_joint_targets) > 0

    def test_ee_reaches_target_after_steps(self, arm_agent):
        """After enough update steps, EE should be near target."""
        agent, sim_core = arm_agent
        target = [0.0, 0.0, 0.75]  # Home position
        agent.move_end_effector(target)

        dt = sim_core._dt
        for _ in range(2000):
            sim_core.tick()
            agent.update(dt)

        # Check EE position
        ee_idx = agent._get_end_effector_link_index()
        link_state = p.getLinkState(agent.body_id, ee_idx, computeForwardKinematics=1, physicsClientId=agent._pid)
        actual_pos = np.array(link_state[0])
        distance = np.linalg.norm(actual_pos - np.array(target))
        assert distance < EE_POS_TOL, f"EE at {actual_pos}, target {target}, distance {distance:.4f}"

    def test_with_orientation(self, arm_agent):
        """move_end_effector with orientation achieves target pose."""
        agent, sim_core = arm_agent
        target_pos = [0.0, 0.0, 0.75]
        target_orn = [0.0, 0.0, 0.0, 1.0]
        result = agent.move_end_effector(target_pos, target_orientation=target_orn)
        assert result is True

        # Step to let joints settle
        for _ in range(2000):
            sim_core.tick()
            agent.update(sim_core._dt)

        ee_idx = agent._get_end_effector_link_index()
        link_state = p.getLinkState(agent.body_id, ee_idx, computeForwardKinematics=1, physicsClientId=agent._pid)
        actual_pos = np.array(link_state[0])
        assert np.linalg.norm(actual_pos - np.array(target_pos)) < EE_POS_TOL

        actual_orn = np.array(link_state[1])
        dot = abs(np.dot(actual_orn, np.array(target_orn)))
        assert dot > 0.95, f"Orientation mismatch: dot={dot:.3f}, actual={actual_orn}"

    def test_mesh_agent_returns_false(self, pybullet_env):
        """Calling move_end_effector on mesh agent returns False."""
        sim_core = MockSimCore(physics=False)
        sim_core._client = pybullet_env
        agent = Agent.from_mesh(
            visual_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 0),
            sim_core=sim_core,
        )
        result = agent.move_end_effector([0.1, 0.0, 0.5])
        assert result is False


# ============================================================
# T3: _last_joint_targets unification
# ============================================================


class TestLastJointTargets:
    """Tests for _last_joint_targets and are_joints_at_targets(None)."""

    def test_are_joints_at_targets_no_args_returns_true_when_settled(self, arm_agent):
        """are_joints_at_targets() returns True when settled; targets persist."""
        agent, sim_core = arm_agent
        agent.set_all_joints_targets([0.1, 0.2, -0.1, 0.05])
        for _ in range(2000):
            sim_core.tick()
            agent.update(sim_core._dt)
        assert agent.are_joints_at_targets()
        # Targets persist after arrival (not deleted)
        assert 0 in agent._last_joint_targets
        assert agent._last_joint_targets[0] == 0.1

    def test_are_joints_at_targets_no_args_returns_false_before_settled(self, arm_agent):
        """are_joints_at_targets() returns False while joints are still moving."""
        agent, sim_core = arm_agent
        agent.set_all_joints_targets([1.0, 1.0, -1.0, 0.5])
        # Only step once — joints should not have arrived yet
        sim_core.tick()
        agent.update(sim_core._dt)
        assert agent.are_joints_at_targets(tolerance=0.01) is False

    def test_are_joints_at_targets_no_targets_set(self, arm_agent, caplog):
        """are_joints_at_targets() returns True with warning when no targets ever set."""
        agent, _ = arm_agent
        import logging

        with caplog.at_level(logging.WARNING):
            result = agent.are_joints_at_targets()
        assert result is True
        assert any(
            "no targets" in msg.lower() for msg in caplog.messages
        ), f"Expected warning about no targets, got: {caplog.messages}"

    def test_last_joint_targets_property(self, arm_agent):
        """last_joint_targets property returns a copy."""
        agent, _ = arm_agent
        agent.set_joint_target(1, 0.5)
        prop = agent.last_joint_targets
        assert prop == {1: 0.5}
        prop[1] = 999.0  # mutate copy
        assert agent.last_joint_targets == {1: 0.5}  # original unchanged


# ============================================================
# T4: IKParams dataclass
# ============================================================


class TestIKParams:
    """Tests for IKParams dataclass and its integration with Agent."""

    def test_default_values(self):
        """IKParams() should have sensible defaults."""
        from pybullet_fleet.agent import IKParams

        cfg = IKParams()
        assert cfg.max_outer_iterations == 5
        assert cfg.convergence_threshold == 0.01
        assert cfg.max_inner_iterations == 200
        assert cfg.residual_threshold == 1e-4
        assert cfg.reachability_tolerance == 0.02
        assert cfg.seed_quartiles == (0.25, 0.5, 0.75)
        assert cfg.ik_joint_names is None

    def test_custom_values(self):
        """IKParams accepts custom values."""
        from pybullet_fleet.agent import IKParams

        cfg = IKParams(
            max_outer_iterations=10,
            convergence_threshold=0.05,
            max_inner_iterations=500,
            residual_threshold=1e-6,
            reachability_tolerance=0.1,
            seed_quartiles=(0.1, 0.5, 0.9),
        )
        assert cfg.max_outer_iterations == 10
        assert cfg.convergence_threshold == 0.05
        assert cfg.max_inner_iterations == 500
        assert cfg.residual_threshold == 1e-6
        assert cfg.reachability_tolerance == 0.1
        assert cfg.seed_quartiles == (0.1, 0.5, 0.9)

    def test_agent_default_ik_params(self, arm_agent):
        """Agent should have default IKParams when none provided."""
        from pybullet_fleet.agent import IKParams

        agent, _ = arm_agent
        assert hasattr(agent, "_ik_params")
        assert isinstance(agent._ik_params, IKParams)
        assert agent._ik_params.max_outer_iterations == 5

    def test_agent_custom_ik_params(self, pybullet_env):
        """Agent.from_urdf should accept ik_params parameter."""
        from pybullet_fleet.agent import IKParams

        sim_core = MockSimCore(physics=False)
        sim_core._client = pybullet_env
        cfg = IKParams(max_outer_iterations=10, convergence_threshold=0.05)
        agent = Agent.from_urdf(
            urdf_path=ARM_URDF,
            pose=Pose.from_xyz(0, 0, 0),
            use_fixed_base=True,
            mass=0.0,
            sim_core=sim_core,
            ik_params=cfg,
        )
        assert agent._ik_params is cfg
        assert agent._ik_params.max_outer_iterations == 10


# ============================================================
# Rail arm IK (prismatic + revolute)
# ============================================================


@pytest.fixture
def rail_arm_agent(pybullet_env):
    """Kinematic rail arm agent (1 prismatic + 4 revolute, mass=0)."""
    sim_core = MockSimCore(physics=False)
    sim_core._client = pybullet_env
    agent = Agent.from_urdf(
        urdf_path=RAIL_ARM_URDF,
        pose=Pose.from_xyz(0, 0, 0),
        use_fixed_base=True,
        mass=0.0,
        sim_core=sim_core,
    )
    return agent, sim_core


class TestRailArmIK:
    """IK unit tests with prismatic + revolute chain (rail_arm_robot.urdf).

    The rail arm has 5 joints: 1 prismatic (Z, 0-1m) + 4 revolute.
    IK must utilise the prismatic joint to reach heights beyond the
    pure revolute arm's workspace.
    """

    def test_solve_ik_reaches_target(self, rail_arm_agent):
        """_solve_ik produces a solution that places EE near the target."""
        agent, _ = rail_arm_agent
        target = [0.1, 0.0, 1.2]  # requires rail extension
        ee_idx = agent._get_end_effector_link_index()
        angles = agent._solve_ik(target, ee_link_index=ee_idx)

        assert len(angles) == agent.get_num_joints()
        reachable = agent._check_ik_reachability(angles, target, ee_idx, tolerance=IK_REACH_TOL_RAIL)
        assert reachable, f"IK solution does not reach target {target}"

    def test_solve_ik_uses_prismatic_for_high_target(self, rail_arm_agent):
        """IK for a high target should extend the prismatic joint and reach the target."""
        agent, _ = rail_arm_agent
        target = [0.1, 0.0, 1.5]  # well above pure arm reach
        ee_idx = agent._get_end_effector_link_index()
        angles = agent._solve_ik(target, ee_link_index=ee_idx)

        # Prismatic joint (index 0) should be significantly > 0
        assert angles[0] > 0.2, f"Prismatic joint should extend for high target, got {angles[0]:.3f}m"
        # EE must actually reach the target
        reachable = agent._check_ik_reachability(angles, target, ee_idx, tolerance=IK_REACH_TOL_RAIL)
        assert reachable, f"IK solution does not reach target {target}"

    def test_solve_ik_low_target_keeps_rail_near_zero(self, rail_arm_agent):
        """IK for a target within arm-only reach keeps rail near zero and reaches target."""
        agent, _ = rail_arm_agent
        target = [0.1, 0.0, 0.5]  # within pure arm reach
        ee_idx = agent._get_end_effector_link_index()
        angles = agent._solve_ik(target, ee_link_index=ee_idx)

        # Prismatic joint should stay near 0
        assert abs(angles[0]) < 0.3, f"Prismatic joint should stay near zero for low target, got {angles[0]:.3f}m"
        # EE must actually reach the target
        reachable = agent._check_ik_reachability(angles, target, ee_idx, tolerance=IK_REACH_TOL_RAIL)
        assert reachable, f"IK solution does not reach target {target}"

    def test_move_end_effector_returns_true(self, rail_arm_agent):
        """move_end_effector returns True for reachable rail arm target."""
        agent, _ = rail_arm_agent
        result = agent.move_end_effector([0.1, 0.0, 1.2])
        assert result is True

    def test_move_end_effector_unreachable(self, rail_arm_agent):
        """move_end_effector returns False for unreachable target."""
        agent, _ = rail_arm_agent
        result = agent.move_end_effector([10.0, 10.0, 10.0])
        assert result is False

    def test_ee_reaches_after_kinematic_steps(self, rail_arm_agent):
        """After kinematic interpolation steps, EE should be near target."""
        agent, sim_core = rail_arm_agent
        target = [0.1, 0.0, 1.2]
        agent.move_end_effector(target)

        dt = sim_core._dt
        # Upper bound: max displacement / min velocity + buffer.
        # Prismatic range 1m @ 0.5 m/s = 2s; revolute ~3 rad @ 2.0 rad/s = 1.5s
        # Worst case ~2s; at 240 Hz = 480 steps. Use 2x buffer.
        max_steps = int(2.0 / dt * 2)
        for _ in range(max_steps):
            sim_core.tick()
            agent.update(dt)

        ee_idx = agent._get_end_effector_link_index()
        link_state = p.getLinkState(agent.body_id, ee_idx, computeForwardKinematics=1, physicsClientId=agent._pid)
        actual_pos = np.array(link_state[0])
        distance = np.linalg.norm(actual_pos - np.array(target))
        assert distance < EE_POS_TOL, f"EE at {actual_pos}, target {target}, distance {distance:.4f}"


# ============================================================
# Mobile Manipulator IK (fixed joint + continuous wheels)
# ============================================================

MOBILE_MANIP_EE_LINK = 6  # end_effector link index


@pytest.fixture
def mobile_manip_agent(pybullet_env):
    """Kinematic mobile manipulator (2 wheels + 1 fixed + 4 revolute, mass=0)."""
    sim_core = MockSimCore(physics=False)
    sim_core._client = pybullet_env
    agent = Agent.from_urdf(
        urdf_path=MOBILE_MANIP_URDF,
        pose=Pose.from_xyz(0, 0, 0.3),
        use_fixed_base=False,
        mass=0.0,
        sim_core=sim_core,
    )
    return agent, sim_core


class TestMobileManipulatorIK:
    """IK for mobile_manipulator.urdf — exercises the fixed-joint mapping.

    mobile_manipulator has 7 joints:
      0: base_to_left_wheel  (continuous, movable)
      1: base_to_right_wheel (continuous, movable)
      2: base_to_mount       (FIXED)
      3: mount_to_shoulder   (revolute)
      4: shoulder_to_elbow   (revolute)
      5: elbow_to_wrist      (revolute)
      6: wrist_to_end        (revolute)

    IK must correctly handle:
    - Fixed joint skipped by calculateInverseKinematics
    - Continuous wheel joints (lower >= upper)
    - Returned list length matching joint_info length
    """

    def test_solve_ik_returns_correct_length(self, mobile_manip_agent):
        """_solve_ik must return len(joint_info) values, not just movable count."""
        agent, _ = mobile_manip_agent
        assert len(agent.joint_info) == 7  # 6 movable + 1 fixed
        target = [0.5, 0.0, 0.6]
        angles = agent._solve_ik(target, ee_link_index=MOBILE_MANIP_EE_LINK)
        assert len(angles) == len(agent.joint_info), f"Expected {len(agent.joint_info)} angles, got {len(angles)}"

    def test_solve_ik_reaches_target(self, mobile_manip_agent):
        """IK solution should place EE near the target position."""
        agent, _ = mobile_manip_agent
        # Target in front of robot, within arm reach
        target = [0.5, 0.0, 0.6]
        angles = agent._solve_ik(target, ee_link_index=MOBILE_MANIP_EE_LINK)
        assert len(angles) == 7

        # Apply and check FK
        for i, angle in enumerate(angles):
            p.resetJointState(agent.body_id, i, angle, physicsClientId=agent._pid)
        link_state = p.getLinkState(
            agent.body_id,
            MOBILE_MANIP_EE_LINK,
            computeForwardKinematics=1,
            physicsClientId=agent._pid,
        )
        distance = np.linalg.norm(np.array(link_state[0]) - np.array(target))
        assert distance < EE_POS_TOL, f"EE distance {distance:.4f} > {EE_POS_TOL}"

    def test_solve_ik_preserves_fixed_joint(self, mobile_manip_agent):
        """Fixed joint (index 2) should keep its original value (0.0)."""
        agent, _ = mobile_manip_agent
        target = [0.5, 0.0, 0.6]
        angles = agent._solve_ik(target, ee_link_index=MOBILE_MANIP_EE_LINK)
        # Joint 2 is fixed → its angle must remain 0.0
        assert angles[2] == 0.0, f"Fixed joint angle should be 0.0, got {angles[2]}"

    def test_solve_ik_locks_wheels_at_nonzero_position(self, mobile_manip_agent):
        """Continuous wheel joints stay at current (non-zero) position after IK.

        Setting wheels to a non-zero value before IK ensures the lock
        mechanism (_LOCK_EPS) is actually preserving the saved position,
        not just coincidentally returning zero.
        """
        agent, _ = mobile_manip_agent
        # Move wheels to a non-zero position (sync both PyBullet and kinematic cache)
        wheel_pos = 1.5
        for idx in (0, 1):
            agent._kinematic_joint_positions[idx] = wheel_pos
            p.resetJointState(agent.body_id, idx, wheel_pos, physicsClientId=agent._pid)

        target = [0.5, 0.0, 0.6]
        angles = agent._solve_ik(target, ee_link_index=MOBILE_MANIP_EE_LINK)
        # Wheels should stay at saved position (1.5), not reset to 0
        assert abs(angles[0] - wheel_pos) < 1e-3, f"Left wheel should stay at {wheel_pos}, got {angles[0]}"
        assert abs(angles[1] - wheel_pos) < 1e-3, f"Right wheel should stay at {wheel_pos}, got {angles[1]}"

    def test_move_end_effector_reachable(self, mobile_manip_agent):
        """move_end_effector returns True for reachable target."""
        agent, _ = mobile_manip_agent
        target = [0.5, 0.0, 0.6]
        result = agent.move_end_effector(
            target,
            end_effector_link=MOBILE_MANIP_EE_LINK,
        )
        assert result is True

    def test_ee_reaches_after_kinematic_steps(self, mobile_manip_agent):
        """After kinematic interpolation steps, EE should be near target."""
        agent, sim_core = mobile_manip_agent
        target = [0.5, 0.0, 0.6]
        agent.move_end_effector(
            target,
            end_effector_link=MOBILE_MANIP_EE_LINK,
        )

        dt = sim_core._dt
        max_steps = int(3.0 / dt)
        for _ in range(max_steps):
            sim_core.tick()
            agent.update(dt)

        link_state = p.getLinkState(
            agent.body_id,
            MOBILE_MANIP_EE_LINK,
            computeForwardKinematics=1,
            physicsClientId=agent._pid,
        )
        actual_pos = np.array(link_state[0])
        distance = np.linalg.norm(actual_pos - np.array(target))
        assert distance < EE_POS_TOL, f"EE at {actual_pos}, target {target}, distance {distance:.4f}"


# ============================================================
# IKParams.ik_joint_names — explicit IK joint selection
# ============================================================

MOBILE_MANIP_ARM_JOINTS = (
    "mount_to_shoulder",
    "shoulder_to_elbow",
    "elbow_to_wrist",
    "wrist_to_end",
)


@pytest.fixture
def mobile_manip_agent_explicit(pybullet_env):
    """Mobile manipulator with explicit ik_joint_names (arm joints only)."""
    from pybullet_fleet.agent import IKParams

    sim_core = MockSimCore(physics=False)
    sim_core._client = pybullet_env
    cfg = IKParams(ik_joint_names=MOBILE_MANIP_ARM_JOINTS)
    agent = Agent.from_urdf(
        urdf_path=MOBILE_MANIP_URDF,
        pose=Pose.from_xyz(0, 0, 0.3),
        use_fixed_base=False,
        mass=0.0,
        sim_core=sim_core,
        ik_params=cfg,
    )
    return agent, sim_core


class TestIKJointNames:
    """Tests for IKParams.ik_joint_names explicit joint selection.

    Only tests behaviour unique to explicit ik_joint_names mode.
    Common IK behaviour (length, reachability, fixed-joint mapping) is
    already covered by TestMobileManipulatorIK with auto mode.
    """

    def test_ik_joint_names_stored_on_params(self):
        """ik_joint_names is stored as a tuple on IKParams."""
        from pybullet_fleet.agent import IKParams

        cfg = IKParams(ik_joint_names=("joint_a", "joint_b"))
        assert cfg.ik_joint_names == ("joint_a", "joint_b")

    def test_wheels_locked_at_nonzero_with_explicit_names(self, mobile_manip_agent_explicit):
        """Wheels not in ik_joint_names stay at non-zero saved position.

        Verifies the explicit-names lock path (as opposed to auto
        continuous-lock tested in TestMobileManipulatorIK).
        """
        agent, _ = mobile_manip_agent_explicit
        wheel_pos = 1.5
        for idx in (0, 1):
            agent._kinematic_joint_positions[idx] = wheel_pos
            p.resetJointState(agent.body_id, idx, wheel_pos, physicsClientId=agent._pid)

        target = [0.5, 0.0, 0.6]
        angles = agent._solve_ik(target, ee_link_index=MOBILE_MANIP_EE_LINK)
        assert abs(angles[0] - wheel_pos) < 1e-3, f"Left wheel should stay at {wheel_pos}, got {angles[0]}"
        assert abs(angles[1] - wheel_pos) < 1e-3, f"Right wheel should stay at {wheel_pos}, got {angles[1]}"

    def test_arm_robot_defaults_work_unchanged(self, arm_agent):
        """arm_robot with ik_joint_names=None still converges (no regression)."""
        agent, _ = arm_agent
        target = [0.0, 0.3, 0.3]
        angles = agent._solve_ik(target)
        reachable = agent._check_ik_reachability(angles, target, agent._get_end_effector_link_index(), IK_REACH_TOL)
        assert reachable

    def test_move_end_effector_with_explicit_names(self, mobile_manip_agent_explicit):
        """move_end_effector uses ik_joint_names and returns reachable."""
        agent, _ = mobile_manip_agent_explicit
        target = [0.5, 0.0, 0.6]
        result = agent.move_end_effector(target, end_effector_link=MOBILE_MANIP_EE_LINK)
        assert result is True


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
