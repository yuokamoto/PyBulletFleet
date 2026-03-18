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
import pybullet_data
import pytest

from pybullet_fleet.agent import Agent
from pybullet_fleet.geometry import Pose
from pybullet_fleet.sim_object import ShapeParams
from tests.conftest import MockSimCore

ARM_URDF = "robots/arm_robot.urdf"


@pytest.fixture
def pybullet_env():
    """PyBullet DIRECT environment with plane."""
    client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10, physicsClientId=client)
    p.loadURDF("plane.urdf", physicsClientId=client)
    yield client
    p.disconnect(client)


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
        assert distance < 0.05, f"EE at {actual_pos}, target {target_pos}, distance {distance:.4f}"

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
        assert np.linalg.norm(actual_pos - np.array(target_pos)) < 0.05
        actual_orn = np.array(link_state[1])
        dot = abs(np.dot(actual_orn, np.array(target_orn)))
        assert dot > 0.95, f"EE orientation mismatch: dot={dot:.3f}, actual={actual_orn}"

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
        reachable = agent._check_ik_reachability(angles, target, ee_idx, tolerance=0.02)
        assert reachable, f"IK solution {angles} does not place EE within 0.02m of target {target}"


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
        assert distance < 0.05, f"EE at {actual_pos}, target {target}, distance {distance:.4f}"

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
        assert np.linalg.norm(actual_pos - np.array(target_pos)) < 0.05

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
        assert agent.are_joints_at_targets(tolerance=0.05)
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
        assert cfg.seed_quartiles == (0.25, 0.75)

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


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
