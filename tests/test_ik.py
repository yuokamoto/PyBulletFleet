"""Tests for IK (Inverse Kinematics) functionality.

Covers:
- Agent._solve_ik() — IK solver wrapper
- Agent._get_end_effector_link_index() — EE link auto-detection
- Agent.move_end_effector() — direct EE position command
- PoseAction — action-based EE control
- PickAction / DropAction EE pose extensions
"""

import numpy as np
import pybullet as p
import pybullet_data
import pytest

from pybullet_fleet.agent import Agent
from pybullet_fleet.action import PoseAction, PickAction, DropAction
from pybullet_fleet.geometry import Pose
from pybullet_fleet.sim_object import SimObject, ShapeParams
from pybullet_fleet.types import ActionStatus

ARM_URDF = "robots/arm_robot.urdf"


class MockSimCore:
    """Minimal sim_core mock for IK tests."""

    def __init__(self, dt=1.0 / 240.0, physics=False):
        self.sim_time = 0.0
        self._dt = dt
        self.sim_objects = []
        self._next_object_id = 0
        self._kinematic_objects = set()
        self._client = 0
        self._params = type("Params", (), {"physics": physics})()

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

    def tick(self, n=1):
        self.sim_time += self._dt * n


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


@pytest.fixture
def physics_arm_agent(pybullet_env):
    """Physics arm agent (mass>0, physics=True)."""
    sim_core = MockSimCore(physics=True)
    sim_core._client = pybullet_env
    agent = Agent.from_urdf(
        urdf_path=ARM_URDF,
        pose=Pose.from_xyz(0, 0, 0),
        use_fixed_base=True,
        mass=1.0,
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
        """Explicit int is returned as-is."""
        agent, _ = arm_agent
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

    def test_returns_joint_angles_list(self, arm_agent):
        """_solve_ik returns a list with one angle per joint."""
        agent, _ = arm_agent
        ee_idx = agent._get_end_effector_link_index()
        # Target: EE home position (always reachable)
        angles = agent._solve_ik([0.0, 0.0, 0.75], ee_link_index=ee_idx)
        assert isinstance(angles, list)
        assert len(angles) == agent.get_num_joints()

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
        """_solve_ik with target_orientation returns valid joint angles."""
        agent, _ = arm_agent
        ee_idx = agent._get_end_effector_link_index()
        target_pos = [0.0, 0.0, 0.75]
        target_orn = [0.0, 0.0, 0.0, 1.0]  # identity quaternion
        angles = agent._solve_ik(target_pos, target_orientation=target_orn, ee_link_index=ee_idx)
        assert isinstance(angles, list)
        assert len(angles) == agent.get_num_joints()

    def test_ik_with_explicit_ee_index(self, arm_agent):
        """_solve_ik with explicit ee_link_index parameter."""
        agent, _ = arm_agent
        angles = agent._solve_ik([0.0, 0.0, 0.75], ee_link_index=3)
        assert len(angles) == agent.get_num_joints()

    def test_ik_with_ee_link_name_via_resolve(self, arm_agent):
        """_solve_ik with link name resolved to index first."""
        agent, _ = arm_agent
        ee_idx = agent._get_end_effector_link_index("end_effector")
        angles = agent._solve_ik([0.0, 0.0, 0.75], ee_link_index=ee_idx)
        assert len(angles) == agent.get_num_joints()


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
        assert len(agent._kinematic_joint_targets) > 0

    def test_sets_joint_targets(self, arm_agent):
        """move_end_effector should set kinematic joint targets."""
        agent, _ = arm_agent
        agent.move_end_effector([0.0, 0.0, 0.75])
        # In kinematic mode, targets should be populated
        assert len(agent._kinematic_joint_targets) > 0

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
        """move_end_effector with orientation returns bool."""
        agent, _ = arm_agent
        result = agent.move_end_effector([0.0, 0.0, 0.75], target_orientation=[0, 0, 0, 1])
        assert isinstance(result, bool)
        assert len(agent._kinematic_joint_targets) > 0

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
# T3: PoseAction
# ============================================================


class TestPoseAction:
    """Tests for PoseAction."""

    def test_pose_action_completes(self, arm_agent):
        """PoseAction should reach COMPLETED status."""
        agent, sim_core = arm_agent
        action = PoseAction(target_position=[0.0, 0.0, 0.75])
        agent.add_action(action)

        dt = sim_core._dt
        for _ in range(3000):
            sim_core.tick()
            agent.update(dt)
            if action.status == ActionStatus.COMPLETED:
                break

        assert action.status == ActionStatus.COMPLETED

    def test_pose_action_ee_at_target(self, arm_agent):
        """After PoseAction completes, EE should be near target."""
        agent, sim_core = arm_agent
        target = [0.0, 0.0, 0.75]  # Home position
        action = PoseAction(target_position=target, tolerance=0.03)
        agent.add_action(action)

        dt = sim_core._dt
        for _ in range(3000):
            sim_core.tick()
            agent.update(dt)
            if action.status == ActionStatus.COMPLETED:
                break

        ee_idx = agent._get_end_effector_link_index()
        link_state = p.getLinkState(agent.body_id, ee_idx, computeForwardKinematics=1, physicsClientId=agent._pid)
        actual_pos = np.array(link_state[0])
        distance = np.linalg.norm(actual_pos - np.array(target))
        assert distance < 0.05

    def test_pose_action_with_orientation(self, arm_agent):
        """PoseAction with orientation completes."""
        agent, sim_core = arm_agent
        action = PoseAction(
            target_position=[0.0, 0.0, 0.75],
            target_orientation=[0.0, 0.0, 0.0, 1.0],
        )
        agent.add_action(action)

        dt = sim_core._dt
        for _ in range(3000):
            sim_core.tick()
            agent.update(dt)
            if action.status == ActionStatus.COMPLETED:
                break

        assert action.status == ActionStatus.COMPLETED

    def test_pose_action_explicit_ee_link(self, arm_agent):
        """PoseAction with explicit end_effector_link."""
        agent, sim_core = arm_agent
        action = PoseAction(
            target_position=[0.0, 0.0, 0.75],
            end_effector_link="end_effector",
        )
        agent.add_action(action)

        dt = sim_core._dt
        for _ in range(3000):
            sim_core.tick()
            agent.update(dt)
            if action.status == ActionStatus.COMPLETED:
                break

        assert action.status == ActionStatus.COMPLETED


# ============================================================
# T4: PickAction / DropAction EE pose extensions
# ============================================================


class TestPickDropEEPose:
    """Tests for ee_target_position on PickAction / DropAction."""

    def test_pick_action_mutual_exclusion(self):
        """ValueError when both joint_targets and ee_target_position are set."""
        with pytest.raises(ValueError, match="Cannot specify both"):
            PickAction(
                target_object_id=999,
                joint_targets=[0.0, 0.0, 0.0, 0.0],
                ee_target_position=[0.1, 0.0, 0.5],
            )

    def test_drop_action_mutual_exclusion(self):
        """ValueError when both joint_targets and ee_target_position are set."""
        with pytest.raises(ValueError, match="Cannot specify both"):
            DropAction(
                drop_pose=Pose.from_xyz(1, 0, 0),
                joint_targets=[0.0, 0.0, 0.0, 0.0],
                ee_target_position=[0.1, 0.0, 0.5],
            )

    def test_pick_action_ee_pose_accepted(self):
        """PickAction with only ee_target_position should not raise."""
        action = PickAction(
            target_object_id=999,
            ee_target_position=[0.1, 0.0, 0.5],
        )
        assert action.ee_target_position == [0.1, 0.0, 0.5]
        assert action.joint_targets is None

    def test_drop_action_ee_pose_accepted(self):
        """DropAction with only ee_target_position should not raise."""
        action = DropAction(
            drop_pose=Pose.from_xyz(1, 0, 0),
            ee_target_position=[0.1, 0.0, 0.5],
        )
        assert action.ee_target_position == [0.1, 0.0, 0.5]
        assert action.joint_targets is None

    def test_pick_with_ee_pose_resolves_ik(self, arm_agent):
        """PickAction with ee_target_position should resolve IK during PICKING phase."""
        agent, sim_core = arm_agent

        # Create a pickable box near arm
        box = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="box", half_extents=[0.03, 0.03, 0.03]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.03, 0.03, 0.03]),
            pose=Pose.from_xyz(0.15, 0.0, 0.3),
            mass=0.0,
            sim_core=sim_core,
        )

        action = PickAction(
            target_object_id=box.body_id,
            use_approach=False,
            ee_target_position=[0.15, 0.0, 0.3],
            attach_link=3,  # end effector
            attach_relative_pose=Pose.from_xyz(0, 0, 0.06),
        )
        agent.add_action(action)

        dt = sim_core._dt
        for _ in range(3000):
            sim_core.tick()
            agent.update(dt)
            if action.status in (ActionStatus.COMPLETED, ActionStatus.FAILED):
                break

        # The action should have resolved joint_targets from IK
        assert action.joint_targets is not None
        assert isinstance(action.joint_targets, list)


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
