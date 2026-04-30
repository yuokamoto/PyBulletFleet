"""
Tests for Action classes.

This module tests:
- Action base class: status lifecycle, cancel, reset, duration, logging
- WaitAction: time-based waiting
- MoveAction: creation and direction normalization
- JointAction: creation
- PickAction: creation and defaults
- DropAction: creation with Pose
"""

import pytest

from pybullet_fleet.action import (
    WaitAction,
    MoveAction,
    JointAction,
    PoseAction,
    PickAction,
    DropAction,
    DEFAULT_MAX_FORCE,
    DEFAULT_JOINT_TOLERANCE,
    DEFAULT_EE_TOLERANCE,
    DEFAULT_MOVE_SKIP_THRESHOLD,
)
from pybullet_fleet.geometry import Pose, Path
from pybullet_fleet.types import ActionStatus, MovementDirection


# ---------------------------------------------------------------------------
# Module-level constants
# ---------------------------------------------------------------------------


class TestActionConstants:
    """Verify module-level default constants."""

    def test_constant_values(self):
        assert DEFAULT_MAX_FORCE == 500.0
        assert DEFAULT_JOINT_TOLERANCE == 0.01
        assert DEFAULT_EE_TOLERANCE == 0.02
        assert DEFAULT_MOVE_SKIP_THRESHOLD == 0.05


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


class _MockSimCore:
    """Minimal sim_core stub."""

    sim_time: float = 0.0


class _MockAgent:
    """Minimal agent stub for actions that don't need PyBullet."""

    def __init__(self, *, with_sim_core: bool = False):
        self.sim_core = _MockSimCore() if with_sim_core else None


@pytest.fixture
def agent():
    """Agent stub without sim_core (sim_time defaults to 0.0)."""
    return _MockAgent()


@pytest.fixture
def agent_with_time():
    """Agent stub with sim_core so start/end times are recorded."""
    return _MockAgent(with_sim_core=True)


# ---------------------------------------------------------------------------
# Action base class
# ---------------------------------------------------------------------------


class TestActionBaseClass:
    """Test Action base-class properties and methods."""

    def test_initial_status(self):
        action = WaitAction(duration=1.0)
        assert action.status is ActionStatus.NOT_STARTED
        assert action.start_time is None
        assert action.end_time is None
        assert action.error_message is None

    def test_is_complete_returns_false_initially(self):
        action = WaitAction(duration=1.0)
        assert action.is_complete() is False

    def test_get_duration_returns_none_before_completion(self):
        action = WaitAction(duration=1.0)
        assert action.get_duration() is None

    def test_get_duration_returns_value_after_completion(self, agent_with_time):
        agent_with_time.sim_core.sim_time = 1.0
        action = WaitAction(duration=0.0)
        # First call starts (sim_time=1.0), second completes
        agent_with_time.sim_core.sim_time = 1.0
        action.execute(agent_with_time, 0.01)  # starts + completes in one step
        assert action.get_duration() is not None
        assert action.get_duration() == 0.0  # same sim_time for start and end

    def test_cancel_not_started(self):
        action = WaitAction(duration=1.0)
        action.cancel()
        assert action.status is ActionStatus.CANCELLED
        assert action.is_complete() is True
        # end_time stays None because action was never started
        assert action.end_time is None

    def test_cancel_in_progress(self, agent):
        action = WaitAction(duration=10.0)
        action.execute(agent, 0.01)  # starts the action
        assert action.status is ActionStatus.IN_PROGRESS

        action.cancel()
        assert action.status is ActionStatus.CANCELLED
        assert action.is_complete() is True
        # end_time set to start_time as placeholder
        assert action.end_time == action.start_time

    def test_reset_clears_state(self, agent):
        action = WaitAction(duration=0.01)
        action.execute(agent, 0.01)  # complete
        assert action.is_complete() is True

        action.reset()
        assert action.status is ActionStatus.NOT_STARTED
        assert action.start_time is None
        assert action.end_time is None
        assert action.error_message is None
        assert action.is_complete() is False

    def test_reset_allows_re_execution(self, agent):
        action = WaitAction(duration=0.01)
        action.execute(agent, 0.01)
        assert action.is_complete() is True

        action.reset()
        completed = action.execute(agent, 0.01)
        assert completed is True
        assert action.status is ActionStatus.COMPLETED


# ---------------------------------------------------------------------------
# WaitAction
# ---------------------------------------------------------------------------


class TestWaitAction:
    """Test WaitAction for time-based waiting."""

    def test_creation_defaults(self):
        action = WaitAction(duration=2.5)
        assert action.duration == 2.5
        assert action.action_type == "idle"

    def test_creation_with_action_type(self):
        action = WaitAction(duration=1.0, action_type="charge")
        assert action.action_type == "charge"

    def test_zero_duration_completes_immediately(self, agent):
        action = WaitAction(duration=0.0)
        completed = action.execute(agent, 0.01)
        assert completed is True
        assert action.status is ActionStatus.COMPLETED

    def test_completes_after_elapsed_time(self, agent):
        action = WaitAction(duration=0.05)
        dt = 0.01

        for _ in range(4):
            assert action.execute(agent, dt) is False
            assert action.status is ActionStatus.IN_PROGRESS

        # 5th step: 0.05s elapsed
        assert action.execute(agent, dt) is True
        assert action.status is ActionStatus.COMPLETED

    def test_does_not_complete_early(self, agent):
        action = WaitAction(duration=1.0)
        # Execute for much less than duration
        for _ in range(10):
            result = action.execute(agent, 0.01)
        assert result is False
        assert action.status is ActionStatus.IN_PROGRESS

    def test_reset_clears_elapsed_time(self, agent):
        action = WaitAction(duration=0.03)
        # Almost done
        action.execute(agent, 0.02)
        assert action.status is ActionStatus.IN_PROGRESS

        action.reset()
        # After reset, needs full duration again
        assert action.execute(agent, 0.02) is False
        assert action.execute(agent, 0.02) is True

    def test_sequential_execution(self, agent):
        """Execute two WaitActions one after the other."""
        a1 = WaitAction(duration=0.02)
        a2 = WaitAction(duration=0.02)

        dt = 0.01
        while not a1.is_complete():
            a1.execute(agent, dt)
        assert a1.status is ActionStatus.COMPLETED

        while not a2.is_complete():
            a2.execute(agent, dt)
        assert a2.status is ActionStatus.COMPLETED


# ---------------------------------------------------------------------------
# MoveAction (creation only — execution needs full agent)
# ---------------------------------------------------------------------------


class TestMoveAction:
    """Test MoveAction instantiation and direction normalization."""

    def test_creation_defaults(self):
        path = Path.from_positions([[0, 0, 0], [1, 0, 0]])
        action = MoveAction(path=path)
        assert action.path is path
        assert action.auto_approach is True
        assert action.final_orientation_align is True
        assert action.direction is MovementDirection.FORWARD
        assert action.status is ActionStatus.NOT_STARTED

    def test_direction_string_normalized_to_enum(self):
        path = Path.from_positions([[0, 0, 0]])
        action = MoveAction(path=path, direction="backward")
        assert action.direction is MovementDirection.BACKWARD

    def test_direction_enum_preserved(self):
        path = Path.from_positions([[0, 0, 0]])
        action = MoveAction(path=path, direction=MovementDirection.BACKWARD)
        assert action.direction is MovementDirection.BACKWARD

    def test_invalid_direction_string_raises(self):
        path = Path.from_positions([[0, 0, 0]])
        with pytest.raises(ValueError):
            MoveAction(path=path, direction="sideways")

    def test_reset(self):
        path = Path.from_positions([[0, 0, 0]])
        action = MoveAction(path=path)
        action.status = ActionStatus.COMPLETED  # simulate completion
        action.reset()
        assert action.status is ActionStatus.NOT_STARTED


# ---------------------------------------------------------------------------
# JointAction (creation only)
# ---------------------------------------------------------------------------


class TestJointAction:
    """Test JointAction instantiation."""

    def test_creation_with_list(self):
        action = JointAction(target_joint_positions=[0.0, 1.0, 0.5])
        assert action.target_joint_positions == [0.0, 1.0, 0.5]
        assert action.max_force == DEFAULT_MAX_FORCE
        assert action.tolerance is None  # Falls back to agent.joint_tolerance at execute time

    def test_creation_with_dict(self):
        targets = {"joint_1": 0.5, "joint_2": 1.0}
        action = JointAction(target_joint_positions=targets)
        assert action.target_joint_positions == targets

    def test_custom_force_and_tolerance(self):
        action = JointAction(
            target_joint_positions=[0.0],
            max_force=100.0,
            tolerance=0.05,
        )
        assert action.max_force == 100.0
        assert action.tolerance == 0.05


# ---------------------------------------------------------------------------
# PoseAction (creation only)
# ---------------------------------------------------------------------------


class TestPoseAction:
    """Test PoseAction instantiation and defaults."""

    def test_creation_with_position_only(self):
        action = PoseAction(target_position=[0.3, 0.0, 0.5])
        assert action.target_position == [0.3, 0.0, 0.5]
        assert action.target_orientation is None
        assert action.end_effector_link is None
        assert action.max_force == DEFAULT_MAX_FORCE
        assert action.tolerance == DEFAULT_EE_TOLERANCE
        assert action.status is ActionStatus.NOT_STARTED

    def test_creation_with_orientation(self):
        action = PoseAction(
            target_position=[0.1, 0.2, 0.3],
            target_orientation=(0.0, 0.0, 0.0, 1.0),
        )
        assert action.target_orientation == (0.0, 0.0, 0.0, 1.0)

    def test_creation_with_explicit_ee_link(self):
        action = PoseAction(target_position=[0.0, 0.0, 0.5], end_effector_link=3)
        assert action.end_effector_link == 3

    def test_creation_with_ee_link_name(self):
        action = PoseAction(target_position=[0.0, 0.0, 0.5], end_effector_link="end_effector")
        assert action.end_effector_link == "end_effector"

    def test_custom_force_and_tolerance(self):
        action = PoseAction(
            target_position=[0.0, 0.0, 0.5],
            max_force=200.0,
            tolerance=0.05,
        )
        assert action.max_force == 200.0
        assert action.tolerance == 0.05

    def test_reset(self):
        action = PoseAction(target_position=[0.0, 0.0, 0.5])
        action.status = ActionStatus.COMPLETED
        action._reachable = True
        action.reset()
        assert action.status is ActionStatus.NOT_STARTED
        assert action._reachable is False


# ---------------------------------------------------------------------------
# PickAction (creation only)
# ---------------------------------------------------------------------------


class TestPickAction:
    """Test PickAction instantiation and defaults."""

    def test_creation_with_target_id(self):
        action = PickAction(target_object_id=42)
        assert action.target_object_id == 42
        assert action.target_position is None
        assert action.use_approach is True
        assert action.approach_offset == 1.0
        assert action.pick_offset == 0.0
        assert action.attach_link == -1
        assert action.status is ActionStatus.NOT_STARTED

    def test_creation_with_target_position(self):
        action = PickAction(target_position=[5.0, 0.0, 0.1], search_radius=1.0)
        assert action.target_position == [5.0, 0.0, 0.1]
        assert action.search_radius == 1.0

    def test_creation_with_link_name(self):
        action = PickAction(target_object_id=1, attach_link="sensor_mast")
        assert action.attach_link == "sensor_mast"
        # _attach_link_index resolved to -1 placeholder for string links
        assert action._attach_link_index == -1

    def test_creation_with_link_index(self):
        action = PickAction(target_object_id=1, attach_link=3)
        assert action._attach_link_index == 3

    def test_default_attach_relative_pose(self):
        action = PickAction(target_object_id=1)
        assert action.attach_relative_pose.position == [0.0, 0.0, 0.0]
        assert action.attach_relative_pose.orientation == [0.0, 0.0, 0.0, 1.0]

    def test_custom_attach_relative_pose(self):
        pose = Pose.from_euler(0.6, 0, -0.2, roll=1.57, pitch=0, yaw=0)
        action = PickAction(target_object_id=1, attach_relative_pose=pose)
        assert action.attach_relative_pose is pose

    def test_reset(self):
        action = PickAction(target_object_id=1)
        action.status = ActionStatus.COMPLETED
        action._target_object = "something"  # type: ignore[assignment]
        action.reset()
        assert action.status is ActionStatus.NOT_STARTED
        assert action._target_object is None
        assert action._phase.value == "init"

    def test_ee_target_position_accepted(self):
        action = PickAction(target_object_id=999, ee_target_position=[0.1, 0.0, 0.5])
        assert action.ee_target_position == [0.1, 0.0, 0.5]
        assert action.joint_targets is None

    def test_joint_targets_accepted(self):
        action = PickAction(target_object_id=999, joint_targets=[0.0, 1.0, -0.5, 0.2])
        assert action.joint_targets == [0.0, 1.0, -0.5, 0.2]
        assert action.ee_target_position is None

    def test_joint_targets_and_ee_mutual_exclusion(self):
        with pytest.raises(ValueError, match="Cannot specify both"):
            PickAction(
                target_object_id=999,
                joint_targets=[0.0, 0.0, 0.0, 0.0],
                ee_target_position=[0.1, 0.0, 0.5],
            )

    def test_continue_on_ik_failure_default_true(self):
        action = PickAction(target_object_id=999, ee_target_position=[0.1, 0.0, 0.5])
        assert action.continue_on_ik_failure is True

    def test_continue_on_ik_failure_explicit(self):
        action = PickAction(
            target_object_id=999,
            ee_target_position=[0.1, 0.0, 0.5],
            continue_on_ik_failure=False,
        )
        assert action.continue_on_ik_failure is False

    def test_ee_tolerance_default(self):
        action = PickAction(target_object_id=999, ee_target_position=[0.1, 0.0, 0.5])
        assert action.ee_tolerance == DEFAULT_EE_TOLERANCE

    def test_ee_tolerance_custom(self):
        action = PickAction(
            target_object_id=999,
            ee_target_position=[0.1, 0.0, 0.5],
            ee_tolerance=0.05,
        )
        assert action.ee_tolerance == 0.05

    def test_ee_tolerance_independent_of_joint_tolerance(self):
        """ee_tolerance must not be derived from joint_tolerance."""
        action = PickAction(
            target_object_id=999,
            ee_target_position=[0.1, 0.0, 0.5],
            joint_tolerance=0.1,
            ee_tolerance=0.03,
        )
        assert action.ee_tolerance == 0.03
        assert action.joint_tolerance == 0.1

    def test_move_skip_threshold_default(self):
        action = PickAction(target_object_id=1)
        assert action.move_skip_threshold == DEFAULT_MOVE_SKIP_THRESHOLD

    def test_move_skip_threshold_custom(self):
        action = PickAction(target_object_id=1, move_skip_threshold=0.1)
        assert action.move_skip_threshold == 0.1


# ---------------------------------------------------------------------------
# DropAction (creation only)
# ---------------------------------------------------------------------------


class TestDropAction:
    """Test DropAction instantiation with Pose-based drop_pose."""

    def test_creation_with_position_only(self):
        pose = Pose(position=[10, 5, 0.1])
        action = DropAction(drop_pose=pose)
        assert action.drop_pose.position == [10, 5, 0.1]
        assert action.drop_pose.orientation == [0.0, 0.0, 0.0, 1.0]
        assert action.use_approach is True
        assert action.place_gently is True

    def test_creation_with_orientation(self):
        pose = Pose(position=[1, 2, 0], orientation=[0, 0, 0.7, 0.7])
        action = DropAction(drop_pose=pose)
        assert action.drop_pose.orientation == [0, 0, 0.7, 0.7]

    def test_creation_no_approach(self):
        action = DropAction(
            drop_pose=Pose(position=[0, 0, 0]),
            use_approach=False,
            drop_offset=0.5,
        )
        assert action.use_approach is False
        assert action.drop_offset == 0.5

    def test_drop_from_height(self):
        action = DropAction(
            drop_pose=Pose(position=[0, 0, 0]),
            place_gently=False,
            drop_height=0.3,
        )
        assert action.place_gently is False
        assert action.drop_height == 0.3

    def test_target_object_id(self):
        action = DropAction(
            drop_pose=Pose(position=[0, 0, 0]),
            target_object_id=99,
        )
        assert action.target_object_id == 99

    def test_default_target_object_is_none(self):
        action = DropAction(drop_pose=Pose(position=[0, 0, 0]))
        assert action.target_object_id is None

    def test_reset(self):
        action = DropAction(drop_pose=Pose(position=[1, 2, 3]))
        action.status = ActionStatus.COMPLETED
        action._target_object = "something"  # type: ignore[assignment]
        action.reset()
        assert action.status is ActionStatus.NOT_STARTED
        assert action._target_object is None
        assert action._phase.value == "init"

    def test_ee_target_position_accepted(self):
        action = DropAction(drop_pose=Pose.from_xyz(1, 0, 0), ee_target_position=[0.1, 0.0, 0.5])
        assert action.ee_target_position == [0.1, 0.0, 0.5]
        assert action.joint_targets is None

    def test_joint_targets_accepted(self):
        action = DropAction(drop_pose=Pose.from_xyz(1, 0, 0), joint_targets=[0.0, 1.0, -0.5, 0.2])
        assert action.joint_targets == [0.0, 1.0, -0.5, 0.2]
        assert action.ee_target_position is None

    def test_joint_targets_and_ee_mutual_exclusion(self):
        with pytest.raises(ValueError, match="Cannot specify both"):
            DropAction(
                drop_pose=Pose.from_xyz(1, 0, 0),
                joint_targets=[0.0, 0.0, 0.0, 0.0],
                ee_target_position=[0.1, 0.0, 0.5],
            )

    def test_continue_on_ik_failure_default_true(self):
        action = DropAction(drop_pose=Pose.from_xyz(1, 0, 0), ee_target_position=[0.1, 0.0, 0.5])
        assert action.continue_on_ik_failure is True

    def test_continue_on_ik_failure_explicit(self):
        action = DropAction(
            drop_pose=Pose.from_xyz(1, 0, 0),
            ee_target_position=[0.1, 0.0, 0.5],
            continue_on_ik_failure=False,
        )
        assert action.continue_on_ik_failure is False

    def test_ee_tolerance_default(self):
        action = DropAction(drop_pose=Pose.from_xyz(1, 0, 0), ee_target_position=[0.1, 0.0, 0.5])
        assert action.ee_tolerance == DEFAULT_EE_TOLERANCE

    def test_ee_tolerance_custom(self):
        action = DropAction(
            drop_pose=Pose.from_xyz(1, 0, 0),
            ee_target_position=[0.1, 0.0, 0.5],
            ee_tolerance=0.05,
        )
        assert action.ee_tolerance == 0.05

    def test_ee_tolerance_independent_of_joint_tolerance(self):
        """ee_tolerance must not be derived from joint_tolerance."""
        action = DropAction(
            drop_pose=Pose.from_xyz(1, 0, 0),
            ee_target_position=[0.1, 0.0, 0.5],
            joint_tolerance=0.1,
            ee_tolerance=0.03,
        )
        assert action.ee_tolerance == 0.03
        assert action.joint_tolerance == 0.1

    def test_move_skip_threshold_default(self):
        action = DropAction(drop_pose=Pose.from_xyz(1, 0, 0))
        assert action.move_skip_threshold == DEFAULT_MOVE_SKIP_THRESHOLD

    def test_move_skip_threshold_custom(self):
        action = DropAction(drop_pose=Pose.from_xyz(1, 0, 0), move_skip_threshold=0.2)
        assert action.move_skip_threshold == 0.2


# ---------------------------------------------------------------------------
# Logging helpers (smoke tests)
# ---------------------------------------------------------------------------


class TestActionLogging:
    """Test that logging helpers don't crash and set expected state."""

    def test_log_start_without_agent(self):
        action = WaitAction(duration=1.0)
        # Should not raise
        action._log_start(agent=None)
        assert "[WaitAction]" in action._log.prefix

    def test_log_start_with_agent_no_log(self):
        """Agent without _log attribute falls back to action-only prefix."""
        action = WaitAction(duration=1.0)
        action._log_start(agent=_MockAgent())
        assert "[WaitAction]" in action._log.prefix

    def test_log_failure_sets_error_message(self):
        action = WaitAction(duration=1.0)
        action._log_failure("something went wrong")
        assert action.error_message == "something went wrong"

    def test_log_end_without_duration(self):
        action = WaitAction(duration=1.0)
        # No start/end times set — should not raise
        action._log_end()

    def test_log_end_with_duration(self, agent):
        action = WaitAction(duration=0.0)
        action.execute(agent, 0.01)
        # Already completed, calling _log_end again should not raise
        action._log_end()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
