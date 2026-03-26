"""Tests for Controller hierarchy, KinematicController, and Agent.set_controller().

TDD tests for the Controller system (Plugin Architecture Phase 2).
OmniController is a kinematic omnidirectional 6-DoF velocity controller:
body-frame (vx, vy, vz, wx, wy, wz) → quaternion-based world-frame integration.
"""

import math

import numpy as np
import pytest

from pybullet_fleet import Agent, AgentSpawnParams, Pose
from pybullet_fleet.controller import (
    Controller,
    KinematicController,
    CONTROLLER_REGISTRY,
    OmniController,
    DifferentialController,
    create_controller,
    register_controller,
)
from pybullet_fleet.types import MotionMode
from tests.conftest import MockSimCore


@pytest.fixture
def sim_core(pybullet_env):
    """MockSimCore wired to the headless pybullet_env fixture."""
    sc = MockSimCore(physics=False)
    sc._client = pybullet_env
    return sc


@pytest.fixture
def mobile_agent(sim_core):
    """Spawn a simple omnidirectional mobile agent."""
    params = AgentSpawnParams(
        urdf_path="robots/mobile_robot.urdf",
        initial_pose=Pose.from_xyz(0, 0, 0.05),
        motion_mode=MotionMode.OMNIDIRECTIONAL,
        max_linear_vel=2.0,
    )
    return Agent.from_params(params, sim_core)


@pytest.fixture
def omni_agent(mobile_agent):
    """mobile_agent with OmniController pre-attached."""
    ctrl = OmniController()
    mobile_agent.set_controller(ctrl)
    return mobile_agent


@pytest.fixture
def diff_agent(mobile_agent):
    """mobile_agent with DifferentialController pre-attached."""
    ctrl = DifferentialController()
    mobile_agent.set_controller(ctrl)
    return mobile_agent


# -----------------------------------------------------------------------
# OmniController (unified: basic + body-frame + 6-DoF)
# -----------------------------------------------------------------------


class TestOmniController:
    """OmniController: kinematic omnidirectional 6-DoF velocity controller.

    Body-frame (vx, vy, vz, wx, wy, wz) → quaternion-based world-frame integration.
    All assertions verify exact target positions, not just "moved from initial."
    """

    # -- Linear motion (exact target verification) --

    def test_vx_reaches_target(self, omni_agent):
        """vx=1.0 for dt=0.1 → x = initial + 0.1 (exact kinematic integration)."""
        omni_agent._controller.set_velocity(vx=1.0)

        initial_x = omni_agent.get_pose().x
        dt = 0.1
        omni_agent.update(dt)

        assert omni_agent.get_pose().x == pytest.approx(initial_x + 1.0 * dt, abs=0.01)

    def test_vy_reaches_target(self, omni_agent):
        """vy=1.0 at yaw=0 for dt=0.1 → y = initial + 0.1."""
        omni_agent._controller.set_velocity(vy=1.0)

        initial_y = omni_agent.get_pose().y
        dt = 0.1
        omni_agent.update(dt)

        assert omni_agent.get_pose().y == pytest.approx(initial_y + 1.0 * dt, abs=0.01)

    def test_vz_reaches_target(self, omni_agent):
        """vz=0.5 for dt=0.1 → z = initial + 0.05."""
        omni_agent._controller.set_velocity(vz=0.5)

        initial_z = omni_agent.get_pose().z
        dt = 0.1
        omni_agent.update(dt)

        assert omni_agent.get_pose().z == pytest.approx(initial_z + 0.5 * dt, abs=0.01)

    # -- Angular motion (exact target verification) --

    def test_wz_reaches_target_yaw(self, omni_agent):
        """wz=1.0 for dt=0.1 → yaw ≈ initial + 0.1 rad, roll/pitch unchanged."""
        omni_agent._controller.set_velocity(wz=1.0)

        initial_yaw = omni_agent.get_pose().yaw
        dt = 0.1
        omni_agent.update(dt)

        pose = omni_agent.get_pose()
        assert pose.yaw == pytest.approx(initial_yaw + 1.0 * dt, abs=0.01)
        # Roll and pitch must remain ~0 (pure yaw rotation)
        assert abs(pose.roll) < 0.01
        assert abs(pose.pitch) < 0.01

    def test_wx_reaches_target_roll(self, omni_agent):
        """wx=1.0 for dt=0.1 → roll ≈ 0.1 rad."""
        omni_agent._controller.set_velocity(wx=1.0)

        dt = 0.1
        omni_agent.update(dt)

        assert omni_agent.get_pose().roll == pytest.approx(1.0 * dt, abs=0.01)

    def test_wy_reaches_target_pitch(self, omni_agent):
        """wy=1.0 for dt=0.1 → pitch ≈ 0.1 rad."""
        omni_agent._controller.set_velocity(wy=1.0)

        dt = 0.1
        omni_agent.update(dt)

        assert omni_agent.get_pose().pitch == pytest.approx(1.0 * dt, abs=0.01)

    # -- Body-frame rotation (quaternion body→world) --

    def test_vx_at_yaw_90_reaches_target_y(self, omni_agent):
        """vx=1 at yaw=π/2 → y += 0.1 (body forward = world +Y)."""
        omni_agent.set_pose(Pose.from_yaw(0, 0, 0.05, math.pi / 2))
        omni_agent._controller.set_velocity(vx=1.0)

        dt = 0.1
        omni_agent.update(dt)

        pose = omni_agent.get_pose()
        assert pose.y == pytest.approx(0.0 + 1.0 * dt, abs=0.02)
        assert abs(pose.x) < 0.02  # Negligible X movement

    def test_vy_at_yaw_90_reaches_target_neg_x(self, omni_agent):
        """vy=1 at yaw=π/2 → x ≈ -0.1 (body left = world -X)."""
        omni_agent.set_pose(Pose.from_yaw(0, 0, 0.05, math.pi / 2))
        omni_agent._controller.set_velocity(vy=1.0)

        dt = 0.1
        omni_agent.update(dt)

        pose = omni_agent.get_pose()
        assert pose.x == pytest.approx(-1.0 * dt, abs=0.02)

    def test_vx_at_pitch_90_reaches_target_neg_z(self, omni_agent):
        """vx=1 with pitch=π/2 → z -= 0.1 (body forward = world -Z, full quaternion rotation)."""
        omni_agent.set_pose(Pose.from_euler(0, 0, 0.5, roll=0, pitch=math.pi / 2, yaw=0))
        omni_agent._controller.set_velocity(vx=1.0)

        initial_z = omni_agent.get_pose().z
        dt = 0.1
        omni_agent.update(dt)

        pose = omni_agent.get_pose()
        assert pose.z == pytest.approx(initial_z - 1.0 * dt, abs=0.02)
        assert abs(pose.x) < 0.02  # Negligible X movement

    def test_vz_at_pitch_90_reaches_target_x(self, omni_agent):
        """vz=1 with pitch=π/2 → x += 0.1 (body up = world +X)."""
        omni_agent.set_pose(Pose.from_euler(0, 0, 0.5, roll=0, pitch=math.pi / 2, yaw=0))
        omni_agent._controller.set_velocity(vz=1.0)

        initial_x = omni_agent.get_pose().x
        dt = 0.1
        omni_agent.update(dt)

        pose = omni_agent.get_pose()
        assert pose.x == pytest.approx(initial_x + 1.0 * dt, abs=0.02)

    # -- Combined angular (quaternion integration) --

    def test_simultaneous_wx_wy_uses_quaternion(self, omni_agent):
        """Simultaneous wx + wy uses quaternion multiplication (no gimbal lock)."""
        omni_agent._controller.set_velocity(wx=1.0, wy=1.0)

        dt = 0.1
        omni_agent.update(dt)

        pose = omni_agent.get_pose()
        # Both roll and pitch should have changed (quaternion integration)
        assert abs(pose.roll) == pytest.approx(0.1, abs=0.02)
        assert abs(pose.pitch) == pytest.approx(0.1, abs=0.02)

    # -- Orientation preservation --

    def test_linear_only_preserves_orientation(self, omni_agent):
        """Linear velocity (wx=wy=wz=0) preserves current orientation exactly."""
        # Set a non-trivial orientation
        omni_agent.set_pose(Pose.from_euler(0, 0, 0.05, roll=0.3, pitch=0.2, yaw=0.5))
        initial_quat = list(omni_agent.get_pose().orientation)

        omni_agent._controller.set_velocity(vx=1.0, vy=0.5, vz=0.2)  # No angular
        omni_agent.update(0.1)

        new_quat = omni_agent.get_pose().orientation
        np.testing.assert_allclose(new_quat, initial_quat, atol=1e-6)

    # -- State management --

    def test_zero_velocity_returns_false(self, omni_agent):
        """compute() returns False when velocity is zero."""
        result = omni_agent._controller.compute(omni_agent, 0.1)
        assert result is False

    def test_nonzero_velocity_returns_true(self, omni_agent):
        """compute() returns True when velocity is nonzero."""
        omni_agent._controller.set_velocity(vx=1.0)
        result = omni_agent._controller.compute(omni_agent, 0.1)
        assert result is True

    def test_stop_clears_all_6dof(self, omni_agent):
        """agent.stop() zeros all 6 DoF velocities via on_stop()."""
        omni_agent._controller.set_velocity(vx=1.0, vy=0.5, vz=0.2, wx=0.1, wy=0.2, wz=0.3)

        omni_agent.stop()
        # All zeros → compute returns False
        result = omni_agent._controller.compute(omni_agent, 0.1)
        assert result is False

    def test_reported_velocity_matches_command(self, omni_agent):
        """After compute(), agent.velocity reflects the commanded world-frame velocity."""
        omni_agent._controller.set_velocity(vx=1.5, vy=-0.5, vz=0.0)

        omni_agent.update(0.1)

        assert omni_agent.velocity[0] == pytest.approx(1.5, abs=0.01)
        assert omni_agent.velocity[1] == pytest.approx(-0.5, abs=0.01)

    # -- Velocity limits (magnitude clamping) --

    def test_linear_speed_clamped(self, mobile_agent):
        """Linear velocity magnitude clamped to max_linear_vel, direction preserved."""
        ctrl = OmniController(max_linear_vel=1.0)
        mobile_agent.set_controller(ctrl)

        ctrl.set_velocity(vx=3.0, vy=4.0)  # magnitude=5.0, clamped to 1.0
        # Direction preserved: (3/5, 4/5) * 1.0
        np.testing.assert_allclose(ctrl._linear_velocity, [0.6, 0.8, 0.0], atol=1e-9)

    def test_angular_speed_clamped(self, mobile_agent):
        """Angular velocity magnitude clamped to max_angular_vel, direction preserved."""
        ctrl = OmniController(max_angular_vel=1.0)
        mobile_agent.set_controller(ctrl)

        ctrl.set_velocity(wx=0.0, wy=0.0, wz=2.0)  # magnitude=2.0, clamped to 1.0
        np.testing.assert_allclose(ctrl._angular_velocity, [0.0, 0.0, 1.0], atol=1e-9)

    def test_below_limit_not_clamped(self, mobile_agent):
        """Velocity below limit passes through unchanged."""
        ctrl = OmniController(max_linear_vel=10.0, max_angular_vel=5.0)
        mobile_agent.set_controller(ctrl)

        ctrl.set_velocity(vx=1.0, vy=2.0, wz=1.5)
        np.testing.assert_allclose(ctrl._linear_velocity, [1.0, 2.0, 0.0], atol=1e-9)
        np.testing.assert_allclose(ctrl._angular_velocity, [0.0, 0.0, 1.5], atol=1e-9)

    def test_default_no_limit(self, omni_agent):
        """Default max_linear_vel/max_angular_vel = inf (no clamping)."""
        omni_agent._controller.set_velocity(vx=100.0, vy=200.0, wz=50.0)
        np.testing.assert_allclose(omni_agent._controller._linear_velocity, [100.0, 200.0, 0.0], atol=1e-9)
        np.testing.assert_allclose(omni_agent._controller._angular_velocity, [0.0, 0.0, 50.0], atol=1e-9)

    def test_from_config_with_limits(self):
        """from_config parses max_linear_vel / max_angular_vel."""
        ctrl = OmniController.from_config({"max_linear_vel": 2.0, "max_angular_vel": 1.0})
        assert ctrl._max_linear_vel == pytest.approx(2.0)  # type: ignore[reportAttributeAccessIssue]
        assert ctrl._max_angular_vel == pytest.approx(1.0)  # type: ignore[reportAttributeAccessIssue]


# -----------------------------------------------------------------------
# Agent.set_controller()
# -----------------------------------------------------------------------


class TestSetController:
    """Agent.set_controller() switches control strategy."""

    def test_set_controller_stores_reference(self, mobile_agent):
        """set_controller(ctrl) stores the controller on the agent."""
        ctrl = OmniController()
        mobile_agent.set_controller(ctrl)
        assert mobile_agent._controller is ctrl

    def test_set_controller_none_disables_movement(self, mobile_agent):
        """set_controller(None) disables all movement."""
        ctrl = OmniController()
        mobile_agent.set_controller(ctrl)
        mobile_agent.set_controller(None)
        assert mobile_agent._controller is None

    def test_goal_based_works_via_controller(self, mobile_agent, sim_core):
        """Goal-based movement works through the auto-assigned controller."""
        mobile_agent.set_goal_pose(Pose.from_xyz(5, 0, 0.05))
        initial_x = mobile_agent.get_pose().x

        # TPI movement depends on sim_core.sim_time — must tick sim_core
        dt = sim_core._dt
        for _ in range(100):
            sim_core.tick()
            mobile_agent.update(dt)

        # Should move toward goal via controller's TPI
        assert mobile_agent.get_pose().x > initial_x

    def test_controller_takes_priority_over_goal(self, mobile_agent):
        """When controller is set, it takes precedence over goal-based movement."""
        # Set a goal pointing in +X
        mobile_agent.set_goal_pose(Pose.from_xyz(10, 0, 0.05))
        # But controller commands -X velocity
        ctrl = OmniController()
        mobile_agent.set_controller(ctrl)
        ctrl.set_velocity(vx=-1.0, vy=0.0)

        initial_x = mobile_agent.get_pose().x
        mobile_agent.update(0.1)

        # Controller wins: robot moved in -X
        assert mobile_agent.get_pose().x < initial_x


# -----------------------------------------------------------------------
# Controller ABC contract
# -----------------------------------------------------------------------


class TestControllerABC:
    """Controller ABC enforces the compute() contract."""

    def test_cannot_instantiate_abstract(self):
        """Controller ABC cannot be instantiated directly."""
        with pytest.raises(TypeError):
            Controller()  # type: ignore[reportAbstractUsage]

    def test_velocity_controller_cannot_instantiate(self):
        """KinematicController is abstract (has abstract template hooks)."""
        with pytest.raises(TypeError):
            KinematicController()  # type: ignore[reportAbstractUsage]


# -----------------------------------------------------------------------
# Controller Registry
# -----------------------------------------------------------------------


class TestControllerRegistry:
    """Controller registry allows name-based creation."""

    def test_builtins_registered(self):
        """Built-in controllers are auto-registered."""
        assert "omni" in CONTROLLER_REGISTRY
        assert "differential" in CONTROLLER_REGISTRY

    def test_create_velocity(self):
        """create_controller('omni_velocity') returns OmniController."""
        ctrl = create_controller("omni")
        assert isinstance(ctrl, OmniController)

    def test_create_differential(self):
        """create_controller('differential_velocity') returns DifferentialController."""
        ctrl = create_controller("differential")
        assert isinstance(ctrl, DifferentialController)

    def test_create_with_config(self):
        """create_controller passes config to from_config()."""
        ctrl = create_controller("differential", {"max_linear_vel": 3.0, "max_angular_vel": 2.0})
        assert isinstance(ctrl, DifferentialController)
        assert ctrl._max_linear_vel == pytest.approx(3.0)
        assert ctrl._max_angular_vel == pytest.approx(2.0)

    def test_unknown_name_raises(self):
        """create_controller with unknown name raises KeyError."""
        with pytest.raises(KeyError, match="nonexistent"):
            create_controller("nonexistent")

    def test_register_custom(self):
        """register_controller adds a custom controller."""

        class MyController(Controller):
            def compute(self, agent, dt):
                return False

        register_controller("my_custom", MyController)
        assert "my_custom" in CONTROLLER_REGISTRY
        ctrl = create_controller("my_custom")
        assert isinstance(ctrl, MyController)

        # Cleanup
        del CONTROLLER_REGISTRY["my_custom"]


# -----------------------------------------------------------------------
# DifferentialController
# -----------------------------------------------------------------------


class TestDifferentialController:
    """DifferentialController uses unicycle kinematics."""

    def test_forward_only(self, diff_agent):
        """vx=1 at yaw=0 moves agent in +X only."""
        diff_agent._controller.set_velocity(vx=1.0)

        initial_x = diff_agent.get_pose().x
        initial_y = diff_agent.get_pose().y
        dt = 0.1
        diff_agent.update(dt)

        pose = diff_agent.get_pose()
        assert pose.x == pytest.approx(initial_x + 1.0 * dt, abs=0.01)
        assert pose.y == pytest.approx(initial_y, abs=0.01)

    def test_lateral_ignored(self, diff_agent):
        """vy is ignored — differential drive cannot strafe."""
        diff_agent._controller.set_velocity(vx=0.0, vy=5.0)  # vy should be ignored

        initial_x = diff_agent.get_pose().x
        initial_y = diff_agent.get_pose().y
        dt = 0.1
        diff_agent.update(dt)

        pose = diff_agent.get_pose()
        # Should not have moved at all (only vx matters, and it's 0)
        assert pose.x == pytest.approx(initial_x, abs=0.01)
        assert pose.y == pytest.approx(initial_y, abs=0.01)

    def test_rotation_only(self, diff_agent):
        """wz rotates in place to target yaw, no translation."""
        diff_agent._controller.set_velocity(vx=0.0, wz=1.0)

        initial_x = diff_agent.get_pose().x
        initial_y = diff_agent.get_pose().y
        initial_yaw = diff_agent.get_pose().yaw
        dt = 0.1
        diff_agent.update(dt)

        pose = diff_agent.get_pose()
        # Rotated to target yaw but didn't translate
        assert pose.x == pytest.approx(initial_x, abs=0.01)
        assert pose.y == pytest.approx(initial_y, abs=0.01)
        assert pose.yaw == pytest.approx(initial_yaw + 1.0 * dt, abs=0.01)

    def test_combined_arc(self, diff_agent):
        """vx + wz produces arc motion (non-zero lateral displacement proves arc, not straight line).

        Unicycle: R = v/wz = 1.0/0.5 = 2.0m.
        After 10×0.1s Euler integration: x ≈ 0.96, y ≈ 0.22, yaw ≈ 0.5 rad.
        """
        diff_agent._controller.set_velocity(vx=1.0, wz=0.5)

        initial_x = diff_agent.get_pose().x
        initial_y = diff_agent.get_pose().y
        dt = 0.1
        for _ in range(10):
            diff_agent.update(dt)

        pose = diff_agent.get_pose()
        # Y displacement > 0 proves arc, not straight line
        assert pose.y - initial_y == pytest.approx(0.22, abs=0.05)
        # Forward displacement
        assert pose.x - initial_x == pytest.approx(0.96, abs=0.05)
        # Yaw = wz * total_time = 0.5 * 1.0
        assert pose.yaw == pytest.approx(0.5, abs=0.02)

    def test_heading_affects_direction(self, diff_agent):
        """vx=1 at yaw=π/2 → moves in +Y to target y ≈ 0.1, x ≈ 0."""
        # Set initial heading to π/2
        diff_agent.set_pose(Pose.from_yaw(0, 0, 0.05, math.pi / 2))

        diff_agent._controller.set_velocity(vx=1.0)
        dt = 0.1
        diff_agent.update(dt)

        pose = diff_agent.get_pose()
        # At yaw=π/2, forward vx=1 → world +Y
        assert pose.y == pytest.approx(1.0 * dt, abs=0.02)
        assert abs(pose.x) < 0.02

    def test_speed_limit_symmetric(self, mobile_agent):
        """Speed exceeding max is clamped symmetrically (both forward and backward).

        Robots go backward, so limits are [-max, max], not [0, max].
        """
        ctrl = DifferentialController(max_linear_vel=1.0, max_angular_vel=0.5)
        mobile_agent.set_controller(ctrl)

        # Positive overshoot
        ctrl.set_velocity(vx=10.0, wz=5.0)
        assert ctrl._linear_velocity[0] == pytest.approx(1.0)
        assert ctrl._angular_velocity[2] == pytest.approx(0.5)

        # Negative overshoot (backward / reverse rotation)
        ctrl.set_velocity(vx=-10.0, wz=-5.0)
        assert ctrl._linear_velocity[0] == pytest.approx(-1.0)
        assert ctrl._angular_velocity[2] == pytest.approx(-0.5)

    def test_backward_motion(self, diff_agent):
        """Negative vx moves robot backward (opposite to heading)."""
        diff_agent._controller.set_velocity(vx=-1.0)

        initial_x = diff_agent.get_pose().x
        dt = 0.1
        diff_agent.update(dt)

        pose = diff_agent.get_pose()
        # At yaw=0, backward → -X
        assert pose.x == pytest.approx(initial_x - 1.0 * dt, abs=0.01)

    def test_zero_returns_false(self, diff_agent):
        """compute() returns False for zero velocity."""
        result = diff_agent._controller.compute(diff_agent, 0.1)
        assert result is False

    def test_nonzero_returns_true(self, diff_agent):
        """compute() returns True for nonzero velocity."""
        diff_agent._controller.set_velocity(vx=1.0)
        result = diff_agent._controller.compute(diff_agent, 0.1)
        assert result is True

    def test_stop_clears_velocity(self, diff_agent):
        """on_stop() zeros velocity state."""
        diff_agent._controller.set_velocity(vx=1.0, wz=0.5)
        diff_agent._controller.on_stop(diff_agent)
        np.testing.assert_allclose(diff_agent._controller._linear_velocity, 0.0)
        np.testing.assert_allclose(diff_agent._controller._angular_velocity, 0.0)

    def test_from_config(self):
        """from_config parses params correctly."""
        ctrl = DifferentialController.from_config({"max_linear_vel": 5.0, "max_angular_vel": 3.0, "wheel_separation": 0.4})
        assert ctrl._max_linear_vel == pytest.approx(5.0)  # type: ignore[reportAttributeAccessIssue]
        assert ctrl._max_angular_vel == pytest.approx(3.0)  # type: ignore[reportAttributeAccessIssue]
        assert ctrl._wheel_separation == pytest.approx(0.4)  # type: ignore[reportAttributeAccessIssue]

    def test_from_config_defaults(self):
        """from_config with empty dict uses defaults."""
        ctrl = DifferentialController.from_config({})
        assert ctrl._max_linear_vel == pytest.approx(2.0)  # type: ignore[reportAttributeAccessIssue]
        assert ctrl._max_angular_vel == pytest.approx(1.5)  # type: ignore[reportAttributeAccessIssue]


# -----------------------------------------------------------------------
# AgentSpawnParams.controller_config
# -----------------------------------------------------------------------


class TestSpawnParamsControllerConfig:
    """AgentSpawnParams.controller_config auto-creates controller."""

    def test_no_config_default_controller(self, sim_core):
        """Default: no controller_config → DifferentialController auto-assigned."""
        params = AgentSpawnParams(
            urdf_path="robots/mobile_robot.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0.05),
        )
        agent = Agent.from_params(params, sim_core)
        assert isinstance(agent._controller, DifferentialController)

    def test_velocity_config(self, sim_core):
        """controller_config with type=omni_velocity creates OmniController."""
        params = AgentSpawnParams(
            urdf_path="robots/mobile_robot.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0.05),
            controller_config={"type": "omni"},
        )
        agent = Agent.from_params(params, sim_core)
        assert isinstance(agent._controller, OmniController)

    def test_differential_config(self, sim_core):
        """controller_config with type=differential_velocity creates DifferentialController."""
        params = AgentSpawnParams(
            urdf_path="robots/mobile_robot.urdf",
            initial_pose=Pose.from_xyz(0, 0, 0.05),
            controller_config={"type": "differential", "max_linear_vel": 1.5},
        )
        agent = Agent.from_params(params, sim_core)
        assert isinstance(agent._controller, DifferentialController)
        assert agent._controller._max_linear_vel == pytest.approx(1.5)
