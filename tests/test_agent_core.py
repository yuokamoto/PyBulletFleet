"""
Tests for Agent class core functionality.

This module tests:
- Agent creation from mesh and URDF
- Agent properties and parameters
- Goal setting and path following
- Motion modes (omnidirectional vs differential)
- Velocity and acceleration constraints
- Agent state management
- Motion update and goal reaching
- URDF joint control (set_joint_target / get_joint_state)
- URDF link attach/detach
- Velocity/acceleration capping during motion
"""

import os

import numpy as np
import pybullet as p
import pybullet_data
import pytest

from pybullet_fleet.agent import Agent, AgentSpawnParams
from pybullet_fleet.geometry import Pose, Path
from pybullet_fleet.sim_object import SimObject, ShapeParams
from pybullet_fleet.types import CollisionMode, MotionMode


MESH_PATH = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
ARM_URDF = "robots/arm_robot.urdf"
MOBILE_URDF = "robots/mobile_robot.urdf"


class MockSimCore:
    """Minimal sim_core mock with advancing sim_time for motion update tests."""

    def __init__(self, dt: float = 1.0 / 240.0):
        self.sim_time = 0.0
        self._dt = dt
        self.sim_objects = []
        self._next_object_id = 0
        self._kinematic_objects = set()

    def add_object(self, obj):
        self.sim_objects.append(obj)

    def remove_object(self, obj):
        if obj in self.sim_objects:
            self.sim_objects.remove(obj)

    def _mark_object_moved(self, object_id):
        """No-op: real sim_core updates spatial hash here."""
        pass

    def tick(self, n: int = 1):
        """Advance sim_time by n time-steps."""
        self.sim_time += self._dt * n


@pytest.fixture
def pybullet_env():
    """Setup and teardown PyBullet environment"""
    physics_client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    plane_id = p.loadURDF("plane.urdf")

    yield physics_client, plane_id

    p.disconnect()


_SKIP = object()  # Sentinel to indicate "skip this check"


def assert_agent_properties(
    agent,
    *,
    # SimObject properties (defaults match create_mesh_agent defaults)
    mass=0.0,
    pickable=False,  # Agent default: not pickable
    name=None,
    position=(0, 0, 0),
    orientation=(0, 0, 0, 1),
    is_kinematic=True,  # mass=0 → kinematic
    collision_mode=CollisionMode.NORMAL_3D,
    # Agent-specific properties (defaults match Agent.__init__ defaults)
    motion_mode=MotionMode.OMNIDIRECTIONAL,
    max_linear_vel=(2.0, 2.0, 2.0),
    max_linear_accel=(5.0, 5.0, 5.0),
    max_angular_vel=(3.0, 3.0, 3.0),
    max_angular_accel=(10.0, 10.0, 10.0),
    is_moving=False,
    is_urdf=False,
    use_fixed_base=False,
    user_data=_SKIP,  # {} from SimObject but _SKIP avoids dict-in-default-arg issues
    # Tolerances
    pos_tolerance=0.01,
    orient_tolerance=0.01,
    vel_tolerance=1e-6,
):
    """
    Assert multiple Agent properties in one call.

    Defaults match the most common test pattern: a mesh agent created by
    create_mesh_agent() with all default arguments.  This means calling
    ``assert_agent_properties(agent)`` with no extra kwargs verifies the
    full default state, just like test_sim_object's assert_object_properties.

    Pass _SKIP to skip a specific check.

    Args:
        agent: Agent instance to verify
        mass: Expected mass (default 0.0 — create_mesh_agent default)
        pickable: Expected pickable flag (default False — Agent overrides SimObject)
        name: Expected name (default None)
        position: Expected (x, y, z) position tuple (default (0,0,0))
        orientation: Expected (x, y, z, w) quaternion (default (0,0,0,1))
        is_kinematic: Expected is_kinematic flag (default True — mass=0)
        collision_mode: Expected CollisionMode (default NORMAL_3D)
        motion_mode: Expected MotionMode (default OMNIDIRECTIONAL)
        max_linear_vel: Expected [vx, vy, vz] (default [2,2,2])
        max_linear_accel: Expected [ax, ay, az] (default [5,5,5])
        max_angular_vel: Expected [wx, wy, wz] (default [3,3,3])
        max_angular_accel: Expected [ex, ey, ez] (default [10,10,10])
        is_moving: Expected is_moving flag (default False)
        is_urdf: Expected is_urdf_robot() result (default False)
        use_fixed_base: Expected use_fixed_base flag (default False)
        user_data: Expected user_data dict (_SKIP by default)
        pos_tolerance: Position comparison tolerance (default 0.01)
        orient_tolerance: Orientation comparison tolerance (default 0.01)
        vel_tolerance: Velocity/acceleration comparison tolerance (default 1e-6)
    """
    assert agent is not None
    assert agent.body_id >= 0

    # SimObject properties
    if mass is not _SKIP:
        assert agent.mass == mass, f"Expected mass={mass}, got {agent.mass}"
    if pickable is not _SKIP:
        assert agent.pickable is pickable, f"Expected pickable={pickable}, got {agent.pickable}"
    if name is not _SKIP:
        assert agent.name == name, f"Expected name={name!r}, got {agent.name!r}"
    if position is not _SKIP or orientation is not _SKIP:
        pose = agent.get_pose()
        if position is not _SKIP:
            for axis, (actual, expected) in enumerate(zip(pose.position, position)):
                assert abs(actual - expected) < pos_tolerance, f"Position axis {axis}: expected {expected}, got {actual}"
        if orientation is not _SKIP:
            for axis, (actual, expected) in enumerate(zip(pose.orientation, orientation)):
                assert abs(actual - expected) < orient_tolerance, f"Orientation axis {axis}: expected {expected}, got {actual}"
    if is_kinematic is not _SKIP:
        assert agent.is_kinematic is is_kinematic, f"Expected is_kinematic={is_kinematic}, got {agent.is_kinematic}"
    if collision_mode is not _SKIP:
        assert agent.collision_mode == collision_mode, f"Expected collision_mode={collision_mode}, got {agent.collision_mode}"

    # Agent-specific properties
    if motion_mode is not _SKIP:
        assert agent.motion_mode == motion_mode, f"Expected motion_mode={motion_mode}, got {agent.motion_mode}"
    if max_linear_vel is not _SKIP:
        expected = np.array(max_linear_vel) if not isinstance(max_linear_vel, np.ndarray) else max_linear_vel
        assert np.allclose(
            agent.max_linear_vel, expected, atol=vel_tolerance
        ), f"Expected max_linear_vel={expected.tolist()}, got {agent.max_linear_vel.tolist()}"
    if max_linear_accel is not _SKIP:
        expected = np.array(max_linear_accel) if not isinstance(max_linear_accel, np.ndarray) else max_linear_accel
        assert np.allclose(
            agent.max_linear_accel, expected, atol=vel_tolerance
        ), f"Expected max_linear_accel={expected.tolist()}, got {agent.max_linear_accel.tolist()}"
    if max_angular_vel is not _SKIP:
        expected = np.array(max_angular_vel) if not isinstance(max_angular_vel, np.ndarray) else max_angular_vel
        assert np.allclose(
            agent.max_angular_vel, expected, atol=vel_tolerance
        ), f"Expected max_angular_vel={expected.tolist()}, got {agent.max_angular_vel.tolist()}"
    if max_angular_accel is not _SKIP:
        expected = np.array(max_angular_accel) if not isinstance(max_angular_accel, np.ndarray) else max_angular_accel
        assert np.allclose(
            agent.max_angular_accel, expected, atol=vel_tolerance
        ), f"Expected max_angular_accel={expected.tolist()}, got {agent.max_angular_accel.tolist()}"
    if is_moving is not _SKIP:
        assert agent.is_moving is is_moving, f"Expected is_moving={is_moving}, got {agent.is_moving}"
    if is_urdf is not _SKIP:
        assert agent.is_urdf_robot() is is_urdf, f"Expected is_urdf_robot()={is_urdf}, got {agent.is_urdf_robot()}"
    if use_fixed_base is not _SKIP:
        assert agent.use_fixed_base is use_fixed_base, f"Expected use_fixed_base={use_fixed_base}, got {agent.use_fixed_base}"
    if user_data is not _SKIP:
        assert agent.user_data == user_data, f"Expected user_data={user_data}, got {agent.user_data}"


def create_mesh_agent(
    pose=None,
    mass=0.0,
    max_linear_vel=2.0,
    max_linear_accel=5.0,
    max_angular_vel=3.0,
    max_angular_accel=10.0,
    motion_mode=MotionMode.OMNIDIRECTIONAL,
    use_fixed_base=False,
    collision_mode=CollisionMode.NORMAL_3D,
    name=None,
    user_data=None,
):
    """
    Create a mesh-based Agent with sensible defaults for testing.

    Reduces boilerplate for the common case of cube.obj visual + box collision.
    """
    return Agent.from_mesh(
        visual_shape=ShapeParams(shape_type="mesh", mesh_path=MESH_PATH, mesh_scale=[0.2, 0.2, 0.2]),
        collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
        pose=pose,
        mass=mass,
        max_linear_vel=max_linear_vel,
        max_linear_accel=max_linear_accel,
        max_angular_vel=max_angular_vel,
        max_angular_accel=max_angular_accel,
        motion_mode=motion_mode,
        use_fixed_base=use_fixed_base,
        collision_mode=collision_mode,
        name=name,
        user_data=user_data,
    )


class TestAgentCreationFromMesh:
    """Test Agent creation from mesh files"""

    def test_from_mesh_basic(self, pybullet_env):
        """Test basic agent creation with all default properties"""
        agent = create_mesh_agent(pose=Pose.from_xyz(0, 0, 0.5), mass=1.0)

        assert_agent_properties(
            agent,
            mass=1.0,
            pickable=False,  # Agent default
            name=None,
            position=(0, 0, 0.5),
            is_kinematic=False,
            motion_mode=MotionMode.OMNIDIRECTIONAL,
            max_linear_vel=[2.0, 2.0, 2.0],
            max_linear_accel=[5.0, 5.0, 5.0],
            max_angular_vel=[3.0, 3.0, 3.0],
            max_angular_accel=[10.0, 10.0, 10.0],
            is_moving=False,
            is_urdf=False,
            use_fixed_base=False,
            user_data={},
        )

    def test_from_mesh_kinematic(self, pybullet_env):
        """Test kinematic agent (mass=0)"""
        agent = create_mesh_agent(pose=Pose.from_xyz(0, 0, 0.5))

        assert_agent_properties(
            agent,
            mass=0.0,
            is_kinematic=True,
            position=(0, 0, 0.5),
        )

    def test_from_mesh_with_custom_params(self, pybullet_env):
        """Test agent creation with custom velocity/acceleration/mode"""
        agent = create_mesh_agent(
            pose=Pose.from_xyz(1, 2, 0.5),
            mass=2.5,
            max_linear_vel=0.8,
            max_linear_accel=1.5,
            motion_mode=MotionMode.DIFFERENTIAL,
        )

        assert_agent_properties(
            agent,
            mass=2.5,
            position=(1, 2, 0.5),
            is_kinematic=False,
            motion_mode=MotionMode.DIFFERENTIAL,
            max_linear_vel=[0.8, 0.8, 0.8],
            max_linear_accel=[1.5, 1.5, 1.5],
        )

    def test_from_mesh_with_name(self, pybullet_env):
        """Test agent with name"""
        agent = create_mesh_agent(
            pose=Pose.from_xyz(0, 0, 0.5),
            name="TestRobot",
        )

        assert_agent_properties(agent, name="TestRobot", position=(0, 0, 0.5))

    def test_from_mesh_with_user_data(self, pybullet_env):
        """Test agent with user_data"""
        agent = create_mesh_agent(
            pose=Pose.from_xyz(0, 0, 0.5),
            user_data={"role": "carrier", "priority": 1},
        )

        assert_agent_properties(
            agent,
            position=(0, 0, 0.5),
            user_data={"role": "carrier", "priority": 1},
        )

    def test_from_mesh_default_pose(self, pybullet_env):
        """Test that pose=None defaults to origin"""
        agent = create_mesh_agent()

        assert_agent_properties(agent, position=(0, 0, 0))

    def test_from_mesh_different_colors(self, pybullet_env):
        """Test creating agents with different colors have different body_ids"""
        agent1 = Agent.from_mesh(
            visual_shape=ShapeParams(
                shape_type="mesh", mesh_path=MESH_PATH, mesh_scale=[0.2, 0.2, 0.2], rgba_color=[1, 0, 0, 1]
            ),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 0.5),
        )
        agent2 = Agent.from_mesh(
            visual_shape=ShapeParams(
                shape_type="mesh", mesh_path=MESH_PATH, mesh_scale=[0.2, 0.2, 0.2], rgba_color=[0, 1, 0, 1]
            ),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(1, 0, 0.5),
        )

        assert agent1.body_id != agent2.body_id


class TestAgentCreationFromURDF:
    """Test Agent creation from URDF files"""

    def test_from_urdf_basic(self, pybullet_env):
        """Test creating agent from URDF with default properties"""
        agent = Agent.from_urdf(urdf_path="robots/mobile_robot.urdf", pose=Pose.from_xyz(0, 0, 0.5))

        assert_agent_properties(
            agent,
            mass=_SKIP,  # Mass comes from URDF
            is_kinematic=_SKIP,  # Depends on URDF mass
            position=(0, 0, 0.5),
            is_urdf=True,
        )
        assert agent.mass >= 0.5  # URDF should have some mass

    def test_from_urdf_with_custom_params(self, pybullet_env):
        """Test URDF agent with custom velocity/mode parameters"""
        agent = Agent.from_urdf(
            urdf_path="robots/mobile_robot.urdf",
            pose=Pose.from_xyz(1, 1, 0.5),
            mass=3.0,
            max_linear_vel=1.5,
            max_linear_accel=3.0,
            motion_mode=MotionMode.OMNIDIRECTIONAL,
        )

        assert_agent_properties(
            agent,
            mass=_SKIP,  # URDF mass, not overridden by parameter (mass>0 uses URDF values)
            is_kinematic=_SKIP,  # Depends on URDF mass
            position=(1, 1, 0.5),
            is_urdf=True,
            max_linear_vel=[1.5, 1.5, 1.5],
            max_linear_accel=[3.0, 3.0, 3.0],
        )

    def test_from_urdf_kinematic(self, pybullet_env):
        """Test URDF agent with mass=0 (kinematic control)"""
        agent = Agent.from_urdf(
            urdf_path="robots/mobile_robot.urdf",
            pose=Pose.from_xyz(0, 0, 0.5),
            mass=0.0,
        )

        assert_agent_properties(
            agent,
            mass=0.0,
            is_kinematic=True,
            is_urdf=True,
            position=(0, 0, 0.5),
        )


class TestAgentSpawnParams:
    """Test AgentSpawnParams dataclass"""

    def test_spawn_params_defaults(self):
        """Test spawn params with default values"""
        params = AgentSpawnParams(
            visual_shape=ShapeParams(shape_type="box", half_extents=[0.2, 0.2, 0.2]),
        )

        assert params.mass == 0.0
        assert params.pickable is True  # SimObjectSpawnParams default
        assert params.name is None
        assert params.max_linear_vel == 2.0
        assert params.max_linear_accel == 5.0
        assert params.max_angular_vel == 3.0
        assert params.max_angular_accel == 10.0
        assert params.motion_mode == MotionMode.OMNIDIRECTIONAL
        assert params.use_fixed_base is False
        assert params.urdf_path is None
        assert params.collision_mode == CollisionMode.NORMAL_3D
        assert params.user_data == {}

    def test_spawn_params_custom(self):
        """Test spawn params with custom values"""
        params = AgentSpawnParams(
            visual_shape=ShapeParams(shape_type="box", half_extents=[0.2, 0.2, 0.2]),
            mass=1.5,
            max_linear_vel=2.0,
            name="Robot_A",
            motion_mode=MotionMode.DIFFERENTIAL,
            user_data={"team": "blue"},
        )

        assert params.mass == 1.5
        assert params.max_linear_vel == 2.0
        assert params.name == "Robot_A"
        assert params.motion_mode == MotionMode.DIFFERENTIAL
        assert params.user_data == {"team": "blue"}

    def test_from_params_mesh(self, pybullet_env):
        """Test creating agent from mesh spawn parameters"""
        params = AgentSpawnParams(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=MESH_PATH, mesh_scale=[0.2, 0.2, 0.2]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            initial_pose=Pose.from_xyz(0, 0, 0.5),
            mass=2.0,
            max_linear_vel=1.0,
            motion_mode=MotionMode.DIFFERENTIAL,
            name="ParamsAgent",
            user_data={"source": "params"},
        )

        agent = Agent.from_params(params)

        assert_agent_properties(
            agent,
            mass=2.0,
            position=(0, 0, 0.5),
            is_kinematic=False,
            motion_mode=MotionMode.DIFFERENTIAL,
            max_linear_vel=[1.0, 1.0, 1.0],
            is_urdf=False,
            name="ParamsAgent",
            user_data={"source": "params"},
        )

    def test_from_params_urdf(self, pybullet_env):
        """Test creating agent from URDF spawn parameters"""
        params = AgentSpawnParams(
            urdf_path="robots/mobile_robot.urdf",
            initial_pose=Pose.from_xyz(1, 1, 0.5),
            max_linear_vel=1.5,
            motion_mode=MotionMode.OMNIDIRECTIONAL,
        )

        agent = Agent.from_params(params)

        assert_agent_properties(
            agent,
            mass=_SKIP,  # URDF mass
            is_kinematic=_SKIP,  # Depends on URDF mass
            position=(1, 1, 0.5),
            is_urdf=True,
            max_linear_vel=[1.5, 1.5, 1.5],
        )


class TestAgentGoalSetting:
    """Test agent goal and path setting"""

    def test_set_goal_pose(self, pybullet_env):
        """Test setting goal pose"""
        agent = create_mesh_agent(pose=Pose.from_xyz(0, 0, 0.5))

        # Initially no goal and not moving
        assert_agent_properties(agent, is_moving=False, position=(0, 0, 0.5))
        assert agent.goal_pose is None

        # Set goal
        agent.set_goal_pose(Pose.from_xyz(5, 0, 0.5))

        # Agent should be moving with a goal set
        assert agent.is_moving or agent.goal_pose is not None

    def test_set_path(self, pybullet_env):
        """Test setting path with waypoints"""
        agent = create_mesh_agent(pose=Pose.from_xyz(0, 0, 0.5))

        path = Path.from_positions([[1, 0, 0.5], [2, 0, 0.5], [2, 1, 0.5]])

        # Set path
        agent.set_path(path)

        # Agent should be moving or have path set
        assert agent.is_moving or hasattr(agent, "_path")


class TestAgentMotionModes:
    """Test different motion modes"""

    def test_omnidirectional_mode(self, pybullet_env):
        """Test omnidirectional motion mode"""
        agent = create_mesh_agent(
            pose=Pose.from_xyz(0, 0, 0.5),
            motion_mode=MotionMode.OMNIDIRECTIONAL,
        )

        assert_agent_properties(agent, motion_mode=MotionMode.OMNIDIRECTIONAL, position=(0, 0, 0.5))

    def test_differential_mode(self, pybullet_env):
        """Test differential drive motion mode"""
        agent = create_mesh_agent(
            pose=Pose.from_xyz(0, 0, 0.5),
            motion_mode=MotionMode.DIFFERENTIAL,
        )

        assert_agent_properties(agent, motion_mode=MotionMode.DIFFERENTIAL, position=(0, 0, 0.5))

    def test_motion_mode_string(self, pybullet_env):
        """Test motion mode accepts string values"""
        agent = create_mesh_agent(
            pose=Pose.from_xyz(0, 0, 0.5),
            motion_mode="differential",
        )

        assert_agent_properties(agent, motion_mode=MotionMode.DIFFERENTIAL, position=(0, 0, 0.5))

    def test_set_motion_mode(self, pybullet_env):
        """Test changing motion mode"""
        agent = create_mesh_agent(
            pose=Pose.from_xyz(0, 0, 0.5),
            motion_mode=MotionMode.OMNIDIRECTIONAL,
        )

        assert_agent_properties(agent, motion_mode=MotionMode.OMNIDIRECTIONAL, position=(0, 0, 0.5))

        # Change to differential
        success = agent.set_motion_mode("differential")
        assert success is True
        assert_agent_properties(agent, motion_mode=MotionMode.DIFFERENTIAL, position=(0, 0, 0.5))


class TestAgentVelocityConstraints:
    """Test velocity and acceleration constraints"""

    def test_scalar_velocity_constraint(self, pybullet_env):
        """Test scalar velocity constraint applied to all axes"""
        agent = create_mesh_agent(pose=Pose.from_xyz(0, 0, 0.5), max_linear_vel=1.5)

        assert_agent_properties(agent, max_linear_vel=[1.5, 1.5, 1.5], position=(0, 0, 0.5))

    def test_vector_velocity_constraint(self, pybullet_env):
        """Test per-axis velocity constraints"""
        agent = create_mesh_agent(pose=Pose.from_xyz(0, 0, 0.5), max_linear_vel=[1.0, 2.0, 0.5])

        assert_agent_properties(agent, max_linear_vel=[1.0, 2.0, 0.5], position=(0, 0, 0.5))

    def test_scalar_acceleration_constraint(self, pybullet_env):
        """Test scalar acceleration constraint"""
        agent = create_mesh_agent(pose=Pose.from_xyz(0, 0, 0.5), max_linear_accel=3.0)

        assert_agent_properties(agent, max_linear_accel=[3.0, 3.0, 3.0], position=(0, 0, 0.5))

    def test_vector_acceleration_constraint(self, pybullet_env):
        """Test per-axis acceleration constraints"""
        agent = create_mesh_agent(pose=Pose.from_xyz(0, 0, 0.5), max_linear_accel=[2.0, 4.0, 1.0])

        assert_agent_properties(agent, max_linear_accel=[2.0, 4.0, 1.0], position=(0, 0, 0.5))

    def test_scalar_angular_velocity(self, pybullet_env):
        """Test scalar angular velocity constraint"""
        agent = create_mesh_agent(pose=Pose.from_xyz(0, 0, 0.5), max_angular_vel=5.0)

        assert_agent_properties(agent, max_angular_vel=[5.0, 5.0, 5.0], position=(0, 0, 0.5))

    def test_scalar_angular_acceleration(self, pybullet_env):
        """Test scalar angular acceleration constraint"""
        agent = create_mesh_agent(pose=Pose.from_xyz(0, 0, 0.5), max_angular_accel=8.0)

        assert_agent_properties(agent, max_angular_accel=[8.0, 8.0, 8.0], position=(0, 0, 0.5))


class TestAgentProperties:
    """Test agent properties and state"""

    def test_is_urdf_robot_mesh(self, pybullet_env):
        """Test is_urdf_robot property for mesh robot"""
        agent = create_mesh_agent(pose=Pose.from_xyz(0, 0, 0.5))

        assert_agent_properties(agent, is_urdf=False, position=(0, 0, 0.5))

    def test_is_urdf_robot_urdf(self, pybullet_env):
        """Test is_urdf_robot property for URDF robot"""
        agent = Agent.from_urdf(urdf_path="robots/mobile_robot.urdf", pose=Pose.from_xyz(0, 0, 0.5))

        assert_agent_properties(
            agent,
            is_urdf=True,
            position=(0, 0, 0.5),
            mass=_SKIP,
            is_kinematic=_SKIP,
        )

    def test_agent_position_access(self, pybullet_env):
        """Test accessing agent position via get_pose"""
        agent = create_mesh_agent(pose=Pose.from_xyz(1, 2, 0.5))

        assert_agent_properties(agent, position=(1, 2, 0.5))

    def test_agent_mass_property(self, pybullet_env):
        """Test mass property"""
        agent = create_mesh_agent(pose=Pose.from_xyz(0, 0, 0.5), mass=3.5)

        assert_agent_properties(agent, mass=3.5, is_kinematic=False, position=(0, 0, 0.5))

    def test_agent_initial_velocity_zero(self, pybullet_env):
        """Test that initial velocity is zero"""
        agent = create_mesh_agent(pose=Pose.from_xyz(0, 0, 0.5))
        velocity = agent.velocity
        assert np.allclose(velocity, [0.0, 0.0, 0.0])

    def test_agent_initial_angular_velocity_zero(self, pybullet_env):
        """Test that initial angular velocity is zero"""
        agent = create_mesh_agent(pose=Pose.from_xyz(0, 0, 0.5))
        assert agent.angular_velocity == 0.0


class TestAgentFixedBase:
    """Test fixed base agents"""

    def test_fixed_base_agent(self, pybullet_env):
        """Test creating fixed base agent"""
        agent = create_mesh_agent(pose=Pose.from_xyz(0, 0, 0.5), use_fixed_base=True)

        assert_agent_properties(agent, use_fixed_base=True, position=(0, 0, 0.5))

        # Position should not change even after simulation steps
        initial_pos, _ = p.getBasePositionAndOrientation(agent.body_id)

        for _ in range(100):
            p.stepSimulation()

        final_pos, _ = p.getBasePositionAndOrientation(agent.body_id)

        # Position should remain the same (fixed base)
        assert np.allclose(initial_pos, final_pos, atol=0.001)


# ============================================================================
# Motion update / goal reaching tests
# ============================================================================


class TestAgentMotionUpdate:
    """Test agent.update() moves toward goals and eventually reaches them."""

    def _create_agent_with_simcore(
        self,
        pose,
        *,
        max_linear_vel=2.0,
        max_linear_accel=5.0,
        max_angular_vel=3.0,
        max_angular_accel=10.0,
        motion_mode=MotionMode.OMNIDIRECTIONAL,
    ):
        """Helper: create mesh agent wired to a MockSimCore."""
        sim_core = MockSimCore()
        agent = Agent.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=MESH_PATH, mesh_scale=[0.2, 0.2, 0.2]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=pose,
            mass=0.0,
            max_linear_vel=max_linear_vel,
            max_linear_accel=max_linear_accel,
            max_angular_vel=max_angular_vel,
            max_angular_accel=max_angular_accel,
            motion_mode=motion_mode,
            sim_core=sim_core,
        )
        return agent, sim_core

    def _run_update_loop(self, agent, sim_core, *, max_steps=5000):
        """Advance sim_core time and call agent.update() until stopped or max_steps."""
        dt = sim_core._dt
        for _ in range(max_steps):
            sim_core.tick()
            agent.update(dt)
            if not agent.is_moving:
                return True
        return False  # Did not reach goal within max_steps

    # -- Omnidirectional -------------------------------------------------

    def test_omni_update_moves_toward_goal(self, pybullet_env):
        """After a few updates the agent should be closer to the goal."""
        agent, sim_core = self._create_agent_with_simcore(Pose.from_xyz(0, 0, 0))
        agent.set_goal_pose(Pose.from_xyz(5, 0, 0))

        start_pos = np.array(agent.get_pose().position)
        goal_pos = np.array([5, 0, 0])

        # Run a few steps
        dt = sim_core._dt
        for _ in range(100):
            sim_core.tick()
            agent.update(dt)

        cur_pos = np.array(agent.get_pose().position)
        assert np.linalg.norm(cur_pos - goal_pos) < np.linalg.norm(
            start_pos - goal_pos
        ), "Agent should be closer to goal after updates"

    def test_omni_reaches_goal(self, pybullet_env):
        """Agent should reach a nearby goal and stop (is_moving=False)."""
        agent, sim_core = self._create_agent_with_simcore(
            Pose.from_xyz(0, 0, 0),
            max_linear_vel=2.0,
            max_linear_accel=5.0,
        )
        goal = Pose.from_xyz(1, 0, 0)
        agent.set_goal_pose(goal)

        reached = self._run_update_loop(agent, sim_core)

        assert reached, "Agent should reach the goal within max_steps"
        assert not agent.is_moving
        pos = np.array(agent.get_pose().position)
        assert np.allclose(pos, [1, 0, 0], atol=0.05), f"Final position {pos} not close to goal"

    def test_omni_reaches_goal_3d(self, pybullet_env):
        """Agent reaches a goal in full 3D (X, Y and Z differ)."""
        agent, sim_core = self._create_agent_with_simcore(Pose.from_xyz(0, 0, 0))
        goal = Pose.from_xyz(1, 2, 0.5)
        agent.set_goal_pose(goal)

        reached = self._run_update_loop(agent, sim_core)
        assert reached
        pos = np.array(agent.get_pose().position)
        assert np.allclose(pos, [1, 2, 0.5], atol=0.05)

    def test_omni_velocity_nonzero_during_motion(self, pybullet_env):
        """While moving, velocity magnitude should be positive."""
        agent, sim_core = self._create_agent_with_simcore(Pose.from_xyz(0, 0, 0))
        agent.set_goal_pose(Pose.from_xyz(5, 0, 0))

        dt = sim_core._dt
        # Skip the very first step (may be zero) and check the next ones
        for _ in range(50):
            sim_core.tick()
            agent.update(dt)

        vel = agent.velocity
        assert np.linalg.norm(vel) > 0.01, f"Velocity should be positive during motion, got {vel}"

    def test_omni_velocity_zero_after_goal(self, pybullet_env):
        """After reaching goal, velocity should be zero."""
        agent, sim_core = self._create_agent_with_simcore(Pose.from_xyz(0, 0, 0))
        agent.set_goal_pose(Pose.from_xyz(1, 0, 0))

        self._run_update_loop(agent, sim_core)

        vel = agent.velocity
        assert np.allclose(vel, [0, 0, 0], atol=1e-4), f"Velocity should be ~0 after goal, got {vel}"

    def test_omni_path_multi_waypoint(self, pybullet_env):
        """Agent follows a multi-waypoint path and reaches the final position."""
        agent, sim_core = self._create_agent_with_simcore(Pose.from_xyz(0, 0, 0))
        path = [Pose.from_xyz(1, 0, 0), Pose.from_xyz(1, 1, 0), Pose.from_xyz(0, 1, 0)]
        agent.set_path(path, auto_approach=False, final_orientation_align=False)

        reached = self._run_update_loop(agent, sim_core, max_steps=10000)
        assert reached
        pos = np.array(agent.get_pose().position)
        assert np.allclose(pos, [0, 1, 0], atol=0.05), f"Final pos {pos} not close to last waypoint"

    def test_omni_stop_clears_goal(self, pybullet_env):
        """Calling stop() should clear the goal and stop motion."""
        agent, sim_core = self._create_agent_with_simcore(Pose.from_xyz(0, 0, 0))
        agent.set_goal_pose(Pose.from_xyz(10, 0, 0))
        assert agent.is_moving

        agent.stop()
        assert not agent.is_moving
        assert agent.goal_pose is None
        assert np.allclose(agent.velocity, [0, 0, 0])

    # -- Differential ----------------------------------------------------

    def test_diff_reaches_goal(self, pybullet_env):
        """Differential-drive agent should reach a nearby goal."""
        agent, sim_core = self._create_agent_with_simcore(
            Pose.from_xyz(0, 0, 0),
            motion_mode=MotionMode.DIFFERENTIAL,
        )
        agent.set_goal_pose(Pose.from_xyz(2, 0, 0))

        reached = self._run_update_loop(agent, sim_core, max_steps=10000)
        assert reached, "Differential agent should reach the goal"
        pos = np.array(agent.get_pose().position)
        assert np.allclose(pos, [2, 0, 0], atol=0.05)

    def test_diff_rotates_then_moves_forward(self, pybullet_env):
        """Differential agent should first rotate, then move forward."""
        agent, sim_core = self._create_agent_with_simcore(
            Pose.from_xyz(0, 0, 0),
            motion_mode=MotionMode.DIFFERENTIAL,
        )
        # Goal is at 90° to the right (positive Y)
        agent.set_goal_pose(Pose.from_xyz(0, 2, 0))

        dt = sim_core._dt
        # Early steps: rotation phase — agent stays near origin
        for _ in range(50):
            sim_core.tick()
            agent.update(dt)

        early_pos = np.array(agent.get_pose().position)
        # Should still be very close to origin during rotation
        assert np.linalg.norm(early_pos) < 0.5, f"During rotation phase agent should stay near origin, pos={early_pos}"

        # Run to completion
        reached = self._run_update_loop(agent, sim_core, max_steps=10000)
        assert reached
        pos = np.array(agent.get_pose().position)
        assert np.allclose(pos, [0, 2, 0], atol=0.05)


# ============================================================================
# Joint control tests (URDF)
# ============================================================================


class TestAgentJointControl:
    """Test URDF joint state and control methods.

    arm_robot.urdf has 4 revolute joints:
        0: base_to_shoulder   (axis Z, ±π)
        1: shoulder_to_elbow  (axis X, ±2.0)
        2: elbow_to_wrist     (axis X, ±2.5)
        3: wrist_to_end       (axis Z, ±1.57)
    """

    ARM_JOINT_NAMES = [
        "base_to_shoulder",
        "shoulder_to_elbow",
        "elbow_to_wrist",
        "wrist_to_end",
    ]

    def _create_arm(self, pybullet_env, *, use_fixed_base=True, mass=1.0):
        agent = Agent.from_urdf(
            urdf_path=ARM_URDF,
            pose=Pose.from_xyz(0, 0, 0.5),
            mass=mass,
            use_fixed_base=use_fixed_base,
        )
        return agent

    # -- Joint info / consistency ----------------------------------------

    def test_num_joints_matches_urdf(self, pybullet_env):
        """get_num_joints() should match the URDF (4 joints for arm_robot)."""
        agent = self._create_arm(pybullet_env)
        assert agent.get_num_joints() == 4

    def test_joint_names_match_urdf(self, pybullet_env):
        """joint_info names should match the URDF joint names."""
        agent = self._create_arm(pybullet_env)
        names = [info[1].decode("utf-8") for info in agent.joint_info]
        assert names == self.ARM_JOINT_NAMES

    def test_joint_types_all_revolute(self, pybullet_env):
        """All arm_robot joints should be revolute (type 0 in pybullet)."""
        agent = self._create_arm(pybullet_env)
        for info in agent.joint_info:
            assert info[2] == p.JOINT_REVOLUTE, f"Joint {info[1]} should be revolute"

    def test_is_urdf_robot(self, pybullet_env):
        """URDF agent should return True from is_urdf_robot()."""
        agent = self._create_arm(pybullet_env)
        assert agent.is_urdf_robot() is True

    def test_mesh_agent_no_joints(self, pybullet_env):
        """Mesh agent should have 0 joints."""
        agent = create_mesh_agent(pose=Pose.from_xyz(0, 0, 0.5))
        assert agent.get_num_joints() == 0
        assert agent.is_urdf_robot() is False

    # -- get_joint_state / set_joint_target (by index) -------------------

    def test_initial_joint_positions_zero(self, pybullet_env):
        """All joints should start at position 0."""
        agent = self._create_arm(pybullet_env)
        for i in range(agent.get_num_joints()):
            pos, vel = agent.get_joint_state(i)
            assert abs(pos) < 0.01, f"Joint {i} should start at 0, got {pos}"

    def test_set_joint_target_and_simulate(self, pybullet_env):
        """set_joint_target should move joint toward target after simulation steps."""
        agent = self._create_arm(pybullet_env)
        target = 0.5  # radians
        agent.set_joint_target(0, target)

        for _ in range(500):
            p.stepSimulation()

        pos, _ = agent.get_joint_state(0)
        assert abs(pos - target) < 0.05, f"Joint 0 should be near {target}, got {pos}"

    def test_is_joint_at_target(self, pybullet_env):
        """is_joint_at_target should return True once joint reaches target."""
        agent = self._create_arm(pybullet_env)
        target = 0.3
        agent.set_joint_target(0, target)

        for _ in range(500):
            p.stepSimulation()

        assert agent.is_joint_at_target(0, target, tolerance=0.05)

    # -- get/set by name -------------------------------------------------

    def test_get_joint_state_by_name(self, pybullet_env):
        """get_joint_state_by_name should return same result as by index."""
        agent = self._create_arm(pybullet_env)
        pos_idx, vel_idx = agent.get_joint_state(0)
        pos_name, vel_name = agent.get_joint_state_by_name("base_to_shoulder")
        assert pos_idx == pos_name
        assert vel_idx == vel_name

    def test_set_joint_target_by_name(self, pybullet_env):
        """set_joint_target_by_name should move joint like set_joint_target."""
        agent = self._create_arm(pybullet_env)
        target = 0.4
        agent.set_joint_target_by_name("shoulder_to_elbow", target)

        for _ in range(500):
            p.stepSimulation()

        assert agent.is_joint_at_target_by_name("shoulder_to_elbow", target, tolerance=0.05)

    def test_is_joint_at_target_by_name(self, pybullet_env):
        """is_joint_at_target_by_name should work correctly."""
        agent = self._create_arm(pybullet_env)
        # Initially all joints at 0
        assert agent.is_joint_at_target_by_name("base_to_shoulder", 0.0, tolerance=0.01)

    # -- Batch operations ------------------------------------------------

    def test_get_all_joints_state(self, pybullet_env):
        """get_all_joints_state returns list of (pos, vel) for every joint."""
        agent = self._create_arm(pybullet_env)
        states = agent.get_all_joints_state()
        assert len(states) == 4
        for pos, vel in states:
            assert abs(pos) < 0.01
            assert abs(vel) < 0.01

    def test_get_all_joints_state_by_name(self, pybullet_env):
        """get_all_joints_state_by_name returns dict keyed by joint name."""
        agent = self._create_arm(pybullet_env)
        states = agent.get_all_joints_state_by_name()
        assert set(states.keys()) == set(self.ARM_JOINT_NAMES)

    def test_set_all_joints_targets(self, pybullet_env):
        """set_all_joints_targets moves all joints to specified positions."""
        agent = self._create_arm(pybullet_env)
        targets = [0.3, 0.2, -0.2, 0.1]
        agent.set_all_joints_targets(targets)

        for _ in range(1000):
            p.stepSimulation()

        assert agent.are_all_joints_at_targets(targets, tolerance=0.05)

    def test_set_joints_targets_by_name(self, pybullet_env):
        """set_joints_targets_by_name (partial update) works correctly."""
        agent = self._create_arm(pybullet_env)
        partial = {"base_to_shoulder": 0.5, "elbow_to_wrist": -0.3}
        agent.set_joints_targets_by_name(partial)

        for _ in range(1000):
            p.stepSimulation()

        assert agent.are_joints_at_targets_by_name(partial, tolerance=0.05)

    def test_set_joints_targets_list_and_dict(self, pybullet_env):
        """set_joints_targets accepts both list and dict."""
        agent = self._create_arm(pybullet_env)

        # list form
        targets_list = [0.1, 0.2, 0.3, 0.1]
        agent.set_joints_targets(targets_list)
        for _ in range(1000):
            p.stepSimulation()
        assert agent.are_joints_at_targets(targets_list, tolerance=0.05)

        # dict form
        targets_dict = {"base_to_shoulder": -0.2, "wrist_to_end": 0.4}
        agent.set_joints_targets(targets_dict)
        for _ in range(1000):
            p.stepSimulation()
        assert agent.are_joints_at_targets(targets_dict, tolerance=0.05)

    # -- Edge cases / warnings -------------------------------------------

    def test_mesh_agent_joint_state_returns_zero(self, pybullet_env):
        """Calling get_joint_state on mesh agent returns (0, 0) gracefully."""
        agent = create_mesh_agent(pose=Pose.from_xyz(0, 0, 0.5))
        pos, vel = agent.get_joint_state(0)
        assert pos == 0.0
        assert vel == 0.0

    def test_get_joints_state_by_name_subset(self, pybullet_env):
        """get_joints_state_by_name with a subset of joint names."""
        agent = self._create_arm(pybullet_env)
        subset = ["base_to_shoulder", "wrist_to_end"]
        states = agent.get_joints_state_by_name(subset)
        assert set(states.keys()) == set(subset)


# ============================================================================
# URDF link attach / detach tests
# ============================================================================


class TestAgentURDFLinkAttach:
    """Test attaching/detaching SimObjects to URDF robot links.

    arm_robot.urdf links:
        -1: base_link
         0: shoulder_link  (child of base_to_shoulder joint)
         1: elbow_link     (child of shoulder_to_elbow joint)
         2: wrist_link     (child of elbow_to_wrist joint)
         3: end_effector   (child of wrist_to_end joint)
    """

    def _create_arm_agent(self, pybullet_env):
        return Agent.from_urdf(
            urdf_path=ARM_URDF,
            pose=Pose.from_xyz(0, 0, 0.5),
            mass=1.0,
            use_fixed_base=True,
        )

    def _create_pickable_box(self, pos=(0, 0, 0)):
        """Create a small pickable SimObject for attachment."""
        obj = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="box", half_extents=[0.02, 0.02, 0.02]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.02, 0.02, 0.02]),
            pose=Pose.from_xyz(*pos),
            mass=0.0,
            pickable=True,
        )
        return obj

    def test_attach_to_end_effector(self, pybullet_env):
        """Attach a box to the end effector link (index 3)."""
        agent = self._create_arm_agent(pybullet_env)
        box = self._create_pickable_box()

        success = agent.attach_object(box, parent_link_index=3)
        assert success is True
        assert box in agent.attached_objects
        assert box.is_attached()
        assert box._attached_link_index == 3

    def test_attach_to_base_link(self, pybullet_env):
        """Attach to base link (index -1)."""
        agent = self._create_arm_agent(pybullet_env)
        box = self._create_pickable_box()

        success = agent.attach_object(box, parent_link_index=-1)
        assert success is True
        assert box._attached_link_index == -1

    def test_detach_from_link(self, pybullet_env):
        """Detach a previously attached object."""
        agent = self._create_arm_agent(pybullet_env)
        box = self._create_pickable_box()

        agent.attach_object(box, parent_link_index=3)
        success = agent.detach_object(box)

        assert success is True
        assert box not in agent.attached_objects
        assert not box.is_attached()
        assert box._attached_link_index == -1

    def test_attach_follows_link_after_joint_move(self, pybullet_env):
        """Kinematic attached object should follow link when joint moves."""
        agent = self._create_arm_agent(pybullet_env)
        box = self._create_pickable_box()

        # Attach to end effector (link 3)
        agent.attach_object(box, parent_link_index=3, relative_pose=Pose(position=[0, 0, 0.1], orientation=[0, 0, 0, 1]))

        # Update kinematics and record initial position
        agent.update_attached_objects_kinematics()
        initial_pos = np.array(box.get_pose().position)

        # Move shoulder pitch joint (joint 1: shoulder_to_elbow, axis X)
        # This tilts the arm and significantly moves the end effector
        agent.set_joint_target(1, 1.0)
        for _ in range(1000):
            p.stepSimulation()

        # Verify joint actually moved
        pos1, _ = agent.get_joint_state(1)
        assert abs(pos1 - 1.0) < 0.1, f"Joint 1 should have moved to ~1.0, got {pos1}"

        # Update attached objects kinematics
        agent.update_attached_objects_kinematics()
        new_pos = np.array(box.get_pose().position)

        # Box should have moved because the end effector moved
        assert not np.allclose(
            initial_pos, new_pos, atol=0.01
        ), f"Attached box should move when joint moves: initial={initial_pos}, new={new_pos}"

    def test_attach_multiple_to_different_links(self, pybullet_env):
        """Attach different objects to different links."""
        agent = self._create_arm_agent(pybullet_env)
        box1 = self._create_pickable_box(pos=(1, 0, 0))
        box2 = self._create_pickable_box(pos=(2, 0, 0))

        s1 = agent.attach_object(box1, parent_link_index=1)
        s2 = agent.attach_object(box2, parent_link_index=3)

        assert s1 and s2
        assert len(agent.attached_objects) == 2
        assert box1._attached_link_index == 1
        assert box2._attached_link_index == 3

    def test_detach_one_keeps_other(self, pybullet_env):
        """Detaching one object should not affect another."""
        agent = self._create_arm_agent(pybullet_env)
        box1 = self._create_pickable_box(pos=(1, 0, 0))
        box2 = self._create_pickable_box(pos=(2, 0, 0))

        agent.attach_object(box1, parent_link_index=1)
        agent.attach_object(box2, parent_link_index=3)

        agent.detach_object(box1)

        assert box1 not in agent.attached_objects
        assert box2 in agent.attached_objects
        assert box2.is_attached()

    def test_cannot_attach_non_pickable(self, pybullet_env):
        """Non-pickable objects should not be attachable."""
        agent = self._create_arm_agent(pybullet_env)
        box = self._create_pickable_box()
        box.pickable = False

        success = agent.attach_object(box, parent_link_index=3)
        assert success is False

    def test_cannot_double_attach(self, pybullet_env):
        """An already-attached object cannot be attached to a second parent."""
        agent1 = self._create_arm_agent(pybullet_env)
        agent2 = self._create_arm_agent(pybullet_env)
        box = self._create_pickable_box()

        s1 = agent1.attach_object(box, parent_link_index=3)
        s2 = agent2.attach_object(box, parent_link_index=3)

        assert s1 is True
        assert s2 is False  # Already attached to agent1


# ============================================================================
# Velocity / acceleration capping tests
# ============================================================================


class TestAgentVelocityCapping:
    """Test that velocity and acceleration never exceed configured max during motion.

    The TPI trajectory planner should respect max_linear_vel and max_linear_accel.
    We verify this by sampling velocity during the entire motion and checking all
    values stay within bounds.
    """

    def _create_agent_with_simcore(
        self,
        pose,
        *,
        max_linear_vel=1.0,
        max_linear_accel=2.0,
        max_angular_vel=1.5,
        max_angular_accel=3.0,
        motion_mode=MotionMode.OMNIDIRECTIONAL,
    ):
        sim_core = MockSimCore()
        agent = Agent.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=MESH_PATH, mesh_scale=[0.2, 0.2, 0.2]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=pose,
            mass=0.0,
            max_linear_vel=max_linear_vel,
            max_linear_accel=max_linear_accel,
            max_angular_vel=max_angular_vel,
            max_angular_accel=max_angular_accel,
            motion_mode=motion_mode,
            sim_core=sim_core,
        )
        return agent, sim_core

    def test_omni_velocity_capped_scalar(self, pybullet_env):
        """Velocity magnitude per axis should never exceed max_linear_vel."""
        max_vel = 1.0
        agent, sim_core = self._create_agent_with_simcore(
            Pose.from_xyz(0, 0, 0),
            max_linear_vel=max_vel,
        )
        agent.set_goal_pose(Pose.from_xyz(10, 0, 0))

        dt = sim_core._dt
        max_observed_vel = 0.0
        for _ in range(3000):
            sim_core.tick()
            agent.update(dt)
            vel = agent.velocity
            for axis in range(3):
                max_observed_vel = max(max_observed_vel, abs(vel[axis]))
            if not agent.is_moving:
                break

        # Allow small tolerance for floating point
        assert (
            max_observed_vel <= max_vel + 0.01
        ), f"Velocity axis max {max_observed_vel:.4f} exceeded max_linear_vel={max_vel}"

    def test_omni_velocity_capped_per_axis(self, pybullet_env):
        """Per-axis velocity limits should each be respected."""
        limits = [0.5, 1.0, 0.3]
        agent, sim_core = self._create_agent_with_simcore(
            Pose.from_xyz(0, 0, 0),
            max_linear_vel=limits,
        )
        agent.set_goal_pose(Pose.from_xyz(5, 5, 2))

        dt = sim_core._dt
        max_per_axis = [0.0, 0.0, 0.0]
        for _ in range(5000):
            sim_core.tick()
            agent.update(dt)
            vel = agent.velocity
            for axis in range(3):
                max_per_axis[axis] = max(max_per_axis[axis], abs(vel[axis]))
            if not agent.is_moving:
                break

        for axis in range(3):
            assert (
                max_per_axis[axis] <= limits[axis] + 0.01
            ), f"Axis {axis}: max vel {max_per_axis[axis]:.4f} exceeded limit {limits[axis]}"

    def test_omni_acceleration_capped(self, pybullet_env):
        """Acceleration magnitude per axis should never exceed max_linear_accel.

        We compute approximate acceleration as Δv / dt between consecutive updates.
        """
        max_accel = 2.0
        agent, sim_core = self._create_agent_with_simcore(
            Pose.from_xyz(0, 0, 0),
            max_linear_accel=max_accel,
        )
        agent.set_goal_pose(Pose.from_xyz(10, 0, 0))

        dt = sim_core._dt
        prev_vel = agent.velocity.copy()
        max_observed_accel = 0.0

        for _ in range(3000):
            sim_core.tick()
            agent.update(dt)
            cur_vel = agent.velocity
            accel = np.abs((cur_vel - prev_vel) / dt) if dt > 0 else np.zeros(3)
            for axis in range(3):
                max_observed_accel = max(max_observed_accel, accel[axis])
            prev_vel = cur_vel.copy()
            if not agent.is_moving:
                break

        # Larger tolerance because discrete approximation of acceleration can overshoot slightly
        assert (
            max_observed_accel <= max_accel + 0.5
        ), f"Acceleration max {max_observed_accel:.4f} exceeded max_linear_accel={max_accel}"

    def test_diff_angular_velocity_capped(self, pybullet_env):
        """Differential drive angular velocity should not exceed max_angular_vel."""
        max_ang_vel = 1.5
        agent, sim_core = self._create_agent_with_simcore(
            Pose.from_xyz(0, 0, 0),
            max_angular_vel=max_ang_vel,
            motion_mode=MotionMode.DIFFERENTIAL,
        )
        # Goal at 90° requires rotation
        agent.set_goal_pose(Pose.from_xyz(0, 3, 0))

        dt = sim_core._dt
        max_observed_ang_vel = 0.0
        for _ in range(5000):
            sim_core.tick()
            agent.update(dt)
            max_observed_ang_vel = max(max_observed_ang_vel, abs(agent.angular_velocity))
            if not agent.is_moving:
                break

        assert (
            max_observed_ang_vel <= max_ang_vel + 0.01
        ), f"Angular velocity max {max_observed_ang_vel:.4f} exceeded max_angular_vel={max_ang_vel}"

    def test_low_velocity_limit_still_reaches_goal(self, pybullet_env):
        """Even with very low velocity limits, agent should still reach goal."""
        agent, sim_core = self._create_agent_with_simcore(
            Pose.from_xyz(0, 0, 0),
            max_linear_vel=0.2,
            max_linear_accel=0.5,
        )
        agent.set_goal_pose(Pose.from_xyz(0.5, 0, 0))

        dt = sim_core._dt
        for _ in range(10000):
            sim_core.tick()
            agent.update(dt)
            if not agent.is_moving:
                break

        assert not agent.is_moving, "Agent should eventually reach goal even with low velocity"
        pos = np.array(agent.get_pose().position)
        assert np.allclose(pos, [0.5, 0, 0], atol=0.05)

    def test_high_velocity_limit_respects_cap(self, pybullet_env):
        """With high max_linear_vel, velocity should still not exceed it."""
        max_vel = 10.0
        agent, sim_core = self._create_agent_with_simcore(
            Pose.from_xyz(0, 0, 0),
            max_linear_vel=max_vel,
            max_linear_accel=50.0,
        )
        agent.set_goal_pose(Pose.from_xyz(100, 0, 0))

        dt = sim_core._dt
        max_observed = 0.0
        for _ in range(3000):
            sim_core.tick()
            agent.update(dt)
            vel = agent.velocity
            max_observed = max(max_observed, np.linalg.norm(vel))
            if not agent.is_moving:
                break

        # Per-axis cap: each axis capped at 10, so norm ≤ 10 for single-axis motion
        assert max_observed <= max_vel + 0.1, f"Velocity norm {max_observed:.4f} exceeded max_linear_vel={max_vel}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
