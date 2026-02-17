"""
Tests for SimObject class.

This module tests:
- SimObject creation (from_mesh, from_params)
- Shape parameters (ShapeParams)
- Pose get/set operations
- Static vs dynamic objects
- Collision modes
- Object properties (name, pickable, mass)
"""

import os

import pybullet as p
import pybullet_data
import pytest

from pybullet_fleet.sim_object import SimObject, SimObjectSpawnParams, ShapeParams
from pybullet_fleet.geometry import Pose
from pybullet_fleet.types import CollisionMode


@pytest.fixture
def pybullet_env():
    """Setup and teardown PyBullet environment for each test"""
    physics_client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)

    yield physics_client

    p.disconnect()


class TestSimObjectCreationFromMesh:
    """Test SimObject creation from mesh files"""

    def test_create_from_mesh_basic(self, pybullet_env):
        """Test basic mesh object creation"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        obj = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 1),
            mass=1.0,
        )

        assert obj is not None
        assert obj.body_id >= 0
        assert obj.mass == 1.0
        assert obj.pickable is True

    def test_create_from_mesh_with_params(self, pybullet_env):
        """Test mesh creation with various parameters"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        obj = SimObject.from_mesh(
            visual_shape=ShapeParams(
                shape_type="mesh",
                mesh_path=mesh_path,
                mesh_scale=[0.5, 0.5, 0.5],
                rgba_color=[1, 0, 0, 1],
            ),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.05, 0.05, 0.05]),
            pose=Pose.from_xyz(1, 2, 3),
            mass=2.5,
            pickable=False,
        )

        assert obj.mass == 2.5
        assert obj.pickable is False
        pos = obj.get_pose().position
        assert abs(pos[0] - 1) < 0.01
        assert abs(pos[1] - 2) < 0.01
        assert abs(pos[2] - 3) < 0.01

    def test_create_static_object(self, pybullet_env):
        """Test creating static (mass=0) object"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        obj = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 0),
            mass=0.0,
        )

        assert obj.mass == 0.0
        # is_static is a property, not a method
        assert obj.is_static is False  # Note: is_static checks CollisionMode.STATIC, not mass
        assert obj.is_kinematic is True  # mass=0 sets is_kinematic


class TestSimObjectCreationFromParams:
    """Test SimObject creation from spawn parameters"""

    def test_create_from_params_box(self, pybullet_env):
        """Test creation from params with box shape"""
        params = SimObjectSpawnParams(
            visual_shape=ShapeParams(shape_type="box", half_extents=[0.5, 0.5, 0.5], rgba_color=[1, 0, 0, 1]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.5, 0.5, 0.5]),
            initial_pose=Pose.from_xyz(0, 0, 1),
            mass=1.0,
            pickable=True,
        )

        obj = SimObject.from_params(params)

        assert obj is not None
        assert obj.mass == 1.0
        assert obj.pickable is True

    def test_create_from_params_sphere(self, pybullet_env):
        """Test creation from params with sphere shape"""
        params = SimObjectSpawnParams(
            visual_shape=ShapeParams(shape_type="sphere", radius=0.3, rgba_color=[0, 1, 0, 1]),
            collision_shape=ShapeParams(shape_type="sphere", radius=0.3),
            initial_pose=Pose.from_xyz(1, 1, 1),
            mass=0.5,
        )

        obj = SimObject.from_params(params)

        assert obj is not None
        assert obj.mass == 0.5

    def test_create_from_params_cylinder(self, pybullet_env):
        """Test creation from params with cylinder shape"""
        params = SimObjectSpawnParams(
            visual_shape=ShapeParams(shape_type="cylinder", radius=0.2, height=0.8, rgba_color=[0, 0, 1, 1]),
            collision_shape=ShapeParams(shape_type="cylinder", radius=0.2, height=0.8),
            initial_pose=Pose.from_xyz(0, 0, 0.5),
            mass=2.0,
        )

        obj = SimObject.from_params(params)

        assert obj is not None
        assert obj.mass == 2.0

    def test_create_from_params_with_name(self, pybullet_env):
        """Test creation with optional name parameter"""
        params = SimObjectSpawnParams(
            visual_shape=ShapeParams(shape_type="box", half_extents=[0.5, 0.5, 0.5]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.5, 0.5, 0.5]),
            initial_pose=Pose.from_xyz(0, 0, 1),
            mass=1.0,
            name="TestObject",
        )

        obj = SimObject.from_params(params)

        assert obj.name == "TestObject"


class TestShapeParams:
    """Test ShapeParams dataclass"""

    def test_shape_params_box(self):
        """Test box shape parameters"""
        shape = ShapeParams(shape_type="box", half_extents=[1.0, 2.0, 3.0])

        assert shape.shape_type == "box"
        assert shape.half_extents == [1.0, 2.0, 3.0]

    def test_shape_params_sphere(self):
        """Test sphere shape parameters"""
        shape = ShapeParams(shape_type="sphere", radius=0.5, rgba_color=[1, 0, 0, 1])

        assert shape.shape_type == "sphere"
        assert shape.radius == 0.5
        assert shape.rgba_color == [1, 0, 0, 1]

    def test_shape_params_cylinder(self):
        """Test cylinder shape parameters"""
        shape = ShapeParams(shape_type="cylinder", radius=0.3, height=1.5)

        assert shape.shape_type == "cylinder"
        assert shape.radius == 0.3
        assert shape.height == 1.5

    def test_shape_params_mesh(self):
        """Test mesh shape parameters"""
        shape = ShapeParams(shape_type="mesh", mesh_path="test.obj", mesh_scale=[2.0, 2.0, 2.0])

        assert shape.shape_type == "mesh"
        assert shape.mesh_path == "test.obj"
        assert shape.mesh_scale == [2.0, 2.0, 2.0]

    def test_shape_params_with_frame_pose(self):
        """Test shape with frame offset"""
        pose = Pose.from_xyz(0.5, 0.5, 0.5)
        shape = ShapeParams(shape_type="box", half_extents=[0.5, 0.5, 0.5], frame_pose=pose)

        assert shape.frame_pose is not None
        assert shape.frame_pose.position[0] == 0.5


class TestSimObjectPose:
    """Test pose get/set operations"""

    def test_get_pose(self, pybullet_env):
        """Test getting object pose"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        obj = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(1, 2, 3),
            mass=1.0,
        )

        pose = obj.get_pose()

        assert pose is not None
        assert abs(pose.position[0] - 1) < 0.01
        assert abs(pose.position[1] - 2) < 0.01
        assert abs(pose.position[2] - 3) < 0.01

    def test_set_pose(self, pybullet_env):
        """Test setting object pose"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        obj = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 1),
            mass=1.0,
        )

        new_pose = Pose.from_xyz(5, 5, 5)
        obj.set_pose(new_pose)  # set_pose doesn't return a value

        current_pose = obj.get_pose()
        assert abs(current_pose.position[0] - 5) < 0.01
        assert abs(current_pose.position[1] - 5) < 0.01
        assert abs(current_pose.position[2] - 5) < 0.01

    def test_set_pose_with_orientation(self, pybullet_env):
        """Test setting pose with specific orientation"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        obj = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 1),
            mass=1.0,
        )

        new_pose = Pose.from_euler(3, 3, 3, roll=0, pitch=0, yaw=1.57)
        obj.set_pose(new_pose)

        current_pose = obj.get_pose()
        assert abs(current_pose.position[0] - 3) < 0.01


class TestSimObjectProperties:
    """Test object properties and methods"""

    def test_is_static_property(self, pybullet_env):
        """Test is_static property"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")

        # Dynamic object (default collision mode is NORMAL_3D)
        dynamic_obj = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 1),
            mass=1.0,
        )
        # is_static checks collision_mode == STATIC, not mass
        assert dynamic_obj.is_static is False

        # Object with STATIC collision mode
        static_obj = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 1),
            mass=0.0,
            collision_mode=CollisionMode.STATIC,
        )
        assert static_obj.is_static is True

    def test_pickable_property(self, pybullet_env):
        """Test pickable property"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")

        # Pickable object
        pickable = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 1),
            mass=1.0,
            pickable=True,
        )
        assert pickable.pickable is True

        # Non-pickable object
        not_pickable = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 1),
            mass=1.0,
            pickable=False,
        )
        assert not_pickable.pickable is False

    def test_mass_property(self, pybullet_env):
        """Test mass property"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")

        obj = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 1),
            mass=3.5,
        )
        assert obj.mass == 3.5

    def test_body_id_property(self, pybullet_env):
        """Test body_id is valid"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")

        obj = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 1),
            mass=1.0,
        )
        assert obj.body_id >= 0

    def test_name_property(self, pybullet_env):
        """Test name property"""
        params = SimObjectSpawnParams(
            visual_shape=ShapeParams(shape_type="box", half_extents=[0.5, 0.5, 0.5]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.5, 0.5, 0.5]),
            initial_pose=Pose.from_xyz(0, 0, 1),
            mass=1.0,
            name="MyObject",
        )

        obj = SimObject.from_params(params)
        assert obj.name == "MyObject"


class TestCollisionMode:
    """Test collision mode settings"""

    def test_collision_mode_normal(self, pybullet_env):
        """Test NORMAL_3D collision mode"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        obj = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 1),
            mass=1.0,
            collision_mode=CollisionMode.NORMAL_3D,
        )

        assert obj.collision_mode == CollisionMode.NORMAL_3D

    def test_collision_mode_disabled(self, pybullet_env):
        """Test DISABLED collision mode"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        obj = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 1),
            mass=1.0,
            collision_mode=CollisionMode.DISABLED,
        )

        assert obj.collision_mode == CollisionMode.DISABLED

    def test_set_collision_mode(self, pybullet_env):
        """Test changing collision mode after creation"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")
        obj = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 1),
            mass=1.0,
        )

        # Change to DISABLED
        obj.set_collision_mode(CollisionMode.DISABLED)
        assert obj.collision_mode == CollisionMode.DISABLED

        # Change to NORMAL_2D
        obj.set_collision_mode(CollisionMode.NORMAL_2D)
        assert obj.collision_mode == CollisionMode.NORMAL_2D


class TestSimObjectSpawnParams:
    """Test SimObjectSpawnParams dataclass"""

    def test_spawn_params_creation(self):
        """Test creating spawn parameters"""
        params = SimObjectSpawnParams(
            visual_shape=ShapeParams(shape_type="box", half_extents=[1, 1, 1]),
            collision_shape=ShapeParams(shape_type="box", half_extents=[1, 1, 1]),
            initial_pose=Pose.from_xyz(0, 0, 0),
            mass=1.0,
            pickable=True,
        )

        assert params.mass == 1.0
        assert params.pickable is True
        assert params.visual_shape.shape_type == "box"
        assert params.collision_shape.shape_type == "box"

    def test_spawn_params_with_name(self):
        """Test spawn params with name"""
        params = SimObjectSpawnParams(
            visual_shape=ShapeParams(shape_type="sphere", radius=0.5),
            collision_shape=ShapeParams(shape_type="sphere", radius=0.5),
            initial_pose=Pose.from_xyz(0, 0, 1),
            mass=2.0,
            name="TestPallet",
        )

        assert params.name == "TestPallet"
        assert params.mass == 2.0

    def test_spawn_params_minimal(self):
        """Test spawn params with minimal configuration"""
        params = SimObjectSpawnParams(
            visual_shape=ShapeParams(shape_type="box"),
            collision_shape=None,  # No collision shape
            initial_pose=Pose.from_xyz(0, 0, 0),
            mass=1.0,
        )

        assert params.visual_shape is not None
        assert params.collision_shape is None
        assert params.pickable is True  # Default value


class TestSimObjectMultiple:
    """Test multiple objects interaction"""

    def test_create_multiple_objects(self, pybullet_env):
        """Test creating multiple objects"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")

        obj1 = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(0, 0, 1),
            mass=1.0,
        )
        obj2 = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(2, 0, 1),
            mass=1.0,
        )
        obj3 = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(4, 0, 1),
            mass=1.0,
        )

        assert obj1.body_id != obj2.body_id
        assert obj2.body_id != obj3.body_id
        assert obj1.body_id != obj3.body_id

    def test_objects_with_different_shapes(self, pybullet_env):
        """Test objects with different shape types"""
        mesh_path = os.path.join(os.path.dirname(__file__), "../mesh/cube.obj")

        box_obj = SimObject.from_params(
            SimObjectSpawnParams(
                visual_shape=ShapeParams(shape_type="box", half_extents=[0.5, 0.5, 0.5]),
                collision_shape=ShapeParams(shape_type="box", half_extents=[0.5, 0.5, 0.5]),
                initial_pose=Pose.from_xyz(0, 0, 1),
                mass=1.0,
            )
        )

        sphere_obj = SimObject.from_params(
            SimObjectSpawnParams(
                visual_shape=ShapeParams(shape_type="sphere", radius=0.5),
                collision_shape=ShapeParams(shape_type="sphere", radius=0.5),
                initial_pose=Pose.from_xyz(2, 0, 1),
                mass=1.0,
            )
        )

        mesh_obj = SimObject.from_mesh(
            visual_shape=ShapeParams(shape_type="mesh", mesh_path=mesh_path),
            collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
            pose=Pose.from_xyz(4, 0, 1),
            mass=1.0,
        )

        assert box_obj.body_id >= 0
        assert sphere_obj.body_id >= 0
        assert mesh_obj.body_id >= 0


class TestSimObjectVisualOnly:
    """Test SimObject with visual-only or collision-only shapes"""

    def test_visual_only_object(self, pybullet_env):
        """Test creating object with visual shape but no collision"""
        obj = SimObject.from_params(
            SimObjectSpawnParams(
                visual_shape=ShapeParams(shape_type="box", half_extents=[0.5, 0.5, 0.5]),
                collision_shape=None,  # No collision
                initial_pose=Pose.from_xyz(0, 0, 1),
                mass=1.0,
            )
        )

        assert obj.body_id >= 0
        assert obj.mass == 1.0

    def test_collision_only_object(self, pybullet_env):
        """Test creating object with collision shape but no visual"""
        obj = SimObject.from_params(
            SimObjectSpawnParams(
                visual_shape=None,  # No visual
                collision_shape=ShapeParams(shape_type="box", half_extents=[0.5, 0.5, 0.5]),
                initial_pose=Pose.from_xyz(0, 0, 1),
                mass=1.0,
            )
        )

        assert obj.body_id >= 0
        assert obj.mass == 1.0


class TestSimObjectKinematic:
    """Test kinematic (mass=0) object behavior"""

    def test_kinematic_flag(self, pybullet_env):
        """Test is_kinematic flag is set correctly"""
        # Kinematic object (mass=0)
        kinematic_obj = SimObject.from_params(
            SimObjectSpawnParams(
                visual_shape=ShapeParams(shape_type="box", half_extents=[0.5, 0.5, 0.5]),
                collision_shape=ShapeParams(shape_type="box", half_extents=[0.5, 0.5, 0.5]),
                initial_pose=Pose.from_xyz(0, 0, 1),
                mass=0.0,
            )
        )

        assert kinematic_obj.is_kinematic is True
        assert kinematic_obj.mass == 0.0

        # Dynamic object (mass>0)
        dynamic_obj = SimObject.from_params(
            SimObjectSpawnParams(
                visual_shape=ShapeParams(shape_type="box", half_extents=[0.5, 0.5, 0.5]),
                collision_shape=ShapeParams(shape_type="box", half_extents=[0.5, 0.5, 0.5]),
                initial_pose=Pose.from_xyz(2, 0, 1),
                mass=1.5,
            )
        )

        assert dynamic_obj.is_kinematic is False
        assert dynamic_obj.mass == 1.5
