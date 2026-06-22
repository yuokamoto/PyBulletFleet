"""Tests for ROS ↔ PyBulletFleet type conversions.

These tests require ROS 2 message types and are designed to run inside Docker
(``colcon test``) or with a sourced ROS workspace.
"""

import math

import pytest

# Skip entire module if ROS message types are not available
ros_msgs = pytest.importorskip("geometry_msgs.msg", reason="ROS 2 messages not available")


def test_pbf_pose_to_ros_pose():
    """PyBulletFleet Pose → geometry_msgs/Pose round-trip."""
    from pybullet_fleet.geometry import Pose
    from pybullet_fleet_ros.conversions import pbf_pose_to_ros, ros_pose_to_pbf

    pbf = Pose.from_xyz(1.0, 2.0, 3.0)
    ros_msg = pbf_pose_to_ros(pbf)
    assert ros_msg.position.x == pytest.approx(1.0)
    assert ros_msg.position.y == pytest.approx(2.0)
    assert ros_msg.position.z == pytest.approx(3.0)
    assert ros_msg.orientation.w == pytest.approx(1.0)

    back = ros_pose_to_pbf(ros_msg)
    assert back.x == pytest.approx(1.0)
    assert back.y == pytest.approx(2.0)
    assert back.z == pytest.approx(3.0)


def test_ros_pose_to_pbf_with_orientation():
    """geometry_msgs/Pose with non-identity orientation."""
    from geometry_msgs.msg import Pose as PoseMsg, Quaternion, Point
    from pybullet_fleet_ros.conversions import ros_pose_to_pbf

    msg = PoseMsg()
    msg.position = Point(x=5.0, y=6.0, z=0.0)
    msg.orientation = Quaternion(x=0.0, y=0.0, z=0.7071068, w=0.7071068)

    pbf = ros_pose_to_pbf(msg)
    assert pbf.x == pytest.approx(5.0)
    assert pbf.yaw == pytest.approx(math.pi / 2, abs=0.01)


def test_pbf_pose_to_odom():
    """PyBulletFleet Pose + velocity → nav_msgs/Odometry."""
    from pybullet_fleet.geometry import Pose
    from pybullet_fleet_ros.conversions import make_odom_msg

    pose = Pose.from_xyz(1.0, 2.0, 0.0)
    velocity = [0.5, 0.0, 0.0]
    angular_vel = 0.1

    msg = make_odom_msg(pose, velocity, angular_vel, "odom", "robot_0/base_link")
    assert msg.pose.pose.position.x == pytest.approx(1.0)
    assert msg.twist.twist.linear.x == pytest.approx(0.5)
    assert msg.twist.twist.angular.z == pytest.approx(0.1)
    assert msg.header.frame_id == "odom"
    assert msg.child_frame_id == "robot_0/base_link"


def test_joint_state_msg():
    """Agent joint states → sensor_msgs/JointState."""
    from pybullet_fleet_ros.conversions import make_joint_state_msg

    joint_names = ["joint_0", "joint_1", "joint_2"]
    positions = [0.1, 0.2, 0.3]
    velocities = [0.01, 0.02, 0.03]

    msg = make_joint_state_msg(joint_names, positions, velocities)
    assert list(msg.name) == joint_names
    assert list(msg.position) == pytest.approx(positions)
    assert list(msg.velocity) == pytest.approx(velocities)


def test_twist_to_velocity_world_frame():
    """Twist (body frame) → world-frame velocity conversion."""
    from geometry_msgs.msg import Twist
    from pybullet_fleet_ros.conversions import twist_to_world_velocity

    twist = Twist()
    twist.linear.x = 1.0
    twist.linear.y = 0.5
    twist.linear.z = 0.0
    twist.angular.z = 0.2

    # Robot facing +X (yaw=0)
    vx, vy, vz, wz = twist_to_world_velocity(twist, current_yaw=0.0)
    assert vx == pytest.approx(1.0)
    assert vy == pytest.approx(0.5)
    assert vz == pytest.approx(0.0)
    assert wz == pytest.approx(0.2)


def test_twist_to_velocity_rotated():
    """Twist → world velocity when robot is rotated 90 degrees."""
    from geometry_msgs.msg import Twist
    from pybullet_fleet_ros.conversions import twist_to_world_velocity

    twist = Twist()
    twist.linear.x = 1.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.z = 0.0

    # Robot facing +Y (yaw=90°)
    vx, vy, vz, wz = twist_to_world_velocity(twist, current_yaw=math.pi / 2)
    assert vx == pytest.approx(0.0, abs=1e-6)
    assert vy == pytest.approx(1.0, abs=1e-6)
    assert vz == pytest.approx(0.0)


def test_twist_to_velocity_with_z():
    """Twist with linear.z is passed through to world frame."""
    from geometry_msgs.msg import Twist
    from pybullet_fleet_ros.conversions import twist_to_world_velocity

    twist = Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.5
    twist.angular.z = 0.0

    vx, vy, vz, wz = twist_to_world_velocity(twist, current_yaw=0.0)
    assert vz == pytest.approx(0.5)


def test_sim_time_to_ros_time():
    """Simulation time float → builtin_interfaces/Time."""
    from pybullet_fleet_ros.conversions import sim_time_to_ros_time

    stamp = sim_time_to_ros_time(1.5)
    assert stamp.sec == 1
    assert stamp.nanosec == 500000000
