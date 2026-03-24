"""Conversion utilities between PyBulletFleet types and ROS 2 messages.

All functions handle PyBulletFleet ↔ ROS 2 type conversions.
Body-frame → world-frame rotation (``twist_to_world_velocity``) delegates to
:func:`pybullet_fleet.tools.body_to_world_velocity_2d` (pure math, no ROS deps).
"""

from typing import List, Optional, Tuple

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import (
    Point,
    Pose as PoseMsg,
    Quaternion,
    TransformStamped,
    Twist,
    Vector3,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from pybullet_fleet.geometry import Pose


def pbf_pose_to_ros(pbf_pose: Pose) -> PoseMsg:
    """Convert PyBulletFleet Pose → geometry_msgs/Pose."""
    msg = PoseMsg()
    msg.position = Point(
        x=float(pbf_pose.position[0]),
        y=float(pbf_pose.position[1]),
        z=float(pbf_pose.position[2]),
    )
    msg.orientation = Quaternion(
        x=float(pbf_pose.orientation[0]),
        y=float(pbf_pose.orientation[1]),
        z=float(pbf_pose.orientation[2]),
        w=float(pbf_pose.orientation[3]),
    )
    return msg


def ros_pose_to_pbf(ros_pose: PoseMsg) -> Pose:
    """Convert geometry_msgs/Pose → PyBulletFleet Pose."""
    return Pose(
        position=[ros_pose.position.x, ros_pose.position.y, ros_pose.position.z],
        orientation=[
            ros_pose.orientation.x,
            ros_pose.orientation.y,
            ros_pose.orientation.z,
            ros_pose.orientation.w,
        ],
    )


def make_odom_msg(
    pose: Pose,
    velocity: List[float],
    angular_velocity: float,
    frame_id: str,
    child_frame_id: str,
    stamp: Optional[TimeMsg] = None,
) -> Odometry:
    """Create nav_msgs/Odometry from PyBulletFleet state."""
    msg = Odometry()
    msg.header = Header(frame_id=frame_id)
    if stamp:
        msg.header.stamp = stamp
    msg.child_frame_id = child_frame_id
    msg.pose.pose = pbf_pose_to_ros(pose)
    msg.twist.twist.linear = Vector3(
        x=float(velocity[0]),
        y=float(velocity[1]),
        z=float(velocity[2]),
    )
    msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=float(angular_velocity))
    return msg


def make_joint_state_msg(
    joint_names: List[str],
    positions: List[float],
    velocities: List[float],
    stamp: Optional[TimeMsg] = None,
) -> JointState:
    """Create sensor_msgs/JointState."""
    msg = JointState()
    if stamp:
        msg.header.stamp = stamp
    msg.name = joint_names
    msg.position = [float(p) for p in positions]
    msg.velocity = [float(v) for v in velocities]
    return msg


def make_transform_stamped(
    pose: Pose,
    parent_frame: str,
    child_frame: str,
    stamp: Optional[TimeMsg] = None,
) -> TransformStamped:
    """Create geometry_msgs/TransformStamped from Pose."""
    t = TransformStamped()
    t.header = Header(frame_id=parent_frame)
    if stamp:
        t.header.stamp = stamp
    t.child_frame_id = child_frame
    t.transform.translation = Vector3(
        x=float(pose.position[0]),
        y=float(pose.position[1]),
        z=float(pose.position[2]),
    )
    t.transform.rotation = Quaternion(
        x=float(pose.orientation[0]),
        y=float(pose.orientation[1]),
        z=float(pose.orientation[2]),
        w=float(pose.orientation[3]),
    )
    return t


def twist_to_world_velocity(
    twist: "Twist",
    current_yaw: float,
) -> Tuple[float, float, float, float]:
    """Convert Twist (body frame) to world-frame velocity (vx, vy, vz, wz).

    Thin wrapper around :func:`pybullet_fleet.tools.body_to_world_velocity_2d`
    that accepts a ``geometry_msgs/Twist`` message directly.

    Args:
        twist: geometry_msgs/Twist message (body-frame velocities)
        current_yaw: Current robot yaw in world frame (radians)

    Returns:
        (vx, vy, vz, wz) in world frame
    """
    from pybullet_fleet.tools import body_to_world_velocity_2d

    vx, vy = body_to_world_velocity_2d(
        vx_body=twist.linear.x,
        vy_body=twist.linear.y,
        yaw=current_yaw,
    )
    return vx, vy, twist.linear.z, twist.angular.z


def sim_time_to_ros_time(sim_time: float) -> TimeMsg:
    """Convert simulation time (float seconds) to builtin_interfaces/Time."""
    sec = int(sim_time)
    nanosec = int((sim_time - sec) * 1e9)
    return TimeMsg(sec=sec, nanosec=nanosec)
