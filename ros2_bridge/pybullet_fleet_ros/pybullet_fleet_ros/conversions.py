"""Conversion utilities between PyBulletFleet types and ROS 2 messages.

All functions handle PyBulletFleet ↔ ROS 2 type conversions.
Body-frame → world-frame rotation (``twist_to_world_velocity``) delegates to
:func:`pybullet_fleet.tools.body_to_world_velocity_2d` (pure math, no ROS deps).
"""

from typing import List, Optional, Tuple

from builtin_interfaces.msg import Time as TimeMsg
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import (
    Point,
    Pose as PoseMsg,
    PoseStamped,
    Quaternion,
    TransformStamped,
    Twist,
    Vector3,
)
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory

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


# ---------------------------------------------------------------------------
# Nav / Arm / Diagnostic conversion helpers
# ---------------------------------------------------------------------------


def pbf_pose_to_pose_stamped(
    pbf_pose: Pose,
    frame_id: str = "odom",
    stamp: Optional[TimeMsg] = None,
) -> PoseStamped:
    """Convert PyBulletFleet Pose → geometry_msgs/PoseStamped."""
    msg = PoseStamped()
    msg.header = Header(frame_id=frame_id)
    if stamp:
        msg.header.stamp = stamp
    msg.pose = pbf_pose_to_ros(pbf_pose)
    return msg


def ros_pose_stamped_to_pbf(msg: PoseStamped) -> Pose:
    """Convert geometry_msgs/PoseStamped → PyBulletFleet Pose."""
    return ros_pose_to_pbf(msg.pose)


def pbf_path_to_ros(
    waypoints: List[Pose],
    frame_id: str = "odom",
    stamp: Optional[TimeMsg] = None,
) -> PathMsg:
    """Convert list of PyBulletFleet Pose → nav_msgs/Path."""
    msg = PathMsg()
    msg.header = Header(frame_id=frame_id)
    if stamp:
        msg.header.stamp = stamp
    for wp in waypoints:
        msg.poses.append(pbf_pose_to_pose_stamped(wp, frame_id=frame_id, stamp=stamp))
    return msg


def ros_path_to_pbf(msg: PathMsg) -> List[Pose]:
    """Convert nav_msgs/Path → list of PyBulletFleet Pose."""
    return [ros_pose_stamped_to_pbf(ps) for ps in msg.poses]


def joint_trajectory_to_targets(msg: JointTrajectory) -> dict:
    """Extract final waypoint from JointTrajectory as {name: position} dict.

    Takes the last trajectory point's positions and maps them to joint names.
    """
    if not msg.points:
        return {}
    last_point = msg.points[-1]
    targets: dict = {}
    for i, name in enumerate(msg.joint_names):
        if i < len(last_point.positions):
            targets[name] = last_point.positions[i]
    return targets


def make_diagnostic_msg(
    robot_name: str,
    is_moving: bool,
    action_type: str = "",
    action_status: str = "",
    distance_to_goal: float = 0.0,
    linear_speed: float = 0.0,
    action_queue: str = "",
    action_queue_size: int = 0,
    stamp: Optional[TimeMsg] = None,
) -> DiagnosticArray:
    """Create diagnostic_msgs/DiagnosticArray with robot status."""
    status = DiagnosticStatus()
    status.name = robot_name
    status.level = DiagnosticStatus.OK
    status.message = f"{action_type} {action_status}" if action_type else ("moving" if is_moving else "idle")
    status.values = [
        KeyValue(key="is_moving", value=str(is_moving).lower()),
        KeyValue(key="action_type", value=action_type),
        KeyValue(key="action_status", value=action_status),
        KeyValue(key="distance_to_goal", value=f"{distance_to_goal:.3f}"),
        KeyValue(key="linear_speed", value=f"{linear_speed:.3f}"),
        KeyValue(key="action_queue", value=action_queue),
        KeyValue(key="action_queue_size", value=str(action_queue_size)),
    ]
    msg = DiagnosticArray()
    if stamp:
        msg.header.stamp = stamp
    msg.status = [status]
    return msg
