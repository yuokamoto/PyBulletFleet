"""Per-robot ROS interface handler.

Creates publishers and subscribers for a single Agent.
Passes body-frame cmd_vel directly to the VelocityController's set_velocity().
The controller handles kinematics (body→world rotation) internally.
"""

import logging
from typing import TYPE_CHECKING, Optional

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from .conversions import (
    make_joint_state_msg,
    make_odom_msg,
    make_transform_stamped,
)

if TYPE_CHECKING:
    from pybullet_fleet.agent import Agent
    from pybullet_fleet.controller import VelocityController
    from rclpy.node import Node
    from tf2_ros import TransformBroadcaster

logger = logging.getLogger(__name__)


class RobotHandler:
    """Manages all ROS 2 interfaces for a single simulated Agent.

    Creates:
    - ``/{name}/cmd_vel`` subscriber (Twist) → VelocityController.set_velocity()
    - ``/{name}/odom`` publisher (Odometry)
    - ``/{name}/joint_states`` publisher (JointState)
    - TF broadcast: ``odom`` → ``{name}/base_link``
    """

    def __init__(
        self,
        node: "Node",
        agent: "Agent",
        vel_controller: Optional["VelocityController"] = None,
        tf_broadcaster: Optional["TransformBroadcaster"] = None,
    ):
        self._node = node
        self.agent = agent
        self._vel_controller = vel_controller
        self._tf_broadcaster = tf_broadcaster
        self._latest_twist: Optional[Twist] = None

        ns = agent.name or f"robot{agent.object_id}"
        self._ns = ns

        # Publishers
        self._odom_pub = node.create_publisher(Odometry, f"/{ns}/odom", 10)
        self._joint_pub = node.create_publisher(JointState, f"/{ns}/joint_states", 10)

        # Subscribers
        self._cmd_vel_sub = node.create_subscription(Twist, f"/{ns}/cmd_vel", self._cmd_vel_cb, 10)

        logger.info("RobotHandler created for '%s' (object_id=%d)", ns, agent.object_id)

    def _cmd_vel_cb(self, msg: Twist) -> None:
        """Store latest cmd_vel for application in next step."""
        self._latest_twist = msg

    def apply_cmd_vel(self) -> None:
        """Apply stored cmd_vel as a velocity command via VelocityController.

        Called once per sim step by BridgeNode._step_callback().
        Passes body-frame Twist directly to the controller's set_velocity().
        The controller handles kinematics (body→world rotation) internally.
        """
        if self._latest_twist is None:
            return
        if self._vel_controller is None:
            logger.warning("RobotHandler '%s': no controller set, ignoring cmd_vel", self._ns)
            self._latest_twist = None
            return

        twist = self._latest_twist
        # Pass body-frame Twist directly — controller handles kinematics
        self._vel_controller.set_velocity(
            vx=twist.linear.x,
            vy=twist.linear.y,
            vz=twist.linear.z,
            wx=twist.angular.x,
            wy=twist.angular.y,
            wz=twist.angular.z,
        )

    def publish_state(self, stamp: TimeMsg) -> None:
        """Publish odom, joint_states, and TF for this agent."""
        pose = self.agent.get_pose()
        velocity = list(self.agent.velocity)
        angular_vel = self.agent.angular_velocity

        # Odometry
        odom_msg = make_odom_msg(
            pose,
            velocity,
            angular_vel,
            frame_id="odom",
            child_frame_id=f"{self._ns}/base_link",
            stamp=stamp,
        )
        self._odom_pub.publish(odom_msg)

        # TF: odom → {ns}/base_link
        if self._tf_broadcaster is not None:
            tf_msg = make_transform_stamped(
                pose,
                parent_frame="odom",
                child_frame=f"{self._ns}/base_link",
                stamp=stamp,
            )
            self._tf_broadcaster.sendTransform(tf_msg)

        # Joint states (only if robot has joints)
        num_joints = self.agent.get_num_joints()
        if num_joints > 0:
            self._publish_joint_states(stamp)

    def _publish_joint_states(self, stamp: TimeMsg) -> None:
        """Publish sensor_msgs/JointState for all joints."""
        joints_by_name = self.agent.get_all_joints_state_by_name()
        if not joints_by_name:
            return

        names = list(joints_by_name.keys())
        positions = [joints_by_name[n][0] for n in names]
        velocities = [joints_by_name[n][1] for n in names]

        msg = make_joint_state_msg(names, positions, velocities, stamp=stamp)
        self._joint_pub.publish(msg)

    def destroy(self) -> None:
        """Clean up ROS interfaces."""
        self._node.destroy_publisher(self._odom_pub)
        self._node.destroy_publisher(self._joint_pub)
        self._node.destroy_subscription(self._cmd_vel_sub)
        logger.info("RobotHandler destroyed for '%s'", self._ns)
