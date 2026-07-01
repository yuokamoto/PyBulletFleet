"""FleetHandler — Pattern 2 (Batch API ROS Wrapper).

A single, simulation-level handler that exposes the whole fleet through O(1) ROS
endpoints instead of O(N) per-robot topics:

- ``/fleet/states`` (FleetState) — one aggregated message with every robot's
  pose/velocity/moving flag, published on every simulation POST_STEP event.
- ``/fleet/navigate`` — batch navigation goals, offered both ways over the same
  shared logic:
    * service  (FleetNavigate)  — apply goals, return how many matched (with ack);
    * topic    (FleetGoal)      — fire-and-forget for high-rate goal streaming.

Selected with ``fleet_interface: batch`` in the bridge config (default
``per_robot`` keeps the existing per-robot RobotHandler interface). Pairs well
with a ``batch_controller`` (batch_omni / batch_differential) for large fleets.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

from pybullet_fleet.geometry import Pose
from pybullet_fleet.logging_utils import get_lazy_logger
from pybullet_fleet_msgs.msg import FleetGoal, FleetState, RobotState
from pybullet_fleet_msgs.srv import FleetNavigate
from std_msgs.msg import Header

if TYPE_CHECKING:
    from rclpy.node import Node

    from pybullet_fleet.core_simulation import MultiRobotSimulationCore

logger = get_lazy_logger(__name__)


class FleetHandler:
    """Fleet-wide ROS interface (Pattern 2). One per bridge, not per robot."""

    def __init__(self, node: "Node", sim_core: "MultiRobotSimulationCore"):
        self._node = node
        self._sim = sim_core
        off = getattr(node, "rmf_frame_offset", (0.0, 0.0))
        self._rmf_offset = (float(off[0]), float(off[1]))

        self._states_pub = node.create_publisher(FleetState, "/fleet/states", 10)
        self._goal_sub = node.create_subscription(FleetGoal, "/fleet/navigate", self._on_fleet_goal, 10)
        self._nav_srv = node.create_service(FleetNavigate, "/fleet/navigate", self._on_fleet_navigate)
        logger.info("FleetHandler up: /fleet/states publisher + /fleet/navigate service & topic")

    # -- goal injection (shared by the service and the topic) -----------------

    def _shift_xy(self, x: float, y: float, sign: int) -> tuple:
        """Apply the sim/RMF frame offset to a planar position.

        ``sign=+1`` converts sim->RMF for published state, while ``sign=-1``
        converts RMF->sim for received goals.
        """
        return x + sign * self._rmf_offset[0], y + sign * self._rmf_offset[1]

    def _set_fleet_goals(self, goals) -> int:
        """Apply each goal to the robot whose name matches; return #applied."""
        by_name = {a.name: a for a in self._sim.agents}
        applied = 0
        for g in goals:
            agent = by_name.get(g.name)
            if agent is None:
                logger.warning("fleet navigate: unknown robot %r — skipped", g.name)
                continue
            sim_x, sim_y = self._shift_xy(float(g.x), float(g.y), -1)
            current_pose = agent.get_pose()
            agent.set_goal_pose(Pose.from_yaw(sim_x, sim_y, current_pose.z, float(g.yaw)))
            applied += 1
        return applied

    def _on_fleet_goal(self, msg: FleetGoal) -> None:
        n = self._set_fleet_goals(msg.goals)
        if n == len(msg.goals):
            logger.debug("fleet navigate (topic): %d/%d goals applied", n, len(msg.goals))
        else:
            logger.warning("fleet navigate (topic): only %d/%d goals applied", n, len(msg.goals))

    def _on_fleet_navigate(self, request, response):
        n = self._set_fleet_goals(request.goals)
        response.accepted = True
        response.num_set = n
        response.message = f"applied {n}/{len(request.goals)} goals"
        logger.info("fleet navigate (service): %s", response.message)
        return response

    # -- state aggregation ----------------------------------------------------

    def post_step(self, dt: float, stamp) -> None:
        """Publish /fleet/states — one aggregated message, every post_step (like
        the per-robot handlers)."""
        msg = FleetState(header=Header(stamp=stamp, frame_id="map"))
        for agent in self._sim.agents:
            pose = agent.get_pose()
            vel = agent.velocity  # world-frame [vx, vy, vz]
            rmf_x, rmf_y = self._shift_xy(float(pose.x), float(pose.y), +1)
            msg.robots.append(
                RobotState(
                    name=agent.name,
                    x=rmf_x,
                    y=rmf_y,
                    yaw=float(pose.yaw),
                    vx=float(vel[0]),
                    vy=float(vel[1]),
                    vyaw=float(agent.angular_velocity),
                    moving=bool(agent.is_moving),
                    battery_soc=float(agent.battery_soc),  # 1.0 when no battery plugin
                    charging=bool(agent.is_charging),
                )
            )
        self._states_pub.publish(msg)

    def destroy(self) -> None:
        self._node.destroy_publisher(self._states_pub)
        self._node.destroy_subscription(self._goal_sub)
        self._node.destroy_service(self._nav_srv)