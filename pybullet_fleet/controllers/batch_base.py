"""Shared base class for vectorized (batched) kinematic controllers.

A BatchKinematicController manages **N agents at once** using NumPy arrays for
per-agent state, replacing the per-agent Python dispatch that
``KinematicController.compute()`` performs in the default path.

Lifecycle
---------
1. User constructs the controller (no agents yet).
2. User registers each agent via ``c.register_agent(agent)``. The first
   registration auto-binds the controller to the agent's ``sim_core`` and
   appends it to ``sim_core._batch_controllers`` so ``step_once()`` will
   drive it. All subsequent agents must belong to the same sim_core.
3. Each sim step, ``sim_core.step_once()`` calls ``c.batch_advance(dt)``
   during Phase 1, which computes new poses for all registered agents and
   writes them via the buffered batch API ``sim_core.set_poses()``.
4. Agents may be unregistered at runtime. When the last agent leaves, the
   controller auto-detaches from ``sim_core._batch_controllers``.

Subclass responsibilities
-------------------------
Concrete subclasses implement ``batch_advance(dt)`` and any controller-specific
state (e.g. trajectory parameters). Subclasses should call
``self._resize_state(n)`` from ``_on_agents_changed`` to keep their own arrays
sized with ``self._agents``.
"""

from __future__ import annotations

from abc import abstractmethod
from typing import TYPE_CHECKING, Dict, List, Optional

import numpy as np

from pybullet_fleet.controller import Controller
from pybullet_fleet.logging_utils import get_lazy_logger

if TYPE_CHECKING:
    from pybullet_fleet.agent import Agent
    from pybullet_fleet.core_simulation import MultiRobotSimulationCore

logger = get_lazy_logger(__name__)


class BatchKinematicController(Controller):
    """Vectorized base for kinematic controllers that drive many agents at once.

    Inherits from :class:`Controller` directly (not ``KinematicController``):
    the per-agent ``compute(agent, dt) -> bool`` contract does not apply here
    since batch controllers act through :meth:`batch_advance` during Phase 1
    of ``sim_core.step_once()``. Per-agent kinematic parameters (max_vel,
    accel, etc.) are read from each ``Agent`` at ``set_path`` time.
    """

    def __init__(
        self,
        navigation_2d: bool = False,
    ) -> None:
        self._navigation_2d: bool = navigation_2d

        self._sim_core: Optional["MultiRobotSimulationCore"] = None
        self._agents: List["Agent"] = []
        # id(agent) -> row index. Stable for the lifetime of an agent in this
        # controller; on unregister we compact rows by swapping with the last
        # row, so other agents may see their index change.
        self._agent_index: Dict[int, int] = {}
        # id(agent) -> prior _needs_update flag (so unregister can restore).
        self._saved_needs_update: Dict[int, bool] = {}

        # Per-agent output buffers (filled by batch_advance, written to sim).
        self._object_ids: np.ndarray = np.zeros((0,), dtype=np.int64)
        self._pos_buf: np.ndarray = np.zeros((0, 3), dtype=np.float64)
        self._orn_buf: np.ndarray = np.zeros((0, 4), dtype=np.float64)
        self._moved_mask: np.ndarray = np.zeros((0,), dtype=bool)

    # ------------------------------------------------------------------ #
    # Lifecycle: agent registration (sim_core attachment is implicit)
    # ------------------------------------------------------------------ #

    def register_agent(self, agent: "Agent") -> int:
        """Register *agent* with this batch controller.

        On the first call, the controller auto-binds to ``agent.sim_core`` and
        appends itself to ``sim_core._batch_controllers`` so the sim's
        ``step_once()`` will drive it during Phase 1. Subsequent registrations
        must use agents from the same sim_core.

        Returns:
            The agent's row index in this controller's state arrays.

        Raises:
            ValueError: If the agent is already registered, has no sim_core,
                or belongs to a different sim_core than previously-registered
                agents.
        """
        key = id(agent)
        if key in self._agent_index:
            raise ValueError(f"Agent {agent} is already registered with this batch controller.")
        agent_sim = getattr(agent, "sim_core", None)
        if agent_sim is None:
            raise ValueError(f"Agent {agent} has no sim_core; spawn it via Agent.from_params(..., sim_core) first.")
        if self._sim_core is None:
            self._sim_core = agent_sim
            if self not in agent_sim._batch_controllers:
                agent_sim._batch_controllers.append(self)
        elif agent_sim is not self._sim_core:
            raise ValueError("Agent belongs to a different sim_core than this batch controller's other agents.")
        idx = len(self._agents)
        self._agents.append(agent)
        self._agent_index[key] = idx
        self._resize_base_buffers(len(self._agents))
        self._object_ids[idx] = agent.object_id
        # Initialize buffer with current pose so a no-op step doesn't move the agent.
        pose = agent.get_pose()
        self._pos_buf[idx] = pose.position
        self._orn_buf[idx] = pose.orientation
        # Suppress the per-step Agent.update() loop for this agent — the batch
        # controller drives it directly via batch_advance(). Save the prior
        # value so unregister can restore it.
        self._saved_needs_update[key] = getattr(agent, "_needs_update", True)
        agent._needs_update = False
        self._on_agent_registered(idx, agent)
        return idx

    def unregister_agent(self, agent: "Agent") -> None:
        """Unregister *agent*. Compacts the state arrays by swapping with last row.

        Raises:
            KeyError: If the agent is not registered.
        """
        key = id(agent)
        if key not in self._agent_index:
            raise KeyError(f"Agent {agent} is not registered with this batch controller.")
        idx = self._agent_index.pop(key)
        prior = self._saved_needs_update.pop(key, True)
        agent._needs_update = prior
        last = len(self._agents) - 1
        if idx != last:
            # Swap last row into the freed slot.
            self._agents[idx] = self._agents[last]
            self._agent_index[id(self._agents[idx])] = idx
            self._swap_rows(idx, last)
        self._agents.pop()
        self._resize_base_buffers(len(self._agents))
        self._on_agent_unregistered(idx, agent)
        # Auto-detach when no agents remain so sim_core stops driving this
        # controller and a fresh `register_agent(other_sim_agent)` could bind
        # it to a different sim_core.
        if not self._agents and self._sim_core is not None:
            sim = self._sim_core
            self._sim_core = None
            try:
                sim._batch_controllers.remove(self)
            except ValueError:
                pass

    def reset(self) -> None:
        """Unregister all agents and reset arrays. Auto-detaches from sim_core
        once the last agent is removed (same behaviour as ``unregister_agent``)."""
        # Snapshot before mutation
        agents = list(self._agents)
        for a in agents:
            self.unregister_agent(a)

    # ------------------------------------------------------------------ #
    # Buffer management (called by lifecycle methods above)
    # ------------------------------------------------------------------ #

    def _resize_base_buffers(self, n: int) -> None:
        """Resize the (N, ...) buffers owned by this base class."""
        self._object_ids = self._resize_rows(self._object_ids, n)
        self._pos_buf = self._resize_rows(self._pos_buf, n)
        self._orn_buf = self._resize_rows(self._orn_buf, n)
        self._moved_mask = self._resize_rows(self._moved_mask, n)

    @staticmethod
    def _resize_rows(arr: np.ndarray, n: int) -> np.ndarray:
        """Resize ``arr`` along axis 0 to length ``n``, preserving existing rows
        and zero-padding new rows.

        Unlike :func:`numpy.resize`, this does **not** wrap around: new rows
        are always zero-initialised. Trailing dims (if any) are preserved.
        """
        if arr.shape[0] == n:
            return arr
        new_shape = (n,) + arr.shape[1:]
        new_arr = np.zeros(new_shape, dtype=arr.dtype)
        copy_n = min(arr.shape[0], n)
        if copy_n > 0:
            new_arr[:copy_n] = arr[:copy_n]
        return new_arr

    def _swap_rows(self, i: int, j: int) -> None:
        """Swap rows ``i`` and ``j`` in all base buffers. Subclasses override
        to also swap their own per-agent arrays.
        """
        self._object_ids[[i, j]] = self._object_ids[[j, i]]
        self._pos_buf[[i, j]] = self._pos_buf[[j, i]]
        self._orn_buf[[i, j]] = self._orn_buf[[j, i]]
        self._moved_mask[[i, j]] = self._moved_mask[[j, i]]

    def _on_agent_registered(self, idx: int, agent: "Agent") -> None:
        """Hook for subclasses to initialise their own per-agent state row."""

    def _on_agent_unregistered(self, idx: int, agent: "Agent") -> None:
        """Hook for subclasses to clear their own per-agent state row."""

    # ------------------------------------------------------------------ #
    # Phase 1 entry point (called by core_simulation.step_once)
    # ------------------------------------------------------------------ #

    @abstractmethod
    def batch_advance(self, dt: float) -> np.ndarray:
        """Compute one timestep for all registered agents.

        Implementations must:

        1. Fill ``self._pos_buf`` and ``self._orn_buf`` with new poses for all
           agents.
        2. Fill ``self._moved_mask`` with True for agents whose pose changed.
        3. Call ``self._apply_phase1()`` to write the moved rows back to sim.

        Returns:
            ``self._moved_mask`` — a (N,) boolean array.
        """

    def _apply_phase1(self) -> None:
        """Write the rows flagged in ``self._moved_mask`` back to sim via the
        buffered batch API ``sim_core.set_poses``.

        No-op when no agents moved or no sim_core attached.
        """
        if self._sim_core is None or len(self._agents) == 0:
            return
        if not bool(self._moved_mask.any()):
            return
        # set_poses accepts a Sequence[int] for object_ids; passing a NumPy
        # boolean-indexed view of int64 ids is cheap (no Python loop).
        ids = self._object_ids[self._moved_mask].tolist()
        positions = self._pos_buf[self._moved_mask]
        orientations = self._orn_buf[self._moved_mask]
        self._sim_core.set_poses(ids, positions, orientations)

    # ------------------------------------------------------------------ #
    # Controller ABC contract — batch controllers act through batch_advance()
    # during Phase 1 of step_once(), not per-agent compute(). The compute()
    # stub is required because Controller.compute is abstract, and is also a
    # safety net in case Agent.update() is ever called on a registered agent
    # (it should be dormant because register_agent sets _needs_update=False).
    # ------------------------------------------------------------------ #

    def compute(self, agent, dt: float) -> bool:
        return False
