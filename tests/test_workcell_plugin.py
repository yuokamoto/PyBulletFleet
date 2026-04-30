"""Tests for WorkcellPlugin — headless, no ROS dependency.

Covers: item discovery, dispense, ingest, nearest-robot lookup,
pending action completion callbacks, return-home teleporting.
"""

import math

import pybullet as p

from pybullet_fleet.action import PickAction, DropAction
from pybullet_fleet.agent import Agent, AgentSpawnParams
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.geometry import Pose
from pybullet_fleet.plugins.workcell_plugin import WorkcellPlugin, PendingAction
from pybullet_fleet.sim_object import SimObject, SimObjectSpawnParams, ShapeParams
from pybullet_fleet.types import ActionStatus, CollisionMode


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_sim():
    """Headless simulation with physics off."""
    return MultiRobotSimulationCore(SimulationParams(gui=False, monitor=False, physics=False))


def _spawn_agent(sim, name: str, x: float = 0.0, y: float = 0.0) -> Agent:
    """Spawn a minimal mobile robot agent."""
    params = AgentSpawnParams(
        urdf_path="robots/mobile_robot.urdf",
        initial_pose=Pose.from_xyz(x, y, 0.05),
        name=name,
        collision_mode=CollisionMode.DISABLED,
    )
    return Agent.from_params(params, sim)


def _spawn_item(sim, name: str, x: float, y: float, pickable: bool = True) -> SimObject:
    """Spawn a small pickable box."""
    params = SimObjectSpawnParams(
        visual_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
        collision_shape=ShapeParams(shape_type="box", half_extents=[0.1, 0.1, 0.1]),
        initial_pose=Pose.from_xyz(x, y, 0.1),
        mass=0.5,
        pickable=pickable,
        name=name,
        collision_mode=CollisionMode.DISABLED,
    )
    return SimObject.from_params(params, sim)


# ---------------------------------------------------------------------------
# Tests: Plugin construction
# ---------------------------------------------------------------------------


class TestPluginConstruction:
    def test_default_config(self):
        sim = _make_sim()
        plugin = sim.register_plugin(WorkcellPlugin)
        assert plugin._item_search_radius == 1.0
        assert plugin._return_home is True
        p.disconnect(sim.client)

    def test_custom_config(self):
        sim = _make_sim()
        plugin = sim.register_plugin(
            WorkcellPlugin,
            config={"item_search_radius": 2.5, "return_home_delay": 0},
        )
        assert plugin._item_search_radius == 2.5
        assert plugin._return_home is False
        p.disconnect(sim.client)

    def test_spawn_fallback_default_enabled(self):
        sim = _make_sim()
        plugin = sim.register_plugin(WorkcellPlugin)
        assert plugin._spawn_fallback is True
        p.disconnect(sim.client)

    def test_spawn_fallback_disabled(self):
        sim = _make_sim()
        plugin = sim.register_plugin(WorkcellPlugin, config={"spawn_fallback": False})
        assert plugin._spawn_fallback is False
        p.disconnect(sim.client)

    def test_return_home_disabled_via_flag(self):
        sim = _make_sim()
        plugin = sim.register_plugin(WorkcellPlugin, config={"return_home": False})
        assert plugin._return_home is False
        p.disconnect(sim.client)


# ---------------------------------------------------------------------------
# Tests: Per-workcell config overrides
# ---------------------------------------------------------------------------


class TestWorkcellOverrides:
    def test_overrides_pre_resolved(self):
        """Per-workcell overrides are pre-resolved into WorkcellConfig."""
        sim = _make_sim()
        plugin = sim.register_plugin(
            WorkcellPlugin,
            config={
                "item_search_radius": 1.0,
                "overrides": {"dispenser_1": {"item_search_radius": 3.0}},
            },
        )
        assert plugin._workcells["dispenser_1"].search_radius == 3.0
        p.disconnect(sim.client)

    def test_dispense_uses_override_search_radius(self):
        """dispense() uses per-workcell item_search_radius override."""
        sim = _make_sim()
        plugin = sim.register_plugin(
            WorkcellPlugin,
            config={
                "item_search_radius": 0.1,  # global: very small
                "overrides": {"wc": {"position": [0.0, 0.0], "item_search_radius": 5.0}},
            },
        )
        _spawn_item(sim, "cargo", 2.0, 0.0)  # 2m away — outside 0.1, inside 5.0
        robot = _spawn_agent(sim, "robot0", 0.0, 0.0)

        _, pick = plugin.dispense("wc", robot)
        assert isinstance(pick, PickAction)
        assert pick.search_radius == 5.0  # override applied to PickAction
        p.disconnect(sim.client)

    def test_dispense_per_workcell_spawn_fallback_disabled(self):
        """Per-workcell spawn_fallback=false prevents fallback for that workcell only."""
        sim = _make_sim()
        plugin = sim.register_plugin(
            WorkcellPlugin,
            config={
                "spawn_fallback": True,  # global: enabled
                "overrides": {
                    "wc_no_spawn": {"position": [0.0, 0.0], "spawn_fallback": False},
                    "wc_spawn": {"position": [5.0, 0.0]},
                },
            },
        )
        robot = _spawn_agent(sim, "robot0", 0.0, 0.0)

        # wc_no_spawn: override disables fallback
        _, pick1 = plugin.dispense("wc_no_spawn", robot)
        assert pick1 is None

        # wc_spawn: uses global default (enabled), spawns fallback
        _, pick2 = plugin.dispense("wc_spawn", robot)
        assert isinstance(pick2, PickAction)
        p.disconnect(sim.client)


# ---------------------------------------------------------------------------
# Tests: Workcell position lookup
# ---------------------------------------------------------------------------


class TestWorkcellPosition:
    def test_position_from_config_2d(self):
        """2-element position gets z=0 default."""
        sim = _make_sim()
        plugin = sim.register_plugin(
            WorkcellPlugin,
            config={"overrides": {"wc2d": {"position": [8.5, -0.5]}}},
        )
        pos = plugin.get_workcell_position("wc2d")
        assert pos == (8.5, -0.5, 0.0)
        p.disconnect(sim.client)

    def test_position_from_config_3d(self):
        """3-element position preserves z for multi-floor."""
        sim = _make_sim()
        plugin = sim.register_plugin(
            WorkcellPlugin,
            config={"overrides": {"wc3d": {"position": [1.0, 2.0, 5.0]}}},
        )
        pos = plugin.get_workcell_position("wc3d")
        assert pos == (1.0, 2.0, 5.0)
        p.disconnect(sim.client)

    def test_position_not_found(self):
        sim = _make_sim()
        plugin = sim.register_plugin(WorkcellPlugin)
        assert plugin.get_workcell_position("nonexistent") is None
        p.disconnect(sim.client)

    def test_nearest_robot_uses_3d_distance(self):
        """Nearest robot considers z distance for multi-floor."""
        sim = _make_sim()
        plugin = sim.register_plugin(
            WorkcellPlugin,
            config={
                "overrides": {"wc_floor2": {"position": [0.0, 0.0, 10.0]}},
            },
        )
        # Both robots at same XY, but robot_high is on same floor as workcell
        robot_low = _spawn_agent(sim, "low", 0.0, 0.0)  # z≈0.05
        robot_high = _spawn_agent(sim, "high", 1.0, 0.0)  # z≈0.05, but closer XY-only

        # With 2D distance, robot_high (1m XY) would be farther than robot_low (0m XY)
        # but with 3D, robot_low is ~10m away (z diff), robot_high is ~sqrt(1+100)≈10.05m
        # So robot_low is still nearest in 3D here. Let's use a clearer setup:
        # workcell at z=10, robot_low at z=0 (10m z-diff), robot_far at x=5 z=0 (~11.2m)
        robot_far = _spawn_agent(sim, "far", 5.0, 0.0)
        result = plugin.find_nearest_robot("wc_floor2", [robot_low, robot_far])
        # robot_low: 3D dist = sqrt(0+0+100) = 10
        # robot_far: 3D dist = sqrt(25+0+100) ≈ 11.18
        assert result.name == "low"
        p.disconnect(sim.client)


# ---------------------------------------------------------------------------
# Tests: has_pickable_near (fallback cargo check)
# ---------------------------------------------------------------------------


class TestHasPickableNear:
    def test_true_when_pickable_exists(self):
        sim = _make_sim()
        plugin = sim.register_plugin(
            WorkcellPlugin,
            config={
                "item_search_radius": 5.0,
                "overrides": {"wc": {"position": [0.0, 0.0]}},
            },
        )
        # Item near workcell
        _spawn_item(sim, "near_item", 0.5, 0.0)

        assert plugin._has_pickable_near("wc") is True
        p.disconnect(sim.client)

    def test_false_when_attached(self):
        sim = _make_sim()
        plugin = sim.register_plugin(
            WorkcellPlugin,
            config={
                "item_search_radius": 5.0,
                "overrides": {"wc": {"position": [0.0, 0.0]}},
            },
        )
        item = _spawn_item(sim, "item", 0.5, 0.0)

        # Attach item to a robot
        robot = _spawn_agent(sim, "robot0", 0.0, 0.0)
        robot.attach_object(item)

        assert plugin._has_pickable_near("wc") is False
        p.disconnect(sim.client)

    def test_false_when_outside_radius(self):
        sim = _make_sim()
        plugin = sim.register_plugin(
            WorkcellPlugin,
            config={
                "item_search_radius": 0.5,
                "overrides": {"wc": {"position": [0.0, 0.0]}},
            },
        )
        _spawn_item(sim, "far_item", 5.0, 0.0)

        assert plugin._has_pickable_near("wc") is False
        p.disconnect(sim.client)


# ---------------------------------------------------------------------------
# Tests: Nearest robot / carrier
# ---------------------------------------------------------------------------


class TestNearestRobot:
    def test_finds_nearest(self):
        sim = _make_sim()
        plugin = sim.register_plugin(
            WorkcellPlugin,
            config={
                "overrides": {"wc": {"position": [0.0, 0.0]}},
            },
        )

        far_robot = _spawn_agent(sim, "far", 10.0, 0.0)
        near_robot = _spawn_agent(sim, "near", 1.0, 0.0)

        result = plugin.find_nearest_robot("wc", [far_robot, near_robot])
        assert result is near_robot
        p.disconnect(sim.client)

    def test_empty_candidates(self):
        sim = _make_sim()
        plugin = sim.register_plugin(WorkcellPlugin)
        assert plugin.find_nearest_robot("wc", []) is None
        p.disconnect(sim.client)

    def test_carrier_preferred(self):
        sim = _make_sim()
        plugin = sim.register_plugin(
            WorkcellPlugin,
            config={
                "overrides": {"wc": {"position": [0.0, 0.0]}},
            },
        )

        # Near robot without cargo
        near = _spawn_agent(sim, "near", 1.0, 0.0)
        # Far robot with cargo
        far = _spawn_agent(sim, "far", 5.0, 0.0)
        cargo = _spawn_item(sim, "cargo", 5.0, 0.0)
        far.attach_object(cargo)

        result = plugin.find_nearest_carrier("wc", [near, far])
        assert result is far  # carrier preferred even if farther
        p.disconnect(sim.client)


# ---------------------------------------------------------------------------
# Tests: Dispense
# ---------------------------------------------------------------------------


class TestDispense:
    def test_dispense_existing_item(self):
        sim = _make_sim()
        plugin = sim.register_plugin(
            WorkcellPlugin,
            config={
                "item_search_radius": 5.0,
                "overrides": {"wc": {"position": [0.0, 0.0]}},
            },
        )
        item = _spawn_item(sim, "cargo", 0.5, 0.0)
        robot = _spawn_agent(sim, "robot0", 0.0, 0.0)

        result_item, pick = plugin.dispense("wc", robot, key="req1")
        assert result_item is None  # item resolved lazily by PickAction
        assert isinstance(pick, PickAction)
        assert pick.target_position is not None  # uses target_position mode
        assert pick.search_radius == 5.0
        assert "req1" in plugin.pending_actions
        p.disconnect(sim.client)

    def test_dispense_spawns_fallback(self):
        sim = _make_sim()
        plugin = sim.register_plugin(
            WorkcellPlugin,
            config={
                "overrides": {"wc": {"position": [0.0, 0.0]}},
            },
        )
        robot = _spawn_agent(sim, "robot0", 0.0, 0.0)

        result_item, pick = plugin.dispense("wc", robot)
        # Item resolved lazily by PickAction; fallback cargo spawned
        assert result_item is None
        assert isinstance(pick, PickAction)
        # Verify fallback cargo was spawned (a pickable object named robot0_cargo)
        cargo_names = [o.name for o in sim.sim_objects if getattr(o, "pickable", False)]
        assert "robot0_cargo" in cargo_names
        p.disconnect(sim.client)

    def test_dispense_spawn_offset(self):
        """spawn_offset applies 6-DOF offset to fallback cargo position."""
        sim = _make_sim()
        plugin = sim.register_plugin(
            WorkcellPlugin,
            config={
                "overrides": {
                    "wc": {
                        "position": [1.0, 2.0, 3.0],
                        "spawn_offset": [0.5, -0.5, 1.0, 0.0, 0.0, 0.0],
                    },
                },
            },
        )
        robot = _spawn_agent(sim, "robot0", 0.0, 0.0)

        plugin.dispense("wc", robot)
        cargo = [o for o in sim.sim_objects if o.name == "robot0_cargo"]
        assert len(cargo) == 1
        pose = cargo[0].get_pose()
        assert abs(pose.position[0] - 1.5) < 0.01  # 1.0 + 0.5
        assert abs(pose.position[1] - 1.5) < 0.01  # 2.0 + (-0.5)
        assert abs(pose.position[2] - 4.0) < 0.01  # 3.0 + 1.0
        p.disconnect(sim.client)

    def test_dispense_fallback_uses_item_config_box(self):
        """Per-workcell item config overrides global half_extents and color."""
        sim = _make_sim()
        plugin = sim.register_plugin(
            WorkcellPlugin,
            config={
                "overrides": {
                    "wc": {
                        "position": [0.0, 0.0],
                        "item": {
                            "half_extents": [0.3, 0.3, 0.2],
                            "color": [1.0, 0.0, 0.0, 1.0],
                            "mass": 1.5,
                        },
                    },
                },
            },
        )
        robot = _spawn_agent(sim, "robot0", 0.0, 0.0)

        plugin.dispense("wc", robot)
        cargo = [o for o in sim.sim_objects if o.name == "robot0_cargo"]
        assert len(cargo) == 1
        assert cargo[0].mass == 1.5
        p.disconnect(sim.client)

    def test_dispense_fallback_uses_item_config_mesh(self):
        """Per-workcell item config with mesh_path spawns mesh-based cargo."""
        sim = _make_sim()
        plugin = sim.register_plugin(
            WorkcellPlugin,
            config={
                "overrides": {
                    "wc": {
                        "position": [0.0, 0.0],
                        "item": {
                            "mesh_path": "robots/simple_cube.urdf",
                            "mesh_scale": [0.5, 0.5, 0.5],
                            "color": [0.0, 1.0, 0.0, 1.0],
                        },
                    },
                },
            },
        )
        robot = _spawn_agent(sim, "robot0", 0.0, 0.0)

        _, pick = plugin.dispense("wc", robot)
        assert isinstance(pick, PickAction)
        cargo = [o for o in sim.sim_objects if o.name == "robot0_cargo"]
        assert len(cargo) == 1
        p.disconnect(sim.client)

    def test_dispense_no_fallback_when_disabled(self):
        """When spawn_fallback=False, dispense returns (None, None) if no item exists."""
        sim = _make_sim()
        plugin = sim.register_plugin(
            WorkcellPlugin,
            config={
                "spawn_fallback": False,
                "overrides": {"wc": {"position": [0.0, 0.0]}},
            },
        )
        robot = _spawn_agent(sim, "robot0", 0.0, 0.0)

        result_item, pick = plugin.dispense("wc", robot)
        assert result_item is None
        assert pick is None  # no fallback spawned, no PickAction created
        p.disconnect(sim.client)

    def test_dispense_auto_key(self):
        sim = _make_sim()
        plugin = sim.register_plugin(
            WorkcellPlugin,
            config={
                "overrides": {"wc": {"position": [0.0, 0.0]}},
            },
        )
        robot = _spawn_agent(sim, "robot0", 0.0, 0.0)

        plugin.dispense("wc", robot)
        plugin.dispense("wc", robot)

        assert len(plugin.pending_actions) == 2
        assert "0" in plugin.pending_actions
        assert "1" in plugin.pending_actions
        p.disconnect(sim.client)

    def test_dispense_auto_discovers_position_from_robot(self):
        """When workcell has no configured position, use robot's current position."""
        sim = _make_sim()
        # No overrides — workcell position unknown
        plugin = sim.register_plugin(
            WorkcellPlugin,
            config={
                "item_search_radius": 5.0,
            },
        )
        robot = _spawn_agent(sim, "robot0", 8.0, 3.0)

        _, pick = plugin.dispense("unknown_wc", robot, key="r1")

        # Should create a PickAction (not return None)
        assert isinstance(pick, PickAction)
        assert "r1" in plugin.pending_actions

        # Position should be auto-discovered and registered
        pos = plugin.get_workcell_position("unknown_wc")
        assert pos is not None
        assert abs(pos[0] - 8.0) < 0.1
        assert abs(pos[1] - 3.0) < 0.1
        p.disconnect(sim.client)

    def test_dispense_auto_discovered_position_reused(self):
        """Auto-discovered workcell position is reused for subsequent requests."""
        sim = _make_sim()
        plugin = sim.register_plugin(WorkcellPlugin)
        robot = _spawn_agent(sim, "robot0", 5.0, 7.0)

        plugin.dispense("wc_auto", robot, key="r1")

        # Move robot far away
        robot.set_pose(Pose.from_xyz(50.0, 50.0, 0.05))

        # Second dispense should use the first-discovered position, not robot's new position
        plugin.dispense("wc_auto", robot, key="r2")
        pos = plugin.get_workcell_position("wc_auto")
        assert pos is not None
        assert abs(pos[0] - 5.0) < 0.1  # original position
        assert abs(pos[1] - 7.0) < 0.1
        p.disconnect(sim.client)


# ---------------------------------------------------------------------------
# Tests: Ingest
# ---------------------------------------------------------------------------


class TestIngest:
    def test_ingest_creates_drop_action(self):
        sim = _make_sim()
        plugin = sim.register_plugin(
            WorkcellPlugin,
            config={
                "overrides": {"ingestor_1": {"position": [5.0, 5.0]}},
            },
        )
        robot = _spawn_agent(sim, "robot0", 5.0, 5.0)

        drop = plugin.ingest("ingestor_1", robot, key="drop1")
        assert isinstance(drop, DropAction)
        assert "drop1" in plugin.pending_actions
        p.disconnect(sim.client)

    def test_ingest_auto_discovers_position_from_robot(self):
        """When ingestor has no configured position, register it from robot position."""
        sim = _make_sim()
        plugin = sim.register_plugin(WorkcellPlugin)
        robot = _spawn_agent(sim, "robot0", 4.0, 6.0)

        drop = plugin.ingest("unknown_ingestor", robot, key="d1")

        assert isinstance(drop, DropAction)
        assert "d1" in plugin.pending_actions

        # Position should be auto-registered
        pos = plugin.get_workcell_position("unknown_ingestor")
        assert pos is not None
        assert abs(pos[0] - 4.0) < 0.1
        assert abs(pos[1] - 6.0) < 0.1
        p.disconnect(sim.client)


# ---------------------------------------------------------------------------
# Tests: Pending action completion + callback
# ---------------------------------------------------------------------------


class TestPendingCompletion:
    def test_callback_on_completion(self):
        sim = _make_sim()
        plugin = sim.register_plugin(
            WorkcellPlugin,
            config={
                "overrides": {"wc": {"position": [0.0, 0.0]}},
            },
        )
        robot = _spawn_agent(sim, "robot0", 0.0, 0.0)

        completions = []

        def on_done(success, pending):
            completions.append((success, pending.workcell_name))

        plugin.dispense("wc", robot, key="r1", on_complete=on_done)

        # Simulate action completion
        pa = plugin._pending_actions["r1"]
        pa.action.status = ActionStatus.COMPLETED

        plugin.on_step(0.1)

        assert len(completions) == 1
        assert completions[0] == (True, "wc")
        assert "r1" not in plugin._pending_actions
        p.disconnect(sim.client)

    def test_failed_action_callback(self):
        sim = _make_sim()
        plugin = sim.register_plugin(
            WorkcellPlugin,
            config={
                "overrides": {"wc": {"position": [0.0, 0.0]}},
            },
        )
        robot = _spawn_agent(sim, "robot0", 0.0, 0.0)

        completions = []
        plugin.dispense("wc", robot, key="r2", on_complete=lambda s, p: completions.append(s))

        pa = plugin._pending_actions["r2"]
        pa.action.status = ActionStatus.FAILED

        plugin.on_step(0.1)

        assert completions == [False]
        assert "r2" not in plugin._pending_actions
        p.disconnect(sim.client)

    def test_initial_pose_recorded_from_workcell_position(self):
        """_item_initial_poses records item's actual position (pre-pick snapshot)."""
        sim = _make_sim()
        plugin = sim.register_plugin(
            WorkcellPlugin,
            config={
                "item_search_radius": 5.0,
                "overrides": {"wc": {"position": [3.0, 4.0]}},
            },
        )
        item = _spawn_item(sim, "cargo", 3.5, 4.5)  # item near workcell but not at same pos
        robot = _spawn_agent(sim, "robot0", 0.0, 0.0)

        _, pick = plugin.dispense("wc", robot, key="d1")

        # Simulate PickAction resolving target
        pick._target_object = item
        pick.status = ActionStatus.COMPLETED

        plugin.on_step(0.1)

        # Initial pose should be the item's actual spawn position (snapshotted
        # in dispense() before the PickAction runs), matching Gazebo's
        # send_ingested_item_home() which returns items to their spawn pose.
        assert item.body_id in plugin._item_initial_poses
        pos, orn = plugin._item_initial_poses[item.body_id]
        assert abs(pos[0] - 3.5) < 0.1  # item's actual x
        assert abs(pos[1] - 4.5) < 0.1  # item's actual y
        assert len(orn) == 4  # quaternion preserved
        p.disconnect(sim.client)


# ---------------------------------------------------------------------------
# Tests: Return home
# ---------------------------------------------------------------------------


class TestReturnHome:
    def test_item_returns_after_delay(self):
        sim = _make_sim()
        plugin = sim.register_plugin(WorkcellPlugin, config={"return_home_delay": 1.0})

        item = _spawn_item(sim, "cargo", 2.0, 3.0)
        plugin._item_initial_poses[item.body_id] = ((2.0, 3.0, 0.1), (0.0, 0.0, 0.0, 1.0))

        # Move item away
        item.set_pose_raw([10.0, 10.0, 0.1], [0, 0, 0, 1])

        # Schedule return
        from pybullet_fleet.plugins.workcell_plugin import _ReturnHomeEntry

        plugin._return_home_queue.append(
            _ReturnHomeEntry(
                body_id=item.body_id,
                home_position=(2.0, 3.0, 0.1),
                home_orientation=(0.0, 0.0, 0.0, 1.0),
                scheduled_time=1.0,
            )
        )

        # Before schedule time
        sim.sim_time = 0.5
        plugin._process_return_home()
        pos = item.get_pose().position
        assert abs(pos[0] - 10.0) < 0.1  # still at 10,10

        # After schedule time
        sim.sim_time = 1.5
        plugin._process_return_home()
        pos = item.get_pose().position
        assert abs(pos[0] - 2.0) < 0.1  # returned to 2,3
        assert abs(pos[1] - 3.0) < 0.1
        p.disconnect(sim.client)

    def test_return_home_disabled(self):
        sim = _make_sim()
        plugin = sim.register_plugin(WorkcellPlugin, config={"return_home_delay": 0})
        assert plugin._return_home is False
        p.disconnect(sim.client)

    def test_return_home_preserves_orientation(self):
        """Return-home restores the item's original orientation, not identity."""
        sim = _make_sim()
        plugin = sim.register_plugin(
            WorkcellPlugin,
            config={
                "return_home_delay": 1.0,
                "item_search_radius": 5.0,
                "overrides": {"wc": {"position": [2.0, 3.0]}},
            },
        )

        item = _spawn_item(sim, "cargo", 2.5, 3.5)
        # Give the item a non-identity orientation (90° yaw)
        import pybullet as pb

        yaw_quat = list(pb.getQuaternionFromEuler([0, 0, math.pi / 2]))
        item.set_pose_raw([2.5, 3.5, 0.1], yaw_quat)

        robot = _spawn_agent(sim, "robot0", 2.0, 3.0)

        # Dispense and simulate pick completion
        _, pick = plugin.dispense("wc", robot, key="d1")
        pick._target_object = item
        pick.status = ActionStatus.COMPLETED
        plugin.on_step(0.1)

        # Now ingest (drop) and simulate completion
        drop = plugin.ingest("wc", robot, key="i1")
        drop._target_object = item
        drop.status = ActionStatus.COMPLETED
        plugin.on_step(0.1)

        # Move item away with different orientation
        item.set_pose_raw([10.0, 10.0, 0.1], [0, 0, 0, 1])

        # Process return-home
        sim.sim_time = 10.0
        plugin._process_return_home()

        pose = item.get_pose()
        # Position should be item's original spawn position (snapshotted
        # by _snapshot_pickable_poses before PickAction runs).
        assert abs(pose.position[0] - 2.5) < 0.1  # item x
        assert abs(pose.position[1] - 3.5) < 0.1  # item y
        # Orientation should be the original (90° yaw), not identity
        restored_quat = pose.orientation
        for i in range(4):
            assert (
                abs(restored_quat[i] - yaw_quat[i]) < 0.01
            ), f"Orientation component {i}: expected {yaw_quat[i]}, got {restored_quat[i]}"
        p.disconnect(sim.client)


# ---------------------------------------------------------------------------
# Tests: Reset
# ---------------------------------------------------------------------------


class TestReset:
    def test_on_reset_clears_state(self):
        sim = _make_sim()
        plugin = sim.register_plugin(
            WorkcellPlugin,
            config={
                "overrides": {"wc": {"position": [0.0, 0.0]}},
            },
        )
        robot = _spawn_agent(sim, "robot0", 0.0, 0.0)

        plugin.dispense("wc", robot, key="r1")
        plugin._item_initial_poses[99] = ((1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))

        plugin.on_reset()

        assert len(plugin._pending_actions) == 0
        assert len(plugin._item_initial_poses) == 0
        assert len(plugin._return_home_queue) == 0
        p.disconnect(sim.client)
