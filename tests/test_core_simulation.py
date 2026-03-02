"""
Unit tests for MultiRobotSimulationCore.

Tests object lifecycle, simulation step, callback system, and configuration
using a real PyBullet environment (DIRECT mode).
"""

import pytest
import pybullet as p

from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import SimObject, ShapeParams
from pybullet_fleet.agent import Agent, AgentSpawnParams
from pybullet_fleet.geometry import Pose
from pybullet_fleet.types import (
    CollisionMode,
    CollisionDetectionMethod,
    SpatialHashCellSizeMode,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------


@pytest.fixture
def sim_core():
    """Headless kinematics sim with collision enabled."""
    params = SimulationParams(
        gui=False,
        physics=False,
        monitor=False,
        collision_detection_method=CollisionDetectionMethod.CLOSEST_POINTS,
        collision_margin=0.02,
        spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
        spatial_hash_cell_size=2.0,
        ignore_static_collision=False,
        log_level="warning",
    )
    sc = MultiRobotSimulationCore(params)
    sc.set_collision_spatial_hash_cell_size_mode()
    yield sc
    p.disconnect(sc.client)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def make_box(sim_core, position, size=0.25, collision_mode=CollisionMode.NORMAL_3D, mass=0.0):
    """Create a box SimObject registered to sim_core."""
    return SimObject.from_mesh(
        visual_shape=ShapeParams(shape_type="box", half_extents=[size, size, size]),
        collision_shape=ShapeParams(shape_type="box", half_extents=[size, size, size]),
        pose=Pose.from_xyz(*position),
        mass=mass,
        sim_core=sim_core,
        collision_mode=collision_mode,
    )


def make_agent(sim_core, position=(0, 0, 0)):
    """Create an Agent registered to sim_core."""
    return Agent.from_params(
        AgentSpawnParams(
            urdf_path="robots/mobile_robot.urdf",
            initial_pose=Pose.from_xyz(*position),
            collision_mode=CollisionMode.NORMAL_3D,
        ),
        sim_core=sim_core,
    )


# ============================================================================
# Object Lifecycle
# ============================================================================


class TestObjectLifecycle:
    """Test add_object / remove_object with real sim_core caches."""

    def test_add_object_registers_in_dict(self, sim_core):
        obj = make_box(sim_core, [0, 0, 0])
        assert obj.object_id in sim_core._sim_objects_dict
        assert obj in sim_core.sim_objects

    def test_add_object_sets_collision_mode_cache(self, sim_core):
        obj = make_box(sim_core, [0, 0, 0], collision_mode=CollisionMode.NORMAL_3D)
        assert sim_core._cached_collision_modes[obj.object_id] == CollisionMode.NORMAL_3D

    def test_add_object_registers_aabb(self, sim_core):
        obj = make_box(sim_core, [0, 0, 0])
        assert obj.object_id in sim_core._cached_aabbs_dict

    def test_add_object_registers_spatial_grid_cell(self, sim_core):
        obj = make_box(sim_core, [0, 0, 0])
        assert obj.object_id in sim_core._cached_object_to_cell
        cells = sim_core._cached_object_to_cell[obj.object_id]
        assert len(cells) >= 1

    def test_add_agent_registers_in_agents_list(self, sim_core):
        agent = make_agent(sim_core)
        assert agent in sim_core.agents
        assert agent in sim_core.sim_objects

    def test_add_disabled_object_skips_collision_system(self, sim_core):
        obj = make_box(sim_core, [0, 0, 0], collision_mode=CollisionMode.DISABLED)
        assert obj.object_id in sim_core._disabled_collision_objects
        assert obj.object_id not in sim_core._cached_aabbs_dict

    def test_add_static_object_registers_in_static_collision_set(self, sim_core):
        """STATIC object should be in _static_collision_objects (collision domain),
        not in _dynamic_collision_objects."""
        obj = make_box(sim_core, [0, 0, 0], collision_mode=CollisionMode.STATIC)
        assert obj.object_id in sim_core._static_collision_objects
        assert obj.object_id not in sim_core._dynamic_collision_objects

    def test_add_normal_object_registers_in_dynamic_collision_set(self, sim_core):
        obj = make_box(sim_core, [0, 0, 0], collision_mode=CollisionMode.NORMAL_3D)
        assert obj.object_id in sim_core._dynamic_collision_objects

    def test_static_collision_objects_not_managed_by_movement_type(self, sim_core):
        """_register_object_movement_type should NOT touch _static_collision_objects.
        The collision set is managed exclusively by collision-domain methods."""
        obj = make_box(sim_core, [0, 0, 0], collision_mode=CollisionMode.STATIC)
        # Remove from collision set manually, then re-register movement type
        sim_core._static_collision_objects.discard(obj.object_id)
        sim_core._register_object_movement_type(obj)
        # Movement type registration should NOT re-add to collision set
        assert obj.object_id not in sim_core._static_collision_objects

    def test_register_object_movement_type_is_idempotent(self, sim_core):
        """_register_object_movement_type is idempotent.

        Calling it twice (initial add + re-registration on mode change) must
        produce exactly the same state with no duplicates or stale entries.
        """
        obj = make_box(sim_core, [0, 0, 0], collision_mode=CollisionMode.NORMAL_3D)
        oid = obj.object_id

        # Initially kinematic (mass=0)
        assert oid in sim_core._kinematic_objects
        assert oid not in sim_core._physics_objects

        # Calling again should be a safe no-op
        sim_core._register_object_movement_type(obj)

        assert oid in sim_core._kinematic_objects
        assert oid not in sim_core._physics_objects
        # set size unchanged (no duplicate entries)
        assert sum(1 for x in sim_core._kinematic_objects if x == oid) == 1

    def test_update_object_movement_type_removed(self, sim_core):
        """_update_object_movement_type was merged into _register_object_movement_type.

        The separate wrapper no longer exists."""
        assert not hasattr(sim_core, "_update_object_movement_type")

    def test_update_object_collision_mode_calls_movement_type_update(self, sim_core):
        """_update_object_collision_mode should delegate movement-type update
        to _register_object_movement_type (single idempotent method)."""
        obj = make_box(sim_core, [0, 0, 0], collision_mode=CollisionMode.NORMAL_3D)
        oid = obj.object_id
        assert oid in sim_core._kinematic_objects

        # Transition NORMAL_3D -> STATIC
        obj.set_collision_mode(CollisionMode.STATIC)

        # Movement type should still be correctly registered
        # (STATIC obj with mass=0 is still "static" in movement sense)
        assert oid not in sim_core._physics_objects
        # Kinematic set should be cleared for static objects
        assert oid not in sim_core._kinematic_objects

    def test_add_object_to_collision_system_classifies_static(self, sim_core):
        """_add_object_to_collision_system should register a STATIC object
        into _static_collision_objects (collision-domain classification)."""
        obj = make_box(sim_core, [0, 0, 0], collision_mode=CollisionMode.STATIC)
        oid = obj.object_id
        # Clear the set to test classification by _add_object_to_collision_system
        sim_core._static_collision_objects.discard(oid)
        assert oid not in sim_core._static_collision_objects

        sim_core._add_object_to_collision_system(oid)
        assert oid in sim_core._static_collision_objects

    def test_add_object_to_collision_system_classifies_normal(self, sim_core):
        """_add_object_to_collision_system should register a NORMAL_3D object
        into _dynamic_collision_objects (collision-domain classification)."""
        obj = make_box(sim_core, [0, 0, 0], collision_mode=CollisionMode.NORMAL_3D)
        oid = obj.object_id
        # Clear the set entry to test classification by _add_object_to_collision_system
        sim_core._dynamic_collision_objects.discard(oid)
        assert oid not in sim_core._dynamic_collision_objects

        sim_core._add_object_to_collision_system(oid)
        assert oid in sim_core._dynamic_collision_objects

    def test_add_duplicate_is_noop(self, sim_core):
        obj = make_box(sim_core, [0, 0, 0])
        count_before = len(sim_core.sim_objects)
        sim_core.add_object(obj)  # duplicate
        assert len(sim_core.sim_objects) == count_before

    def test_remove_object_clears_all_caches(self, sim_core):
        obj = make_box(sim_core, [0, 0, 0])
        obj_id = obj.object_id
        sim_core.remove_object(obj)

        assert obj_id not in sim_core._sim_objects_dict
        assert obj not in sim_core.sim_objects
        assert obj_id not in sim_core._cached_collision_modes
        assert obj_id not in sim_core._cached_aabbs_dict
        assert obj_id not in sim_core._cached_object_to_cell
        assert obj_id not in sim_core._dynamic_collision_objects

    def test_remove_agent_clears_agents_list(self, sim_core):
        agent = make_agent(sim_core)
        sim_core.remove_object(agent)
        assert agent not in sim_core.agents
        assert agent not in sim_core.sim_objects

    def test_remove_nonexistent_is_noop(self, sim_core):
        obj = make_box(sim_core, [0, 0, 0])
        sim_core.remove_object(obj)
        # Second remove should not raise
        sim_core.remove_object(obj)

    def test_remove_object_clears_robot_original_colors(self, sim_core):
        """remove_object should delete body_id entry from _robot_original_colors."""
        obj = make_box(sim_core, [0, 0, 0])
        body_id = obj.body_id
        # add_object should have stored the color
        assert body_id in sim_core._robot_original_colors

        sim_core.remove_object(obj)
        # After removal the stale entry must be gone
        assert body_id not in sim_core._robot_original_colors

    def test_remove_clears_active_collision_pairs(self, sim_core):
        obj1 = make_box(sim_core, [0, 0, 0])
        obj2 = make_box(sim_core, [0.3, 0, 0])
        sim_core._moved_this_step.add(obj2.object_id)
        sim_core.check_collisions()

        sim_core.remove_object(obj2)
        # No pair should reference removed object
        for pair in sim_core._active_collision_pairs:
            assert obj2.object_id not in pair


# ============================================================================
# SimulationParams
# ============================================================================


class TestSimulationParams:
    """Test SimulationParams construction and from_dict."""

    def test_defaults(self):
        params = SimulationParams(gui=False, monitor=False)
        assert params.physics is False
        assert params.timestep == pytest.approx(1.0 / 240.0)
        assert params.collision_margin == 0.02
        assert params.multi_cell_threshold == 1.5

    def test_auto_select_closest_points_for_kinematics(self):
        params = SimulationParams(gui=False, physics=False, monitor=False)
        assert params.collision_detection_method == CollisionDetectionMethod.CLOSEST_POINTS

    def test_auto_select_contact_points_for_physics(self):
        params = SimulationParams(gui=False, physics=True, monitor=False)
        assert params.collision_detection_method == CollisionDetectionMethod.CONTACT_POINTS

    def test_explicit_method_overrides_auto(self):
        params = SimulationParams(
            gui=False,
            physics=False,
            monitor=False,
            collision_detection_method=CollisionDetectionMethod.HYBRID,
        )
        assert params.collision_detection_method == CollisionDetectionMethod.HYBRID

    def test_from_dict(self):
        cfg = {
            "gui": False,
            "physics": True,
            "monitor": False,
            "timestep": 0.01,
            "collision_margin": 0.05,
        }
        params = SimulationParams.from_dict(cfg)
        assert params.physics is True
        assert params.timestep == 0.01
        assert params.collision_margin == 0.05

    def test_from_yaml(self, tmp_path):
        import yaml

        cfg = {
            "gui": False,
            "physics": False,
            "monitor": False,
            "timestep": 0.05,
            "speed": 2.0,
        }
        yaml_file = tmp_path / "test_config.yaml"
        yaml_file.write_text(yaml.dump(cfg))
        params = SimulationParams.from_config(str(yaml_file))
        assert params.timestep == 0.05
        assert params.speed == 2.0


# ============================================================================
# Initialization
# ============================================================================


class TestInitialization:
    """Test MultiRobotSimulationCore initialization."""

    def test_pybullet_connected(self, sim_core):
        assert sim_core.client is not None
        assert p.isConnected(physicsClientId=sim_core.client)

    def test_empty_initial_state(self, sim_core):
        assert len(sim_core.sim_objects) == 0
        assert len(sim_core.agents) == 0
        assert sim_core.step_count == 0
        assert sim_core.sim_time == 0.0

    def test_from_dict(self):
        cfg = {"gui": False, "physics": False, "monitor": False}
        sc = MultiRobotSimulationCore.from_dict(cfg)
        try:
            assert sc.params.gui is False
            assert p.isConnected(physicsClientId=sc.client)
        finally:
            p.disconnect(sc.client)


# ============================================================================
# step_once
# ============================================================================


class TestStepOnce:
    """Test step_once execution."""

    def test_step_increments_count(self, sim_core):
        sim_core.step_once()
        assert sim_core.step_count == 1

    def test_step_advances_sim_time(self, sim_core):
        # sim_time is computed BEFORE step_count is incremented:
        # sim_time = step_count * dt (where step_count is the OLD value)
        # After 2 steps: step_count=2, sim_time = 1*dt
        dt = sim_core.params.timestep
        sim_core.step_once()
        sim_core.step_once()
        assert sim_core.sim_time == pytest.approx(dt, abs=1e-9)

    def test_step_updates_agent(self, sim_core):
        agent = make_agent(sim_core, position=(0, 0, 0))
        goal = Pose.from_xyz(5, 0, 0)
        agent.set_goal_pose(goal)

        for _ in range(10):
            sim_core.step_once()

        pos = agent.get_pose().position
        # Agent should have moved towards goal
        assert pos[0] > 0.0, "Agent should move towards goal"

    def test_step_return_profiling(self, sim_core):
        # Enable time profiling on the sim_core
        sim_core._enable_time_profiling = True
        result = sim_core.step_once(return_profiling=True)
        assert result is not None
        assert "total" in result
        assert "agent_update" in result
        assert "collision_check" in result
        assert all(v >= 0 for v in result.values())

    def test_multiple_steps_accumulate(self, sim_core):
        n = 50
        dt = sim_core.params.timestep
        for _ in range(n):
            sim_core.step_once()
        assert sim_core.step_count == n
        # sim_time is computed before step_count increment, so after n steps: sim_time = (n-1) * dt
        assert sim_core.sim_time == pytest.approx((n - 1) * dt, abs=1e-6)

    def test_step_triggers_collision_check(self, sim_core):
        obj1 = make_box(sim_core, [0, 0, 0])
        obj2 = make_box(sim_core, [0.3, 0, 0])
        sim_core._moved_this_step.add(obj1.object_id)

        sim_core.step_once()

        # After step, active_collision_pairs should be populated
        pairs = sim_core.get_active_collision_pairs()
        expected = (
            min(obj1.object_id, obj2.object_id),
            max(obj1.object_id, obj2.object_id),
        )
        assert expected in pairs


# ============================================================================
# Callback System
# ============================================================================


class TestCallbackSystem:
    """Test register_callback and execution."""

    def test_callback_called_every_step(self, sim_core):
        calls = []
        sim_core.register_callback(lambda sc, dt: calls.append(dt), frequency=None)
        for _ in range(5):
            sim_core.step_once()
        assert len(calls) == 5

    def test_callback_frequency_control(self, sim_core):
        """Callback at 1 Hz should fire ~once per second of sim_time."""
        calls = []
        sim_core.register_callback(lambda sc, dt: calls.append(sc.sim_time), frequency=1.0)

        dt = sim_core.params.timestep
        # Run 2 seconds of sim time
        steps = int(2.0 / dt)
        for _ in range(steps):
            sim_core.step_once()

        # Should fire approximately 2 times (at t≈1.0 and t≈2.0)
        assert 1 <= len(calls) <= 3, f"Expected ~2 calls at 1Hz over 2s, got {len(calls)}"

    def test_multiple_callbacks(self, sim_core):
        calls_a, calls_b = [], []
        sim_core.register_callback(lambda sc, dt: calls_a.append(1), frequency=None)
        sim_core.register_callback(lambda sc, dt: calls_b.append(1), frequency=None)
        sim_core.step_once()
        assert len(calls_a) == 1
        assert len(calls_b) == 1


# ============================================================================
# ignore_static_collision
# ============================================================================


class TestIgnoreStaticCollision:
    """Test ignore_static_collision flag behaviour."""

    @pytest.fixture
    def sim_ignore_static(self):
        params = SimulationParams(
            gui=False,
            physics=False,
            monitor=False,
            collision_detection_method=CollisionDetectionMethod.CLOSEST_POINTS,
            spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
            spatial_hash_cell_size=2.0,
            ignore_static_collision=True,
            log_level="warning",
        )
        sc = MultiRobotSimulationCore(params)
        sc.set_collision_spatial_hash_cell_size_mode()
        yield sc
        p.disconnect(sc.client)

    def test_static_collision_ignored(self, sim_ignore_static):
        static = make_box(sim_ignore_static, [0, 0, 0], collision_mode=CollisionMode.STATIC)
        normal = make_box(sim_ignore_static, [0.3, 0, 0], collision_mode=CollisionMode.NORMAL_3D)
        sim_ignore_static._moved_this_step.add(normal.object_id)

        pairs, _ = sim_ignore_static.check_collisions()
        pair = (min(static.object_id, normal.object_id), max(static.object_id, normal.object_id))
        assert pair not in pairs, "Static collision should be ignored"

    def test_static_collision_detected_when_not_ignored(self, sim_core):
        static = make_box(sim_core, [0, 0, 0], collision_mode=CollisionMode.STATIC)
        normal = make_box(sim_core, [0.3, 0, 0], collision_mode=CollisionMode.NORMAL_3D)
        sim_core._moved_this_step.add(normal.object_id)

        pairs, _ = sim_core.check_collisions()
        pair = (min(static.object_id, normal.object_id), max(static.object_id, normal.object_id))
        assert pair in pairs, "Static collision should be detected when ignore_static=False"


# ============================================================================
# Collision Check Frequency
# ============================================================================


class TestCollisionCheckFrequency:
    """Test collision_check_frequency parameter."""

    def test_frequency_zero_skips_checks(self):
        params = SimulationParams(
            gui=False,
            physics=False,
            monitor=False,
            collision_check_frequency=0,
            log_level="warning",
        )
        sc = MultiRobotSimulationCore(params)
        try:
            obj1 = make_box(sc, [0, 0, 0])
            obj2 = make_box(sc, [0.3, 0, 0])
            sc._moved_this_step.add(obj1.object_id)

            sc.step_once()

            # With frequency=0, collision check is skipped
            assert len(sc.get_active_collision_pairs()) == 0
        finally:
            p.disconnect(sc.client)

    def test_frequency_none_checks_every_step(self, sim_core):
        obj1 = make_box(sim_core, [0, 0, 0])
        obj2 = make_box(sim_core, [0.3, 0, 0])
        sim_core._moved_this_step.add(obj1.object_id)

        sim_core.step_once()
        assert len(sim_core.get_active_collision_pairs()) > 0


# ============================================================================
# configure_visualizer
# ============================================================================


class TestConfigureVisualizerWarning:
    """Test configure_visualizer static objects warning prints correct count."""

    def test_static_warning_expands_count(self, sim_core, capsys):
        """Bug: print('%d', len(...)) does not expand %d; should use f-string."""
        # Create > 100 static objects to trigger the warning
        for i in range(101):
            make_box(sim_core, [i * 2, 0, 0], collision_mode=CollisionMode.STATIC)

        # Force gui param so configure_visualizer body executes
        sim_core._params.gui = True
        sim_core.configure_visualizer()

        captured = capsys.readouterr().out
        # The warning must contain the actual integer count, not literal "%d"
        assert "%d" not in captured, "print still contains unexpanded %d placeholder"
        assert "101" in captured, "Warning should contain the actual static object count"


class TestConfigureVisualizerLogMessage:
    """Test configure_visualizer log messages are accurate."""

    def test_keyboard_log_no_visual_key(self, sim_core, caplog):
        """v=visual and c=collision should not appear in log since features were removed."""
        import logging

        sim_core._params.gui = True
        with caplog.at_level(logging.INFO):
            sim_core.configure_visualizer()

        keyboard_logs = [r.message for r in caplog.records if "Keyboard controls" in r.message]
        assert keyboard_logs, "Expected a 'Keyboard controls' log message"
        assert "v=visual" not in keyboard_logs[0], "Stale 'v=visual' reference in log message"
        assert "c=collision" not in keyboard_logs[0], "Stale 'c=collision' reference — duplicates PyBullet 'w' key"


class TestCollisionShapesVisibilityRemoved:
    """Verify _set_collision_shapes_visibility and c-key handler are removed."""

    def test_method_removed(self, sim_core):
        """_set_collision_shapes_visibility should no longer exist (duplicates PyBullet 'w')."""
        assert not hasattr(
            sim_core, "_set_collision_shapes_visibility"
        ), "_set_collision_shapes_visibility should be removed (duplicates PyBullet built-in 'w' key)"

    def test_collision_shapes_enabled_attr_removed(self, sim_core):
        """_collision_shapes_enabled state should no longer exist."""
        assert not hasattr(sim_core, "_collision_shapes_enabled"), "_collision_shapes_enabled should be removed"


class TestConfigureVisualizerBuiltinKeybinds:
    """Test configure_visualizer documents PyBullet built-in keybinds."""

    def test_prints_builtin_keybind_info(self, sim_core, capsys):
        """configure_visualizer should document PyBullet built-in keybinds."""
        sim_core._params.gui = True
        sim_core.configure_visualizer()

        captured = capsys.readouterr().out
        # Should mention 'w' for wireframe (PyBullet built-in)
        assert "w" in captured.lower(), "Should document PyBullet's built-in 'w' key for wireframe"


class TestSetStructureTransparencyLogging:
    """Test _set_structure_transparency uses logger, not print."""

    def test_no_print_output(self, sim_core, capsys, monkeypatch):
        """_set_structure_transparency should use logger.info, not print."""
        import pybullet as pb

        monkeypatch.setattr(pb, "changeVisualShape", lambda *args, **kwargs: None)
        sim_core._params.gui = True
        # Add a static object so there's something to process
        obj = make_box(sim_core, [0, 0, 0], collision_mode=CollisionMode.STATIC)
        sim_core._original_visual_colors[(obj.body_id, -1)] = [1.0, 0.0, 0.0, 1.0]

        sim_core._set_structure_transparency(True)

        captured = capsys.readouterr().out
        assert "[TRANSPARENCY]" not in captured, "Should use logger.info, not print"


class TestSetStructureTransparencyRendering:
    """Test _set_structure_transparency disables rendering during batch update."""

    def test_disables_rendering_during_batch_update(self, sim_core, monkeypatch):
        """Should disable rendering before changing shapes and re-enable after."""
        import pybullet as pb

        sim_core._params.gui = True
        obj = make_box(sim_core, [0, 0, 0], collision_mode=CollisionMode.STATIC)
        sim_core._original_visual_colors[(obj.body_id, -1)] = [1.0, 0.0, 0.0, 1.0]

        calls = []
        monkeypatch.setattr(
            pb,
            "configureDebugVisualizer",
            lambda flag, enable, **kw: calls.append(("configDbgVis", flag, enable)),
        )
        monkeypatch.setattr(
            pb,
            "changeVisualShape",
            lambda *args, **kwargs: calls.append(("changeVis",)),
        )

        sim_core._set_structure_transparency(True)

        # Rendering must be disabled (COV_ENABLE_RENDERING=0) before any changeVisualShape
        # and re-enabled (COV_ENABLE_RENDERING=1) after all changeVisualShape calls
        assert len(calls) >= 3, f"Expected at least 3 calls, got {len(calls)}"

        # First call: disable rendering
        assert calls[0] == ("configDbgVis", pb.COV_ENABLE_RENDERING, 0), f"First call should disable rendering, got {calls[0]}"
        # Last call: re-enable rendering
        assert calls[-1] == (
            "configDbgVis",
            pb.COV_ENABLE_RENDERING,
            1,
        ), f"Last call should re-enable rendering, got {calls[-1]}"
        # All middle calls should be changeVisualShape
        for c in calls[1:-1]:
            assert c == ("changeVis",), f"Middle calls should be changeVisualShape, got {c}"

    def test_rendering_restored_on_exception(self, sim_core, monkeypatch):
        """Rendering must be re-enabled even if changeVisualShape raises."""
        import pybullet as pb

        sim_core._params.gui = True
        obj = make_box(sim_core, [0, 0, 0], collision_mode=CollisionMode.STATIC)
        sim_core._original_visual_colors[(obj.body_id, -1)] = [1.0, 0.0, 0.0, 1.0]
        # Add a second entry that will cause exception
        sim_core._original_visual_colors[(9999, -1)] = [0.0, 1.0, 0.0, 1.0]
        sim_core._static_collision_objects.add("fake_static_9999")
        # Patch _sim_objects_dict to map fake_static_9999 to body_id 9999
        from unittest.mock import MagicMock

        fake_obj = MagicMock()
        fake_obj.body_id = 9999
        sim_core._sim_objects_dict["fake_static_9999"] = fake_obj

        rendering_states = []

        def fake_configure(flag, enable, **kw):
            if flag == pb.COV_ENABLE_RENDERING:
                rendering_states.append(enable)

        call_count = [0]

        def fake_change(*args, **kwargs):
            call_count[0] += 1
            if call_count[0] == 2:
                raise RuntimeError("simulated GPU error")

        monkeypatch.setattr(pb, "configureDebugVisualizer", fake_configure)
        monkeypatch.setattr(pb, "changeVisualShape", fake_change)

        # Should not raise despite internal error
        sim_core._set_structure_transparency(True)

        # Rendering must be re-enabled (last state = 1)
        assert rendering_states[-1] == 1, f"Rendering should be re-enabled after exception, states: {rendering_states}"


# ---------------------------------------------------------------------------
# setup_camera
# ---------------------------------------------------------------------------


class TestSetupCamera:
    """Test setup_camera behavior."""

    def test_auto_mode_no_print_output(self, sim_core, capsys, monkeypatch):
        """auto mode should use logger, not print."""
        import pybullet as pb

        monkeypatch.setattr(pb, "resetDebugVisualizerCamera", lambda **kw: None)
        sim_core._params.gui = True
        # Place two objects so extent > 0
        make_box(sim_core, [0, 0, 0])
        make_box(sim_core, [5, 5, 0])

        sim_core.setup_camera(camera_config={"camera_mode": "auto"})

        captured = capsys.readouterr().out
        assert "Camera View" not in captured, "auto mode should use logger.info, not print"
        assert "extent" not in captured, "auto mode should use logger.info, not print"

    def test_unknown_camera_mode_warns(self, sim_core, caplog, monkeypatch):
        """Unknown camera_mode should log a warning."""
        import logging

        sim_core._params.gui = True

        with caplog.at_level(logging.WARNING, logger="pybullet_fleet.core_simulation"):
            sim_core.setup_camera(camera_config={"camera_mode": "autto"})

        assert any("autto" in r.message for r in caplog.records), "Should warn about unknown camera_mode"

    def test_auto_mode_minimum_distance(self, sim_core, monkeypatch):
        """auto mode should enforce a minimum distance when all objects are co-located."""
        import pybullet as pb

        captured_kwargs = {}

        def fake_reset(**kwargs):
            captured_kwargs.update(kwargs)

        monkeypatch.setattr(pb, "resetDebugVisualizerCamera", fake_reset)
        sim_core._params.gui = True

        # All objects at the same position → extent = [0,0,0]
        sim_core.setup_camera(
            camera_config={"camera_mode": "auto"},
            entity_positions=[[3, 3, 0], [3, 3, 0], [3, 3, 0]],
        )

        assert (
            captured_kwargs.get("cameraDistance", 0) > 0
        ), f"Camera distance should never be 0, got {captured_kwargs.get('cameraDistance')}"


# ============================================================================
# initialize_simulation
# ============================================================================


class TestInitializeSimulation:
    """Test initialize_simulation resets all stale state for a fresh run."""

    def test_resets_simulation_paused(self, sim_core):
        """#1: _simulation_paused must be reset so step_once doesn't skip."""
        sim_core._simulation_paused = True
        sim_core.initialize_simulation()
        assert sim_core._simulation_paused is False

    def test_clears_speed_history(self, sim_core):
        """#2: _speed_history deque must be emptied for accurate speed calculation."""
        import time

        sim_core._speed_history.append((time.time(), 1.0))
        sim_core._speed_history.append((time.time(), 2.0))
        assert len(sim_core._speed_history) > 0

        sim_core.initialize_simulation()
        assert len(sim_core._speed_history) == 0

    def test_resets_callback_last_exec(self, sim_core):
        """#3: callback last_exec must be reset to 0.0 for correct dt on restart."""
        called_dts = []

        def cb(sc, dt):
            called_dts.append(dt)

        sim_core.register_callback(cb, frequency=10.0)
        # Simulate previous run leaving stale last_exec
        sim_core._callbacks[0]["last_exec"] = 999.0

        sim_core.initialize_simulation()
        assert sim_core._callbacks[0]["last_exec"] == 0.0

    def test_clears_profiling_stats(self, sim_core):
        """#4: _profiling_stats lists must be emptied to avoid mixing runs."""
        sim_core._profiling_stats["total"].append(1.23)
        sim_core._profiling_stats["agent_update"].append(0.5)

        sim_core.initialize_simulation()
        for key, vals in sim_core._profiling_stats.items():
            assert len(vals) == 0, f"_profiling_stats['{key}'] not cleared"

    def test_resets_memory_stats(self, sim_core):
        """#5: _memory_stats must be reset to initial values."""
        sim_core._memory_stats["count"] = 100
        sim_core._memory_stats["current_max"] = 999.0
        sim_core._memory_stats["peak_max"] = 888.0

        sim_core.initialize_simulation()
        assert sim_core._memory_stats["count"] == 0
        assert sim_core._memory_stats["current_max"] == 0.0
        assert sim_core._memory_stats["current_min"] == float("inf")

    def test_resets_last_ignore_static(self, sim_core):
        """#6: _last_ignore_static must be None for fresh mode-change detection."""
        sim_core._last_ignore_static = True

        sim_core.initialize_simulation()
        assert sim_core._last_ignore_static is None

    def test_counters_reset(self, sim_core):
        """Basic counters should all be zeroed."""
        sim_core._step_count = 100
        sim_core._collision_count = 50
        sim_core.sim_time = 99.9

        sim_core.initialize_simulation()
        assert sim_core._step_count == 0
        assert sim_core._collision_count == 0
        assert sim_core.sim_time == 0.0


# ============================================================================
# run_simulation
# ============================================================================


class TestRunSimulation:
    """Test run_simulation uses module logger, not root logging."""

    def test_no_root_logging_calls_in_run_simulation(self):
        """run_simulation must use logger.xxx(), not logging.xxx()."""
        import inspect
        from pybullet_fleet.core_simulation import MultiRobotSimulationCore

        source = inspect.getsource(MultiRobotSimulationCore.run_simulation)
        # Find bare logging.info/warning/error/debug calls (not logger.xxx)
        import re

        # Match 'logging.info', 'logging.warning', etc. but not 'logger.info'
        matches = re.findall(r"\blogging\.(info|warning|error|debug)\b", source)
        assert matches == [], (
            f"run_simulation uses root logging module directly: {matches}. " "Should use module-level 'logger' instead."
        )


# ============================================================================
# step_once ordering & dead code
# ============================================================================


class TestStepOnceCodeQuality:
    """Source-level checks for step_once correctness."""

    def test_physics_aabb_update_after_step_simulation(self):
        """#1 HIGH: Physics object AABB/grid update must happen AFTER stepSimulation().

        If AABB update runs before stepSimulation(), collision detection uses
        1-frame-old positions for physics objects — a correctness bug.
        """
        import inspect
        import re
        from pybullet_fleet.core_simulation import MultiRobotSimulationCore

        source = inspect.getsource(MultiRobotSimulationCore.step_once)

        # Find the position of stepSimulation() call
        step_sim_match = re.search(r"p\.stepSimulation\(\)", source)
        assert step_sim_match is not None, "stepSimulation() call not found in step_once"

        # Find the physics-object AABB update loop
        # Pattern: "for obj_id in self._physics_objects:" followed by "_update_object_aabb"
        aabb_update_match = re.search(
            r"for obj_id in self\._physics_objects:.*?_update_object_aabb",
            source,
            re.DOTALL,
        )
        assert aabb_update_match is not None, "_update_object_aabb loop not found in step_once"

        # AABB update must come AFTER stepSimulation()
        assert aabb_update_match.start() > step_sim_match.start(), (
            "Physics object AABB/grid update runs BEFORE stepSimulation(). "
            "This means collision detection uses 1-frame-old positions. "
            "Move the AABB update block to after p.stepSimulation()."
        )

    def test_no_unused_collision_timings_variable(self):
        """#2 LOW: collision_timings should not be assigned if never read."""
        import inspect
        import re
        from pybullet_fleet.core_simulation import MultiRobotSimulationCore

        source = inspect.getsource(MultiRobotSimulationCore.step_once)

        # Check for "collision_timings" variable assignment (not underscore)
        # The variable is assigned but never consumed — should use _ or be removed
        assignments = re.findall(r"\bcollision_timings\b", source)
        assert len(assignments) == 0, (
            f"Found {len(assignments)} references to 'collision_timings' in step_once. "
            "This variable is assigned but never read. Remove it or replace with '_'."
        )


# ============================================================================
# Whole-file code quality
# ============================================================================


class TestCoreSimulationCodeQuality:
    """Source-level checks for core_simulation.py overall quality."""

    def test_no_mutable_default_arguments(self):
        """#1 HIGH: __init__ must not use mutable default arguments (list/dict).

        Mutable defaults are shared across all call sites, so mutating the
        default value affects every future caller.
        """
        import inspect
        from pybullet_fleet.core_simulation import MultiRobotSimulationCore

        sig = inspect.signature(MultiRobotSimulationCore.__init__)
        for name, param in sig.parameters.items():
            if name == "self":
                continue
            if isinstance(param.default, (list, dict, set)):
                raise AssertionError(
                    f"Parameter '{name}' has mutable default {param.default!r}. "
                    "Use None and assign inside __init__ instead."
                )

    def test_no_dead_set_profiling_log_frequency(self):
        """#2 MED: set_profiling_log_frequency sets _profiling_log_frequency,
        but step_once uses _profiling_interval. The method is dead/ineffective.
        It should either be removed or fixed to update _profiling_interval.
        """
        from pybullet_fleet.core_simulation import MultiRobotSimulationCore

        assert not hasattr(MultiRobotSimulationCore, "set_profiling_log_frequency"), (
            "set_profiling_log_frequency still exists but sets _profiling_log_frequency "
            "which is never read. step_once uses _profiling_interval instead. "
            "Remove this dead method or fix it to update _profiling_interval."
        )

    def test_no_unused_plane_id(self):
        """#4 MED: _plane_id is assigned but never referenced."""
        import inspect
        import re
        from pybullet_fleet.core_simulation import MultiRobotSimulationCore

        source = inspect.getsource(MultiRobotSimulationCore)
        # Find all references to _plane_id
        refs = re.findall(r"\b_plane_id\b", source)
        # Should have 0 references (assignment removed entirely)
        # or if kept, must have at least 2 (assignment + usage)
        assert len(refs) == 0, (
            f"Found {len(refs)} reference(s) to '_plane_id'. " "It is assigned but never read. Remove the variable assignment."
        )

    def test_statistics_imported_at_top_level(self):
        """#5 LOW: import statistics should be at module level, not inside method."""
        import inspect
        from pybullet_fleet.core_simulation import MultiRobotSimulationCore

        source = inspect.getsource(MultiRobotSimulationCore._print_profiling_summary)
        # Should NOT contain 'import statistics' inside the method
        assert "import statistics" not in source, (
            "_print_profiling_summary has a deferred 'import statistics'. "
            "Move it to the top-level imports for consistency and performance."
        )


# ============================================================================
# filter_aabb_pairs
# ============================================================================


class TestFilterAabbPairs:
    """Test filter_aabb_pairs collision candidate filtering."""

    def test_overlapping_objects_returned_as_pair(self, sim_core):
        """Two boxes placed close together should be returned as a candidate pair."""
        obj1 = make_box(sim_core, [0, 0, 0])
        obj2 = make_box(sim_core, [0.3, 0, 0])
        sim_core._moved_this_step.add(obj1.object_id)

        pairs, _ = sim_core.filter_aabb_pairs()

        expected = (min(obj1.object_id, obj2.object_id), max(obj1.object_id, obj2.object_id))
        assert expected in pairs

    def test_distant_objects_not_returned(self, sim_core):
        """Two boxes far apart should NOT appear in candidate pairs."""
        obj1 = make_box(sim_core, [0, 0, 0])
        obj2 = make_box(sim_core, [100, 100, 0])
        sim_core._moved_this_step.add(obj1.object_id)

        pairs, _ = sim_core.filter_aabb_pairs()

        pair = (min(obj1.object_id, obj2.object_id), max(obj1.object_id, obj2.object_id))
        assert pair not in pairs

    def test_no_moved_objects_returns_empty(self, sim_core):
        """If no objects moved, filter should return no pairs."""
        make_box(sim_core, [0, 0, 0])
        make_box(sim_core, [0.3, 0, 0])
        sim_core._moved_this_step.clear()

        pairs, _ = sim_core.filter_aabb_pairs()
        assert pairs == []

    def test_disabled_objects_excluded(self, sim_core):
        """DISABLED collision mode objects should never appear in pairs."""
        normal = make_box(sim_core, [0, 0, 0], collision_mode=CollisionMode.NORMAL_3D)
        disabled = make_box(sim_core, [0.3, 0, 0], collision_mode=CollisionMode.DISABLED)
        sim_core._moved_this_step.add(normal.object_id)

        pairs, _ = sim_core.filter_aabb_pairs()

        for a, b in pairs:
            assert a != disabled.object_id and b != disabled.object_id

    def test_ignore_static_skips_static_pairs(self, sim_core):
        """With ignore_static=True, static objects should be excluded from pairs."""
        static = make_box(sim_core, [0, 0, 0], collision_mode=CollisionMode.STATIC)
        normal = make_box(sim_core, [0.3, 0, 0], collision_mode=CollisionMode.NORMAL_3D)
        sim_core._moved_this_step.add(normal.object_id)

        pairs, _ = sim_core.filter_aabb_pairs(ignore_static=True)

        pair = (min(static.object_id, normal.object_id), max(static.object_id, normal.object_id))
        assert pair not in pairs

    def test_ignore_static_false_includes_static(self, sim_core):
        """With ignore_static=False, static+normal overlapping pair should be included."""
        static = make_box(sim_core, [0, 0, 0], collision_mode=CollisionMode.STATIC)
        normal = make_box(sim_core, [0.3, 0, 0], collision_mode=CollisionMode.NORMAL_3D)
        sim_core._moved_this_step.add(normal.object_id)

        pairs, _ = sim_core.filter_aabb_pairs(ignore_static=False)

        pair = (min(static.object_id, normal.object_id), max(static.object_id, normal.object_id))
        assert pair in pairs

    def test_2d_mode_ignores_z_separation(self, sim_core):
        """Two NORMAL_2D objects at same XY but Z-separated (within same grid cell)
        should be detected because 2D mode skips the Z-axis AABB overlap check.
        Use Z=1.5 so both objects stay in the same spatial grid Z-cell (cell_size=2.0)
        but their Z AABBs (size=0.25) do NOT overlap."""
        obj1 = make_box(sim_core, [0, 0, 0], collision_mode=CollisionMode.NORMAL_2D)
        obj2 = make_box(sim_core, [0, 0, 1.5], collision_mode=CollisionMode.NORMAL_2D)
        sim_core._moved_this_step.add(obj1.object_id)

        pairs, _ = sim_core.filter_aabb_pairs()

        pair = (min(obj1.object_id, obj2.object_id), max(obj1.object_id, obj2.object_id))
        assert pair in pairs, "2D mode should ignore Z-axis separation"

    def test_3d_mode_respects_z_separation(self, sim_core):
        """Two NORMAL_3D objects separated only in Z should NOT overlap."""
        obj1 = make_box(sim_core, [0, 0, 0], collision_mode=CollisionMode.NORMAL_3D)
        obj2 = make_box(sim_core, [0, 0, 5], collision_mode=CollisionMode.NORMAL_3D)
        sim_core._moved_this_step.add(obj1.object_id)

        pairs, _ = sim_core.filter_aabb_pairs()

        pair = (min(obj1.object_id, obj2.object_id), max(obj1.object_id, obj2.object_id))
        assert pair not in pairs, "3D mode should respect Z-axis separation"

    def test_return_profiling_includes_timings(self, sim_core):
        """return_profiling=True should return a dict with timing keys."""
        make_box(sim_core, [0, 0, 0])
        sim_core._moved_this_step.add(list(sim_core._sim_objects_dict.keys())[0])

        pairs, timings = sim_core.filter_aabb_pairs(return_profiling=True)

        assert timings is not None
        assert "get_aabbs" in timings
        assert "spatial_hashing" in timings
        assert "aabb_filtering" in timings

    def test_return_profiling_false_returns_none(self, sim_core):
        """return_profiling=False should return None for timings."""
        make_box(sim_core, [0, 0, 0])

        _, timings = sim_core.filter_aabb_pairs(return_profiling=False)
        assert timings is None

    def test_mode_change_forces_full_recalculation(self, sim_core):
        """Changing ignore_static should clear active_collision_pairs and recheck all."""
        static = make_box(sim_core, [0, 0, 0], collision_mode=CollisionMode.STATIC)
        normal = make_box(sim_core, [0.3, 0, 0], collision_mode=CollisionMode.NORMAL_3D)

        # First call with ignore_static=False — sets _last_ignore_static
        sim_core._moved_this_step.add(normal.object_id)
        sim_core.filter_aabb_pairs(ignore_static=False)

        # Clear moved set to simulate "nothing moved"
        sim_core._moved_this_step.clear()

        # Switch to ignore_static=True — should force full recalculation
        pairs, _ = sim_core.filter_aabb_pairs(ignore_static=True)

        # Static+normal pair should NOT be in results because ignore_static=True
        pair = (min(static.object_id, normal.object_id), max(static.object_id, normal.object_id))
        assert pair not in pairs


# ============================================================================
# _calculate_cell_size_from_aabbs
# ============================================================================


class TestCalculateCellSizeFromAabbs:
    """Test _calculate_cell_size_from_aabbs cell size calculation."""

    def test_returns_none_for_empty_aabbs(self, sim_core):
        """Empty AABB list should return None."""
        result = sim_core._calculate_cell_size_from_aabbs([])
        assert result is None

    def test_returns_none_when_no_cached_aabbs(self, sim_core):
        """No objects in sim → None when called without explicit aabbs."""
        assert len(sim_core._cached_aabbs_dict) == 0
        result = sim_core._calculate_cell_size_from_aabbs()
        assert result is None

    def test_single_object_returns_valid_cell_size(self, sim_core):
        """Single AABB should return a positive cell size."""
        aabbs = [((-0.5, -0.5, -0.5), (0.5, 0.5, 0.5))]
        result = sim_core._calculate_cell_size_from_aabbs(aabbs)
        assert result is not None
        assert result > 0

    def test_cell_size_at_least_min_floor(self, sim_core):
        """Cell size should be at least 0.5 even for tiny objects."""
        tiny_aabbs = [((-0.01, -0.01, -0.01), (0.01, 0.01, 0.01))]
        result = sim_core._calculate_cell_size_from_aabbs(tiny_aabbs)
        assert result >= 0.5, f"Cell size {result} should be >= 0.5"

    def test_cell_size_scales_with_object_size(self, sim_core):
        """Larger objects should produce larger cell sizes."""
        small_aabbs = [((-0.5, -0.5, -0.5), (0.5, 0.5, 0.5))]
        large_aabbs = [((-5.0, -5.0, -5.0), (5.0, 5.0, 5.0))]

        small_size = sim_core._calculate_cell_size_from_aabbs(small_aabbs)
        large_size = sim_core._calculate_cell_size_from_aabbs(large_aabbs)

        assert large_size > small_size

    def test_uses_cached_aabbs_when_none(self, sim_core):
        """When aabbs=None, should use cached AABBs from sim objects."""
        obj = make_box(sim_core, [0, 0, 0], size=1.0)
        assert len(sim_core._cached_aabbs_dict) > 0

        result = sim_core._calculate_cell_size_from_aabbs()
        assert result is not None
        assert result > 0


# ============================================================================
# get_aabbs
# ============================================================================


class TestGetAabbs:
    """Test get_aabbs returns correct AABB data."""

    def test_returns_empty_for_no_objects(self, sim_core):
        """No objects should return empty list."""
        result = sim_core.get_aabbs()
        assert result == []

    def test_returns_aabb_per_object(self, sim_core):
        """Should return one AABB per registered object."""
        make_box(sim_core, [0, 0, 0])
        make_box(sim_core, [5, 5, 0])
        make_box(sim_core, [10, 10, 0])

        result = sim_core.get_aabbs()
        assert len(result) == 3

    def test_aabb_format_is_min_max_tuple(self, sim_core):
        """Each AABB should be ((min_x, min_y, min_z), (max_x, max_y, max_z))."""
        make_box(sim_core, [0, 0, 0])
        result = sim_core.get_aabbs()
        assert len(result) == 1
        aabb = result[0]
        assert len(aabb) == 2  # (min, max)
        assert len(aabb[0]) == 3  # min_x, min_y, min_z
        assert len(aabb[1]) == 3  # max_x, max_y, max_z
        # min should be <= max for all axes
        for i in range(3):
            assert aabb[0][i] <= aabb[1][i]


# ============================================================================
# update_monitor
# ============================================================================


class TestUpdateMonitor:
    """Test update_monitor data reporting."""

    def test_uses_logger_not_print(self, sim_core, capsys):
        """update_monitor should use logger, not print."""
        sim_core._start_time = __import__("time").time()
        sim_core.update_monitor()

        captured = capsys.readouterr().out
        assert "[MONITOR]" not in captured, "Should use logger.debug, not print"

    def test_collision_count_logging(self, sim_core, caplog):
        """Should log NEW COLLISION message when collision_count increases."""
        import logging

        sim_core._start_time = __import__("time").time()
        sim_core._collision_count = 5
        sim_core._last_logged_collision_count = 0

        with caplog.at_level(logging.INFO, logger="pybullet_fleet.core_simulation"):
            sim_core.update_monitor()

        assert any("NEW COLLISION" in r.message for r in caplog.records)

    def test_no_collision_log_when_count_unchanged(self, sim_core, caplog):
        """Should NOT log NEW COLLISION when count doesn't change."""
        import logging

        sim_core._start_time = __import__("time").time()
        sim_core._collision_count = 5
        sim_core._last_logged_collision_count = 5

        with caplog.at_level(logging.INFO, logger="pybullet_fleet.core_simulation"):
            sim_core.update_monitor()

        assert not any("NEW COLLISION" in r.message for r in caplog.records)

    def test_speed_history_populated(self, sim_core):
        """update_monitor should append to _speed_history deque."""
        sim_core._start_time = __import__("time").time()
        assert len(sim_core._speed_history) == 0

        sim_core.update_monitor()
        assert len(sim_core._speed_history) == 1


# ============================================================================
# _print_profiling_summary
# ============================================================================


class TestPrintProfilingSummary:
    """Test _print_profiling_summary output."""

    def test_noop_when_no_stats(self, sim_core, caplog):
        """Should do nothing when profiling_stats is empty."""
        import logging

        with caplog.at_level(logging.INFO, logger="pybullet_fleet.core_simulation"):
            sim_core._print_profiling_summary()

        assert not any("[PROFILING]" in r.message for r in caplog.records)

    def test_logs_summary_when_stats_present(self, sim_core, caplog):
        """Should log a summary line when profiling data is available."""
        import logging

        # Populate profiling stats
        for key in sim_core._profiling_stats:
            sim_core._profiling_stats[key].append(1.0)

        with caplog.at_level(logging.INFO, logger="pybullet_fleet.core_simulation"):
            sim_core._print_profiling_summary()

        assert any("[PROFILING]" in r.message for r in caplog.records)

    def test_clears_stats_after_print(self, sim_core):
        """Should clear profiling_stats after printing summary."""
        for key in sim_core._profiling_stats:
            sim_core._profiling_stats[key].append(2.0)

        sim_core._print_profiling_summary()

        for key in sim_core._profiling_stats:
            assert len(sim_core._profiling_stats[key]) == 0, f"_profiling_stats['{key}'] should be cleared after summary"


# ============================================================================
# disable_rendering / enable_rendering
# ============================================================================


class TestRenderingControl:
    """Test disable_rendering and enable_rendering."""

    def test_disable_sets_flag(self, sim_core, monkeypatch):
        """disable_rendering should set _rendering_enabled to False."""
        import pybullet as pb

        monkeypatch.setattr(pb, "configureDebugVisualizer", lambda *a, **kw: None)
        sim_core._params.gui = True
        sim_core._rendering_enabled = True

        sim_core.disable_rendering()
        assert sim_core._rendering_enabled is False

    def test_enable_sets_flag(self, sim_core, monkeypatch):
        """enable_rendering should set _rendering_enabled to True."""
        import pybullet as pb

        monkeypatch.setattr(pb, "configureDebugVisualizer", lambda *a, **kw: None)
        sim_core._params.gui = True
        sim_core._rendering_enabled = False

        sim_core.enable_rendering()
        assert sim_core._rendering_enabled is True

    def test_enable_noop_when_already_enabled(self, sim_core, monkeypatch):
        """enable_rendering should be a no-op if already enabled."""
        calls = []

        import pybullet as pb

        monkeypatch.setattr(pb, "configureDebugVisualizer", lambda *a, **kw: calls.append(1))
        sim_core._params.gui = True
        sim_core._rendering_enabled = True

        sim_core.enable_rendering()
        assert len(calls) == 0, "Should not call pybullet API when already enabled"

    def test_disable_noop_when_not_gui(self, sim_core):
        """disable_rendering should be a no-op in headless mode."""
        sim_core._params.gui = False
        sim_core._rendering_enabled = True  # artificially set

        sim_core.disable_rendering()
        # Should not crash, and state should not change (gui guard)
        assert sim_core._rendering_enabled is True

    def test_enable_noop_when_not_gui(self, sim_core):
        """enable_rendering should be a no-op in headless mode."""
        sim_core._params.gui = False
        sim_core._rendering_enabled = False

        sim_core.enable_rendering()
        assert sim_core._rendering_enabled is False
