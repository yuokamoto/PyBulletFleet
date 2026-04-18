"""Tests for CameraController — interactive camera control with right-drag panning."""

from unittest.mock import patch

import pybullet as p
import pytest


# --- Mouse event constants (mirror PyBullet API) ---
MOUSE_MOVE = 1
MOUSE_BUTTON = 2
BTN_LEFT = 0
BTN_MIDDLE = 1
BTN_RIGHT = 2
STATE_UP = 0
STATE_DOWN = 3  # PyBullet reports button press as state=3 (not 1)
STATE_RELEASED = 4


@pytest.mark.parametrize("key", ["=", "-", "o"])
class TestHelpers:
    """Test _pressed and _triggered helper functions across all used keys."""

    def test_returns_true_for_matching_state(self, key):
        from pybullet_fleet.camera_controller import _pressed, _triggered

        keys_down = {ord(key): p.KEY_IS_DOWN}
        assert _pressed(keys_down, ord(key)) is True

        keys_triggered = {ord(key): p.KEY_WAS_TRIGGERED}
        assert _triggered(keys_triggered, ord(key)) is True

    def test_returns_false_for_wrong_state(self, key):
        from pybullet_fleet.camera_controller import _pressed, _triggered

        keys_released = {ord(key): p.KEY_WAS_RELEASED}
        assert _pressed(keys_released, ord(key)) is False

        keys_held = {ord(key): p.KEY_IS_DOWN}
        assert _triggered(keys_held, ord(key)) is False

    def test_returns_false_when_key_absent(self, key):
        from pybullet_fleet.camera_controller import _pressed, _triggered

        keys = {}
        assert _pressed(keys, ord(key)) is False
        assert _triggered(keys, ord(key)) is False


# getDebugVisualizerCamera returns a tuple where:
# index 8 = yaw, 9 = pitch, 10 = distance, 11 = target position
_MOCK_CAMERA = (None,) * 8 + (0.0, -45.0, 10.0, (0.0, 0.0, 0.0))


def _call_update(controller, keys, mouse_events=None, camera=_MOCK_CAMERA):
    """Helper: call controller.update with mocked PyBullet, return resetCamera args."""
    with patch("pybullet_fleet.camera_controller.p.getDebugVisualizerCamera", return_value=camera):
        with patch("pybullet_fleet.camera_controller.p.resetDebugVisualizerCamera") as mock_reset:
            controller.update(keys, mouse_events=mouse_events or [])
            return mock_reset.call_args


def _extract(call_args):
    """Extract (dist, yaw, pitch, target) from resetDebugVisualizerCamera call."""
    a = call_args[0]
    return a[0], a[1], a[2], a[3]


class TestRightDragPan:
    """Test right-drag camera panning (no Shift needed)."""

    @pytest.fixture()
    def controller(self):
        from pybullet_fleet.camera_controller import CameraController

        return CameraController(client_id=0)

    def test_right_drag_pans_camera(self, controller):
        """Right-drag should move camera target without needing Shift."""
        no_keys = {}
        mouse_start = [(MOUSE_BUTTON, 100, 200, BTN_RIGHT, STATE_DOWN)]
        _call_update(controller, no_keys, mouse_events=mouse_start)
        mouse_drag = [(MOUSE_MOVE, 150, 200, -1, STATE_UP)]
        result = _call_update(controller, no_keys, mouse_events=mouse_drag)
        _, _, _, target = _extract(result)
        assert target[0] < 0.0, "At yaw=0, drag right should move target -X"

    def test_right_drag_up_pans_camera_up(self, controller):
        """Dragging up at pitch=-45 should move target +Z and -Y."""
        no_keys = {}
        mouse_start = [(MOUSE_BUTTON, 200, 200, BTN_RIGHT, STATE_DOWN)]
        _call_update(controller, no_keys, mouse_events=mouse_start)
        mouse_drag = [(MOUSE_MOVE, 200, 150, -1, STATE_UP)]
        result = _call_update(controller, no_keys, mouse_events=mouse_drag)
        _, _, _, target = _extract(result)
        assert target[2] < 0.0, "Drag up should move view up (target -Z)"
        assert target[1] < 0.0, "At pitch=-45, drag up also moves -Y"

    def test_camera_local_pan_with_rotated_yaw(self, controller):
        """At yaw=90, horizontal drag should move along Y (not X)."""
        no_keys = {}
        rotated_cam = (None,) * 8 + (90.0, -45.0, 10.0, (0.0, 0.0, 0.0))
        _call_update(controller, no_keys, mouse_events=[(MOUSE_BUTTON, 100, 200, BTN_RIGHT, STATE_DOWN)], camera=rotated_cam)
        result = _call_update(
            controller,
            no_keys,
            mouse_events=[(MOUSE_MOVE, 150, 200, -1, STATE_UP)],
            camera=rotated_cam,
        )
        _, _, _, target = _extract(result)
        assert abs(target[1]) > abs(target[0]), "At yaw=90, horizontal drag should mainly move Y"

    def test_left_drag_does_not_pan(self, controller):
        """Left-drag should NOT trigger pan (left-drag is native rotation)."""
        no_keys = {}
        mouse_start = [(MOUSE_BUTTON, 100, 200, BTN_LEFT, STATE_DOWN)]
        _call_update(controller, no_keys, mouse_events=mouse_start)
        mouse_drag = [(MOUSE_MOVE, 150, 200, -1, STATE_UP)]
        result = _call_update(controller, no_keys, mouse_events=mouse_drag)
        assert result is None, "Left-drag should not trigger pan"

    def test_button_release_stops_panning(self, controller):
        """After right-button release, further moves should not pan."""
        no_keys = {}
        _call_update(controller, no_keys, mouse_events=[(MOUSE_BUTTON, 100, 200, BTN_RIGHT, STATE_DOWN)])
        _call_update(controller, no_keys, mouse_events=[(MOUSE_BUTTON, 100, 200, BTN_RIGHT, STATE_RELEASED)])
        result = _call_update(controller, no_keys, mouse_events=[(MOUSE_MOVE, 200, 300, -1, STATE_UP)])
        assert result is None, "Should not pan after button release"

    def test_no_mouse_events_no_pan(self, controller):
        """No mouse events should not call resetDebugVisualizerCamera."""
        result = _call_update(controller, {}, mouse_events=[])
        assert result is None, "No events should not call resetDebugVisualizerCamera"

    def test_drag_scales_with_distance(self, controller):
        """Pan amount should scale with camera distance (farther = faster pan)."""
        from pybullet_fleet.camera_controller import CameraController

        no_keys = {}
        # Close camera (dist=5)
        close_cam = (None,) * 8 + (0.0, -45.0, 5.0, (0.0, 0.0, 0.0))
        _call_update(controller, no_keys, mouse_events=[(MOUSE_BUTTON, 100, 200, BTN_RIGHT, STATE_DOWN)], camera=close_cam)
        result_close = _call_update(controller, no_keys, mouse_events=[(MOUSE_MOVE, 150, 200, -1, STATE_UP)], camera=close_cam)
        _, _, _, target_close = _extract(result_close)

        controller2 = CameraController(client_id=0)
        far_cam = (None,) * 8 + (0.0, -45.0, 20.0, (0.0, 0.0, 0.0))
        _call_update(controller2, no_keys, mouse_events=[(MOUSE_BUTTON, 100, 200, BTN_RIGHT, STATE_DOWN)], camera=far_cam)
        result_far = _call_update(controller2, no_keys, mouse_events=[(MOUSE_MOVE, 150, 200, -1, STATE_UP)], camera=far_cam)
        _, _, _, target_far = _extract(result_far)

        assert abs(target_far[0]) > abs(target_close[0]), "Farther camera should pan faster"

    def test_pan_stable_despite_target_drift(self, controller):
        """Pan must compute from drag-start target, not PyBullet's drifted per-frame target."""
        no_keys = {}
        start_cam = (None,) * 8 + (0.0, -45.0, 10.0, (0.0, 0.0, 0.0))
        _call_update(controller, no_keys, mouse_events=[(MOUSE_BUTTON, 100, 200, BTN_RIGHT, STATE_DOWN)], camera=start_cam)
        drifted_cam = (None,) * 8 + (0.0, -45.0, 10.0, (1.0, 1.0, 0.0))
        result = _call_update(controller, no_keys, mouse_events=[(MOUSE_MOVE, 150, 200, -1, STATE_UP)], camera=drifted_cam)
        _, _, _, target = _extract(result)
        assert abs(target[1]) < 0.1, f"Pan should not inherit target drift, but got target[1]={target[1]}"

    def test_multi_frame_drag_consistent(self, controller):
        """Dragging across multiple frames must produce the same result as a single large drag."""
        from pybullet_fleet.camera_controller import CameraController

        no_keys = {}
        cam = (None,) * 8 + (0.0, -45.0, 10.0, (0.0, 0.0, 0.0))

        ctrl_single = CameraController(client_id=0)
        _call_update(ctrl_single, no_keys, mouse_events=[(MOUSE_BUTTON, 100, 200, BTN_RIGHT, STATE_DOWN)], camera=cam)
        result_single = _call_update(ctrl_single, no_keys, mouse_events=[(MOUSE_MOVE, 200, 250, -1, STATE_UP)], camera=cam)
        _, _, _, target_single = _extract(result_single)

        ctrl_multi = CameraController(client_id=0)
        _call_update(ctrl_multi, no_keys, mouse_events=[(MOUSE_BUTTON, 100, 200, BTN_RIGHT, STATE_DOWN)], camera=cam)
        _call_update(ctrl_multi, no_keys, mouse_events=[(MOUSE_MOVE, 150, 225, -1, STATE_UP)], camera=cam)
        result_multi = _call_update(ctrl_multi, no_keys, mouse_events=[(MOUSE_MOVE, 200, 250, -1, STATE_UP)], camera=cam)
        _, _, _, target_multi = _extract(result_multi)

        assert target_single[0] == pytest.approx(target_multi[0], abs=1e-6), "X should match single vs multi-frame"
        assert target_single[1] == pytest.approx(target_multi[1], abs=1e-6), "Y should match single vs multi-frame"
        assert target_single[2] == pytest.approx(target_multi[2], abs=1e-6), "Z should match single vs multi-frame"


class TestKeyboardControls:
    """Test keyboard-only controls (zoom, top-down, no-keys)."""

    @pytest.fixture()
    def controller(self):
        from pybullet_fleet.camera_controller import CameraController

        return CameraController(client_id=0)

    def test_zoom_in_decreases_distance(self, controller):
        dist, yaw, pitch, target = _extract(_call_update(controller, {ord("="): p.KEY_IS_DOWN}))
        assert dist < 10.0, "= key should zoom in (decrease distance)"

    def test_zoom_out_increases_distance(self, controller):
        dist, yaw, pitch, target = _extract(_call_update(controller, {ord("-"): p.KEY_IS_DOWN}))
        assert dist > 10.0, "- key should zoom out (increase distance)"

    def test_zoom_in_respects_minimum_distance(self, controller):
        close_camera = (None,) * 8 + (0.0, -45.0, 1.5, (0.0, 0.0, 0.0))
        dist, yaw, pitch, target = _extract(_call_update(controller, {ord("="): p.KEY_IS_DOWN}, camera=close_camera))
        assert dist >= 1.0, "Zoom should not go below minimum distance"

    def test_topdown_sets_pitch_and_yaw(self, controller):
        dist, yaw, pitch, target = _extract(_call_update(controller, {ord("o"): p.KEY_WAS_TRIGGERED}))
        assert yaw == 0
        assert pitch == pytest.approx(-89.9, abs=0.2)

    def test_no_keys_no_change(self, controller):
        """No input should NOT call resetDebugVisualizerCamera."""
        result = _call_update(controller, {})
        assert result is None, "No input should skip resetDebugVisualizerCamera"


class TestSetupCameraMode:
    """Interactive controls are always-on; valid modes are 'auto' and 'manual'."""

    def test_default_camera_mode_is_auto(self):
        """Default camera_mode should be 'auto'."""
        from pybullet_fleet import MultiRobotSimulationCore, SimulationParams

        params = SimulationParams(gui=False, monitor=False)
        sim = MultiRobotSimulationCore(params)
        try:
            sim.setup_camera({"camera_mode": "auto"})
        finally:
            p.disconnect(sim.client)
