"""Interactive camera control for PyBullet GUI.

Provides right-drag camera panning, keyboard zooming, and view presets.
Integrated via ``_handle_keyboard_events()`` in :class:`MultiRobotSimulationCore`.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import pybullet as p

from pybullet_fleet.logging_utils import get_lazy_logger

logger = get_lazy_logger(__name__)

# PyBullet mouse event constants
_MOUSE_MOVE = 1
_MOUSE_BUTTON = 2
_STATE_DOWN = 3  # button pressed / held (PyBullet never sends state=1 for mouse)
_STATE_RELEASED = 4
_BTN_RIGHT = 2

# Indices into the tuple returned by p.getDebugVisualizerCamera()
_CAM_YAW = 8
_CAM_PITCH = 9
_CAM_DIST = 10
_CAM_TARGET = 11

#: Minimum camera distance before zoom switches to target-forward mode.
_MIN_ZOOM_DIST = 2.0

# Type alias for a single mouse event tuple from PyBullet
MouseEvent = Tuple[int, int, int, int, int]


class CameraController:
    """Interactive camera control for PyBullet GUI.

    Call :meth:`update` each simulation step with the result of
    ``p.getKeyboardEvents()`` and ``p.getMouseEvents()`` to process camera movement.

    Controls:
    - **Right-drag** — pan camera target (no conflict with native left-drag rotation)
    - ``=`` / ``-`` — zoom in / out
    - ``o`` — top-down view
    """

    def __init__(self, client_id: int, drag_sensitivity: float = 0.01, zoom_speed: float = 1.0) -> None:
        self._cid = client_id
        self._drag_sensitivity = drag_sensitivity
        self._zoom_speed = zoom_speed
        # Drag state — accumulate from start for drift-free panning
        self._dragging = False
        self._drag_start_x: Optional[int] = None
        self._drag_start_y: Optional[int] = None
        self._drag_start_target: Optional[list] = None  # target position when drag began
        self._drag_yaw: Optional[float] = None
        self._drag_pitch: Optional[float] = None

    def update(self, keys: dict, mouse_events: Optional[List[MouseEvent]] = None) -> None:
        """Process keyboard + mouse events and update camera.

        Only calls ``resetDebugVisualizerCamera`` when our controls actually
        changed something.  This avoids interfering with PyBullet's native
        camera manipulation (left-drag rotation, scroll zoom) when idle.

        Args:
            keys: Result of ``p.getKeyboardEvents()``.
            mouse_events: Result of ``p.getMouseEvents()``.
        """
        cam = p.getDebugVisualizerCamera(physicsClientId=self._cid)
        dist: float = cam[_CAM_DIST]
        yaw: float = cam[_CAM_YAW]
        pitch: float = cam[_CAM_PITCH]
        target = list(cam[_CAM_TARGET])

        camera_changed = False

        # --- Right-drag → pan (camera-local) ---
        if mouse_events is not None:
            if self._process_mouse(mouse_events, target, dist, yaw, pitch):
                camera_changed = True

        # +/- → zoom (proportional: 10% per press, min step 0.5)
        # When distance is already small, move the *target* forward so the
        # camera keeps advancing — like a two-finger pinch zoom.
        if _pressed(keys, ord("=")):
            step = max(0.5, dist * 0.1 * self._zoom_speed)
            new_dist = dist - step
            if new_dist < _MIN_ZOOM_DIST:
                # Shift target forward (camera→target direction) instead of stopping
                overshoot = _MIN_ZOOM_DIST - new_dist
                rad_yaw = math.radians(yaw)
                rad_pitch = math.radians(pitch)
                cos_pitch = math.cos(rad_pitch)
                # Forward direction: camera → target (away from camera)
                fwd_x = -math.sin(rad_yaw) * cos_pitch
                fwd_y = math.cos(rad_yaw) * cos_pitch
                fwd_z = math.sin(rad_pitch)
                target[0] += overshoot * fwd_x
                target[1] += overshoot * fwd_y
                target[2] += overshoot * fwd_z
                new_dist = _MIN_ZOOM_DIST
            dist = new_dist
            camera_changed = True
        if _pressed(keys, ord("-")):
            step = max(0.5, dist * 0.1 * self._zoom_speed)
            dist += step
            camera_changed = True

        # 'o' → top-down view
        if _triggered(keys, ord("o")):
            pitch = -89.9  # not -90 to avoid gimbal lock
            yaw = 0
            camera_changed = True

        if camera_changed:
            p.resetDebugVisualizerCamera(dist, yaw, pitch, target, physicsClientId=self._cid)

    def _process_mouse(
        self,
        mouse_events: List[MouseEvent],
        target: list,
        dist: float,
        yaw: float,
        pitch: float,
    ) -> bool:
        """Process mouse events for right-drag panning in camera-local space.

        Right-drag does **not** conflict with PyBullet's native left-drag
        camera rotation, so no suppression flags are needed.

        Returns:
            True if the camera target was modified by a pan event.
        """
        changed = False
        for event in mouse_events:
            etype, mx, my, btn_idx, state = event

            if etype == _MOUSE_BUTTON and btn_idx == _BTN_RIGHT:
                if state == _STATE_DOWN and not self._dragging:
                    self._dragging = True
                    self._drag_start_x = mx
                    self._drag_start_y = my
                    self._drag_start_target = target[:]  # snapshot target at drag start
                    self._drag_yaw = yaw
                    self._drag_pitch = pitch
                elif state == _STATE_RELEASED:
                    self._dragging = False
                    self._drag_start_x = None
                    self._drag_start_y = None
                    self._drag_start_target = None
                    self._drag_yaw = None
                    self._drag_pitch = None

            elif etype == _MOUSE_MOVE and self._dragging:
                if self._drag_start_x is not None and self._drag_start_y is not None and self._drag_start_target is not None:
                    # Total delta from drag start (not incremental per-frame)
                    total_dx = mx - self._drag_start_x
                    total_dy = my - self._drag_start_y
                    scale = self._drag_sensitivity * dist

                    frozen_yaw = self._drag_yaw if self._drag_yaw is not None else yaw
                    frozen_pitch = self._drag_pitch if self._drag_pitch is not None else pitch
                    yaw_r = math.radians(frozen_yaw)
                    pitch_r = math.radians(frozen_pitch)

                    # Camera-local right vector (horizontal, verified against PyBullet view matrix)
                    right_x = math.cos(yaw_r)
                    right_y = math.sin(yaw_r)

                    # Camera-local up vector (tilts with pitch)
                    up_x = math.sin(yaw_r) * math.sin(pitch_r)
                    up_y = -math.cos(yaw_r) * math.sin(pitch_r)
                    up_z = math.cos(pitch_r)

                    # Compute absolute target from drag-start position.
                    base = self._drag_start_target
                    target[0] = base[0] + scale * (-total_dx * right_x + total_dy * up_x)
                    target[1] = base[1] + scale * (-total_dx * right_y + total_dy * up_y)
                    target[2] = base[2] + scale * (total_dy * up_z)
                    changed = True

        return changed


def _pressed(keys: dict, key: int) -> bool:
    """Key is held down (state includes KEY_IS_DOWN)."""
    return key in keys and bool(keys[key] & p.KEY_IS_DOWN)


def _triggered(keys: dict, key: int) -> bool:
    """Key was just pressed (state == KEY_WAS_TRIGGERED)."""
    return key in keys and bool(keys[key] & p.KEY_WAS_TRIGGERED)
