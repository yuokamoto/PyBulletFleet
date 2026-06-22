# Camera Control GUI — Agent Specification

## Requirements

- `CameraController` クラスを `pybullet_fleet/camera_controller.py` に新設
- `core_simulation.py` の `_handle_keyboard_events()` から呼び出し
- `SimulationParams.camera_mode` に `"interactive"` 値を追加
- p.GUI モード以外ではスキップ

## Constraints

- `p.getKeyboardEvents()` は dict を返す: `{key_code: state}`
- `p.getMouseEvents()` はリストを返す: `[(event_type, mx, my, btn_idx, state), ...]`
- `p.getDebugVisualizerCamera()` で現在のカメラ状態を取得
- `p.resetDebugVisualizerCamera()` でカメラ設定
- `p.computeViewMatrix()` / `p.computeProjectionMatrix()` は使わない (resetDebugVisualizerCamera で十分)
- 既存の camera_mode: `"none"`, `"manual"`, `"auto"` と共存
- PyBullet が内部で矢印キーを消費するため、キーボードベースのパンは不可

## Approach

CameraController を step_once の keyboard event ハンドラに組み込む。
毎ステップ `getKeyboardEvents()` + `getMouseEvents()` を読み取り、
Shift+左ドラッグでカメラパン、+/- でズーム、'o' でトップダウン。

## Design

### CameraController

```python
# pybullet_fleet/camera_controller.py
class CameraController:
    """Interactive camera control for PyBullet GUI."""

    def __init__(self, client_id: int, drag_sensitivity: float = 0.005, zoom_speed: float = 1.0):
        self._cid = client_id
        self._drag_sensitivity = drag_sensitivity
        self._zoom_speed = zoom_speed
        self._dragging = False
        self._last_mouse_x = None
        self._last_mouse_y = None

    def update(self, keys: dict, mouse_events: list | None = None) -> None:
        """Process keyboard + mouse events and update camera.

        Args:
            keys: Result of p.getKeyboardEvents()
            mouse_events: Result of p.getMouseEvents()
        """
        cam = p.getDebugVisualizerCamera(physicsClientId=self._cid)
        dist, yaw, pitch, target = cam[10], cam[8], cam[9], list(cam[11])

        shift_held = _pressed(keys, p.B3G_SHIFT)

        # Shift + left-drag → pan (Gazebo-style)
        if mouse_events:
            self._process_mouse(mouse_events, shift_held, target, dist)

        # +/- → zoom
        if _pressed(keys, ord('=')):  # + key
            dist = max(1.0, dist - self._zoom_speed)
        if _pressed(keys, ord('-')):
            dist += self._zoom_speed

        # 'o' → top-down orthographic
        if _triggered(keys, ord('o')):
            pitch = -89.9
            yaw = 0

        p.resetDebugVisualizerCamera(dist, yaw, pitch, target,
                                     physicsClientId=self._cid)


def _pressed(keys, key):
    """Key is held down (state includes KEY_IS_DOWN)."""
    return key in keys and (keys[key] & p.KEY_IS_DOWN)

def _triggered(keys, key):
    """Key was just pressed (state == KEY_WAS_TRIGGERED)."""
    return key in keys and (keys[key] & p.KEY_WAS_TRIGGERED)
```

### Integration in core_simulation.py

```python
# In _handle_keyboard_events() (L1610-1645):
def _handle_keyboard_events(self):
    keys = p.getKeyboardEvents(physicsClientId=self.client)
    # Existing: SPACE = pause, 't' = transparency
    ...
    # New: camera controller with mouse events
    if self._camera_controller is not None:
        mouse_events = p.getMouseEvents(physicsClientId=self.client)
        self._camera_controller.update(keys, mouse_events=mouse_events)
```

### SimulationParams Changes

```python
# camera_mode: "none" | "manual" | "auto" | "interactive"
# When "interactive", CameraController is created in setup_camera()
```

## File References

- `pybullet_fleet/core_simulation.py:L1610-L1645` — `_handle_keyboard_events()`
- `pybullet_fleet/core_simulation.py:L1731-L1790` — `setup_camera()`
- `pybullet_fleet/core_simulation.py:L1530-L1579` — `configure_visualizer()`
- `pybullet_fleet/_defaults.py` — camera_mode のデフォルト値

## Success Criteria

- [x] `camera_controller.py` 新規ファイル作成
- [x] Shift+左ドラッグでカメラパン (Gazebo スタイル)
- [x] +/- でズーム
- [x] 'o' で top-down ビュー
- [x] `camera_mode: "interactive"` で有効化
- [x] p.DIRECT モードで CameraController がインスタンス化されない
- [x] 既存キーバインド (SPACE, 't') が維持
- [ ] `make verify` 通過
