# Camera Control GUI Specification

**Date:** 2026-04-18
**Status:** Validated
**Parent:** [ROS 2 Bridge v2 ROADMAP](../../../ros2_bridge/pybullet_fleet_ros/ROADMAP.md) — Phase 1, Item 4

## Context

大規模マップ (airport_terminal, hotel) ではデフォルトカメラ位置だとロボットを追跡できない。
現在は `setup_camera()` で初期位置を設定するのみ。
ユーザーがキーボード/マウスでカメラをリアルタイム操作できるようにする。

## Decision

`pybullet_fleet/camera_controller.py` に CameraController クラスを新設。
PyBullet の `getKeyboardEvents()` / `getMouseEvents()` を使い、
既存の `_handle_keyboard_events()` (SPACE/t キー) を拡張する。

## Requirements

- Top-down orthographic ビュー切替 (キーバインド)
- Shift+矢印キーでカメラ平行移動
- +/- キーでズーム
- `camera_mode: "interactive"` を YAML config で設定可能

## Constraints

- `p.GUI` モードのみ (p.DIRECT では無効)
- `p.getKeyboardEvents()` は毎ステップ呼ぶ必要あり → `step_once()` 内で処理
- 既存の SPACE (pause) / 't' (transparency) キーバインドと共存

## Out of Scope

- フロア切替ボタン (Item 9: Multi-Floor Visualization で対応)
- GUI ウィジェット (PyBullet にはボタン/スライダー API なし)
- ROS 2 topic によるカメラ制御

## Success Criteria

- [ ] Shift+矢印でカメラ移動、+/-でズーム
- [ ] 'o' キーで top-down orthographic ビューに切替
- [ ] `camera_mode: "interactive"` が YAML config で有効化可能
- [ ] 既存 SPACE / 't' キーバインドが維持
- [ ] p.DIRECT モードではエラーなく無効化
