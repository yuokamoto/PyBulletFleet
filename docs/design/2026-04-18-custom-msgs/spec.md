# Custom Message Package Specification

**Date:** 2026-04-18
**Status:** Validated
**Parent:** [ROS 2 Bridge v2 ROADMAP](../../../ros2_bridge/pybullet_fleet_ros/ROADMAP.md) — Phase 1, Item 3

## Context

現在の `pybullet_fleet_ros` は `ament_python` パッケージのため、
`.msg` / `.srv` / `.action` ファイルを定義できない。
Phase 2 の Batch API (Item 8) と ROS 2 Action Server (Item 10) で
カスタムメッセージが必要になる。

ただし、rmf_demos Docker イメージには `rmf_internal_msgs` が全て含まれており、
多くのユースケースは既存メッセージで対応可能。

## Decision

**方法 B: `pybullet_fleet_msgs` を `ament_cmake` で別パッケージ追加。**

理由:
- `pybullet_fleet_ros` の `setup.py` → `CMakeLists.txt` 移行を回避
- ROS 2 慣習に沿う (`nav2_msgs`, `rmf_fleet_msgs` 等と同じパターン)
- メッセージ定義とノード実装を分離

初期リリースは最小限。Phase 2 で必要になるメッセージのみ定義。

## Requirements

- `pybullet_fleet_msgs` パッケージを `ament_cmake` で作成
- 初期メッセージ: `ExecuteAction.action` (Phase 2 Item 10 用)
- `pybullet_fleet_ros` から `pybullet_fleet_msgs` に依存を追加
- Docker ビルドで `pybullet_fleet_msgs` → `pybullet_fleet_ros` の順でビルド

## Constraints

- rmf_demos Docker イメージにビルドツールが含まれていること
- `ament_cmake` パッケージは `colcon build` でビルド必須
- メッセージ定義は最小限 — 既存 rmf_internal_msgs を優先使用

## Out of Scope

- FleetStates.msg の独自定義 (rmf_fleet_msgs/FleetState で十分)
- カスタム srv の定義 (Phase 2 で必要になれば追加)

## Success Criteria

- [ ] `pybullet_fleet_msgs` パッケージが `colcon build` で成功
- [ ] `ExecuteAction.action` が定義され、Python から import 可能
- [ ] `pybullet_fleet_ros` の `package.xml` に依存追加
- [ ] Docker ビルドが成功
