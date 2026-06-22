# bridge_node EventBus Refactor Specification

**Date:** 2026-04-18
**Status:** Validated
**Parent:** [ROS 2 Bridge v2 ROADMAP](../../../ros2_bridge/pybullet_fleet_ros/ROADMAP.md) — Phase 1, Item 2

## Context

現在の bridge_node は sim_core を所有し、ROS timer で `step_once()` を駆動する。
これにより bridge_node がシミュレーションの「主人」になっている。
Plugin パターン (Pattern 3/4)、Device plugins、ExternalAgent plugin は
sim_core がメインループを所有し、bridge_node が read-only observer になることを前提とする。

## Decision

bridge_node を EventBus ベースの observer に変更する。
現在の ROS timer 駆動 (owner モード) は廃止し、observer 一本化とする。
sim_core が `run_simulation()` でメインループを持ち、bridge_node は
`POST_STEP` イベントで ROS publish を行う。

Owner モードを廃止する理由:
- 2 コードパスの維持コストが高い
- ROS timer 駆動はタイミング精度が低い (ジッタ)
- Phase 2 Plugin パターンと互換性がない
- RTF 制御が sim_core に一元化される

## Requirements

- bridge_node を EventBus ベースの observer に全面移行
- `POST_STEP` イベントで全ロボットの状態を publish
- `/clock` publish を EventBus callback 内で throttle (100 Hz)
- sim_core は外部から注入 (`create_observer()` classmethod)
- sim_core の `run_simulation()` を別スレッドで実行

## Constraints

- bridge_node は ROS 2 ノード — `rclpy.spin()` が必要
- sim_core の `run_simulation()` はブロッキングループ → 別スレッドで実行
- `MultiThreadedExecutor` は既に使用中
- GIL 依存で十分 — スレッドセーフ化は不要

## Out of Scope

- Pattern 3 (Plugin Only) の実装 — EventBus refactor 完了後にフォロー
- Pattern 4 (Plugin + Bridge) の実装 — 同上
- sim_core のスレッドセーフ化 — 現状の GIL 依存を維持

## Success Criteria

- [ ] bridge_node が EventBus 経由で ROS publish を実行
- [ ] `/clock`, `/odom`, `/tf`, `/joint_states` が正しく publish される
- [ ] `NavigateToPose` action server が動作
- [ ] 既存テスト全通過
