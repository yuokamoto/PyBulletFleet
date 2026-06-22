# Multi Fleet Support Specification

**Date:** 2026-04-18
**Status:** Validated
**Parent:** [ROS 2 Bridge v2 ROADMAP](../../../ros2_bridge/pybullet_fleet_ros/ROADMAP.md) — Phase 1, Item 1

## Context

Office demo は1フリート (tinyRobot ×2) で動作済み。
rmf_demos の airport_terminal (5 fleets), hotel (3 fleets), clinic (2 fleets) を
再現するには、launch file で複数の fleet_adapter インスタンスを起動する必要がある。

## Decision

`office_pybullet.launch.py` の構造を参考に、fleet ごとに fleet_adapter Node を追加する。
`fleet_adapter.py` のコード変更は不要 — launch file と config 追加のみ。
bridge_node は EventBus で動的にハンドラを追加するため対応済み。

## Requirements

- 1つの launch file から複数の fleet_adapter インスタンスを起動
- 各 fleet_adapter は独立した fleet config YAML と nav_graph を持つ
- bridge_node は1つのみ、全フリートのロボットを管理
- bridge_config YAML の `entities:` に複数フリートのロボットを列挙

## Constraints

- `pybullet_common.launch.py` は fleet_adapter を1つだけ起動する設計
- マルチフリート対応は demo-specific launch file 側で fleet_adapter を追加する
- fleet_adapter.py のコード変更なし

## Out of Scope

- フリート間の衝突回避 (RMF traffic schedule が担当)
- フリート固有のロボットモデル切替 (YAML config で解決)
- hotel/clinic デモの elevator 対応 (Item 6 で対応)

## Success Criteria

- [ ] airport_terminal 相当の launch file で5つの fleet_adapter が起動
- [ ] 各フリートのロボットが RMF タスクを受け付ける
- [ ] `ros2 topic echo /fleet_states` で全フリートが表示される
- [ ] 既存 office demo に影響なし
