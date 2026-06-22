# sim_time Acceleration Specification

**Date:** 2026-04-18
**Status:** Validated
**Parent:** [ROS 2 Bridge v2 ROADMAP](../../../ros2_bridge/pybullet_fleet_ros/ROADMAP.md) — Phase 1, Item 5

## Context

PyBulletFleet はターゲット RTF > 1.0 (例: 10x) で実行可能。
bridge_node は既に `/clock` を publish しているが、
他のノード (fleet_adapter, door_adapter, workcell_adapter) は wall clock を使用。
結果、sim_time が 10x で進んでも ROS ノード側が追従できない。

Open-RMF は `use_sim_time` を "all or nothing" で要求する。
全ノードが同じ時間ソースを使わないとタスクのスケジューリングが壊れる。

## Decision

launch arg `use_sim_time:=true` を追加し、
全ノードに `use_sim_time` ROS パラメータを伝播する。

## Requirements

- `pybullet_common.launch.py` に `use_sim_time` launch argument を追加
- `bridge_node` — 既存 `/clock` publish を維持、`use_sim_time` パラメータ追加
- `fleet_adapter` — `-sim` flag を launch から渡す
- `door_adapter` — `use_sim_time` パラメータ追加、`self.get_clock()` は自動対応
- `workcell_adapter` — 同上
- `target_rtf > 1.0` 時に実際に加速されることを確認

## Constraints

- Open-RMF の `use_sim_time` は全ノードに適用しないと不整合が起きる
- `door_adapter` / `workcell_adapter` は `rclpy.Node` 基底 → `use_sim_time` ROS param を
  `True` にすれば `self.get_clock()` が自動的に sim_time を返す
- `fleet_adapter` は独自の `-sim` argparse flag で `adapter.node.use_sim_time()` を呼ぶ
- `/clock` の publish 頻度はシミュレーション step 毎 (bridge_node L204-206)
- デフォルトは `use_sim_time:=true` (高速化がメイン目的のため)

## Out of Scope

- target_rtf の動的変更 (Phase 2: Fast-Forward/Slow-Motion で対応)
- ライフサイクル管理で sim_time 設定を切り替え
- カスタム clock source

## Success Criteria

- [ ] デフォルト (`use_sim_time:=true`) で全ノードが sim_time を使用
- [ ] `use_sim_time:=false` で wall clock モードに切替可能
- [ ] `target_rtf: 5.0` で実際に Open-RMF タスクが 5x で実行される
- [ ] `/clock` トピックの stamp が sim_time と一致
