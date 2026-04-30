# Device Base + Door/Elevator & ExternalAgent Plugin — Specification

**Date:** 2026-04-21
**Status:** Ready for Implementation
**ROADMAP items:** 6 (Device Base + Door/Elevator), 7 (ExternalAgent Plugin)

## Context

Phase 1 の EventBus refactor (item 2) が完了し、bridge_node は read-only observer として動作。
Phase 2 では infrastructure devices (Door, Elevator, Workcell) と
RMF 非管理の ExternalAgent を実装し、hotel / airport_terminal デモへの対応を可能にする。

## Decisions

### Device: Agent サブクラス + URDF ジョイント

Door/Elevator は Agent のサブクラスとして実装する。URDF の revolute/prismatic ジョイントで
開閉・昇降をシミュレーションし、既存の JointAction / ジョイント制御をフル活用する。

理由:
- JointAction のライフサイクル (NOT_STARTED → IN_PROGRESS → COMPLETED) がデバイス状態遷移に直結
- Agent のジョイント interpolation が自動でアニメーション処理
- fleet_adapter は fleet config YAML の `known_robots` のみ管理するため、
  Device を `_agents` に追加しても fleet_adapter への影響はゼロ

WorkcellDevice は移動・ジョイントが不要なため SimObject サブクラスとする。

### ExternalAgent: Controller チェーン

Agent に複数 Controller のスタックを導入する。
高レベル Controller (PatrolController 等) が `set_goal_pose()` を設定し、
低レベル Controller (KinematicController) が実移動を処理する。

専用の ExternalAgentPlugin は不要 — Controller チェーンが YAML で宣言的に設定されるため。

### SimObject.update() + _needs_update

SimObject に `update(dt)` メソッド (no-op default) と `_needs_update` フラグを追加。
`step_once()` を `_sim_objects` の統一ループに変更し、Agent も Device も同一パスで処理する。

### Elevator: ハイブリッド attach

`auto_attach=true` (デフォルト) でプラットフォーム上のオブジェクトを自動 attach。
手動 `attach_object()` / `detach_object()` も利用可能。

## Requirements

- DoorDevice: URDF revolute/prismatic ジョイントで開閉
- ElevatorDevice: URDF prismatic Z ジョイントでフロア間移動
- WorkcellDevice: 位置マーカー (PickAction の target_position として使用)
- PatrolController: ウェイポイント巡回 (loop 対応)
- RandomWalkController: ランダム近傍歩行
- Controller チェーン: 高レベル → 低レベル順に compute()
- 既存 `set_controller()` の後方互換性維持
- entity_registry で YAML スポーン対応
- Demo launch files & configs: 各 rmf_demos ワールドが docker compose で起動可能
  - battle_royale: 1 fleet × 4 robots (config のみ)
  - airport_terminal: 5 fleets, Door ×5, caddy (PatrolController), crowd sim 簡易版 (RandomWalkController)
  - hotel: 3 fleets, Door ×12, Elevator ×2, 3 floors
  - clinic: 2 fleets, Door ~10, Elevator ×2 (active), 2 floors

## Constraints

- ROS 非依存 — Device / Controller は pybullet_fleet コアに実装
- p.DIRECT テスト — GUI 不可の CI 環境で全テスト実行
- Agent factory (`from_params`) との整合性 — DoorDevice/ElevatorDevice のサブクラス生成
- 既存テスト (1255 件) が全パスすること
- coverage 75% 以上維持

## Out of Scope

- Batch API ROS Wrapper (item 8) — FleetStates.msg 未定義
- ROS 2 Action Server (item 10) — ExecuteAction.action 未ビルド
- Multi-floor visualization (item 9)
- body_id Optional (Phase 2 で全 Device が物理ボディを持つため不要)
- Door/Elevator URDF の詳細形状デザイン (simple_cube ベースで開始)
- Crowd simulator の nav graph ベースパス計画 (RandomWalkController で簡易代替)

## Open Questions

- [ ] Agent.from_params() のサブクラス生成方法 (cls.__new__ vs __class__ 代入 vs factory 修正)
- [ ] ElevatorDevice の platform_size パラメータ設計 (AABB vs 距離ベース)

## Success Criteria

- [ ] DoorDevice: request_open() → JointAction → door_state が "closed" → "opening" → "open" 遷移
- [ ] DoorDevice: request_close() → door_state が "open" → "closing" → "closed" 遷移
- [ ] ElevatorDevice: request_floor("L2") → Z 移動 → current_floor 更新
- [ ] ElevatorDevice: auto_attach=true でプラットフォーム上 Agent が追従
- [ ] WorkcellDevice: 静的位置マーカーとして機能
- [ ] PatrolController: ウェイポイント巡回 + loop
- [ ] RandomWalkController: ランダム近傍歩行
- [ ] Controller チェーン: 高レベル → 低レベル順の compute() 実行
- [ ] set_controller() の後方互換性
- [ ] entity_registry で YAML スポーン
- [ ] Demo: battle_royale — launch + config → patrol tasks → docker compose 起動
- [ ] Demo: hotel — launch + config → patrol + door + elevator → docker compose 起動
- [ ] Demo: clinic — launch + config → patrol + door + elevator → docker compose 起動
- [ ] Demo: airport_terminal — config 更新 → doors + caddy patrol → docker compose 起動
- [ ] 既存テスト全パス + coverage 75%+
- [ ] `make verify` パス

## References

- [ROADMAP.md](../../../ros2_bridge/pybullet_fleet_ros/ROADMAP.md) — Phase 2 items 6, 7
- [plugin-architecture/spec.md](../plugin-architecture/spec.md) — Controller/EventBus patterns
- [sim_object.py](../../../pybullet_fleet/sim_object.py) — SimObject base class
- [agent.py](../../../pybullet_fleet/agent.py) — Agent / JointAction / Controller
- [controller.py](../../../pybullet_fleet/controller.py) — Controller ABC / Registry
- [entity_registry.py](../../../pybullet_fleet/entity_registry.py) — YAML type dispatch
