# Open-RMF Fleet Adapter Example — Design Spec

**Date:** 2026-04-20
**Status:** Draft
**Depends on:** [EventBus](../eventbus/spec.md) (PR 1), [ROS 2 Bridge Phase 1](../ros2-bridge/spec.md) (implemented)
**Parent spec:** [ros2-bridge/spec.md](../ros2-bridge/spec.md) Phase 2

## Context

ROS 2 Bridge Phase 1 は実装完了。Per-robot の `NavigateToPose` / `FollowPath` Action server、
`/clock`、`odom`、`tf` が揃っている。次のステップとして、Open-RMF との統合 example を作成し、
PyBulletFleet を Open-RMF のフリートシミュレータとして使えることを実証する。

`rmf_demos_fleet_adapter` のパターンを参考に、**最小限の fleet adapter script** を
example として提供する。フル機能の fleet adapter ではなく、動作デモと統合パターンの提示が目的。

## Decision

**Example script レベル**の Open-RMF 統合を実装する。

- `rmf_fleet_adapter` Python API を使った fleet adapter ノード
- rmf_demos の office world をベースにした 5台ロボットのデモ
- Docker に Open-RMF 依存を追加
- EventBus の `agent_added` / `agent_removed` で RobotHandler 自動管理を実証

フル機能の fleet adapter (双方向タスクアロケーション、charging、docking 等) は
将来のフェーズに残す。

## Requirements

### Fleet Adapter Node

- `rmf_fleet_adapter` Python API (`rmf_adapter`) を使用
- `RobotCommandHandle` を実装し、PyBulletFleet Agent のナビゲーションに接続
- `RobotUpdateHandle` で位置・ステータスを Open-RMF に報告
- `FleetAdapterHandle` でフリート全体を管理
- 各ロボットの位置を ROS `odom` topic から取得
- ナビゲーション指令を `NavigateToPose` action で送信
- `/clock` をシム時間として利用 (`use_sim_time:=true`)

### デモシナリオ (rmf_demos office)

- rmf_demos の office world SDF/NAV を使用（環境はビジュアル参考のみ、PyBulletFleet は独自環境）
- PyBulletFleet 5台の mobile robot を office のフロアプランに配置
- Open-RMF でタスク(patrol / loop) を dispatch → PyBulletFleet 側でロボットが移動
- RViz で Open-RMF の schedule visualizer と PyBulletFleet の TF/odom を同時表示

### EventBus 活用

- `agent_added` イベントで `RobotUpdateHandle` を自動登録
- `agent_removed` イベントで自動クリーンアップ
- `post_step` イベントでフリート状態を一括更新（publish_rate に依存しない内部更新）

### Docker 拡張

- `Dockerfile.jazzy` に Open-RMF パッケージ追加:
  - `ros-jazzy-rmf-fleet-adapter` (fleet adapter Python API)
  - `ros-jazzy-rmf-building-map-tools` (map support)
  - `ros-jazzy-rmf-demos` (demo worlds — optional, size 次第)
- または: 別の `Dockerfile.rmf` を用意して multi-stage でサイズ管理

### Launch & Config

- `rmf_demo.launch.py` — PyBulletFleet bridge + fleet adapter + (optional) RMF schedule visualizer
- `rmf_fleet.yaml` — fleet 設定:
  - fleet_name
  - robot_model (URDF path, footprint, max velocity)
  - map/level definitions (Open-RMF graph format or simplified)
  - タスク能力 (loop, delivery, etc.)

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      Docker Container                           │
│                                                                 │
│  ┌─────────────────────┐     ┌──────────────────────────────┐  │
│  │  pybullet_fleet_ros  │     │  rmf_fleet_adapter_pybullet  │  │
│  │  (BridgeNode)        │     │  (FleetAdapter example)      │  │
│  │                     │     │                              │  │
│  │  /{name}/odom ──────┼─────┼─► RobotUpdateHandle          │  │
│  │  /{name}/nav_to_pose◄─────┼── RobotCommandHandle         │  │
│  │  /clock ────────────┼─────┼─► use_sim_time               │  │
│  │  /tf ───────────────┼─────┼─► Schedule Visualizer        │  │
│  │                     │     │                              │  │
│  │  EventBus:          │     │  Open-RMF:                   │  │
│  │   agent_added ──────┼─────┼─► auto register robot        │  │
│  │   agent_removed ────┼─────┼─► auto unregister            │  │
│  └─────────────────────┘     └──────────────────────────────┘  │
│           │                              │                      │
│           ▼                              ▼                      │
│  ┌────────────────────────┐   ┌──────────────────────────┐     │
│  │ MultiRobotSimulationCore│   │ Open-RMF Traffic Schedule │     │
│  │ (in-process)            │   │ (rmf_traffic)            │     │
│  └────────────────────────┘   └──────────────────────────┘     │
└─────────────────────────────────────────────────────────────────┘
```

### Command Handle Flow

```
Open-RMF dispatches task (patrol point A → B)
  │
  ▼
RobotCommandHandle.navigate(waypoints, graph_index)
  │
  ├─ Convert RMF waypoints to Pose list
  ├─ Call NavigateToPose action on /{name}/navigate_to_pose
  │
  ▼
RobotHandler receives action goal → agent.set_goal_pose()
  │
  ▼
Agent navigates (TPI / Controller)
  │
  ▼
RobotUpdateHandle.update_position() ← RobotHandler.publish_state() ← odom
  │
  ▼
Action completes → RobotCommandHandle._on_nav_complete() → report to RMF
```

## Constraints

- `rmf_fleet_adapter` の ROS 2 Jazzy バイナリが apt で利用可能であること
- Open-RMF の full stack (traffic scheduler, door/lift adapters) は不要 — fleet adapter のみ
- PyBulletFleet のコアライブラリ (`pybullet_fleet/`) には変更なし
- Example は `ros2_bridge/` 以下に配置（PyPI パッケージには含まない）
- rmf_demos の map/building ファイルは Docker 内で参照、リポジトリにはコピーしない

## Out of Scope

- 充電ステーション/ドッキング対応
- Delivery タスク (pick → place)
- Door / Lift adapter
- カスタム RMF task type
- rmf_demos world の SDF を PyBulletFleet 内でレンダリング
- rmf_visualization_rviz_plugins のカスタマイズ
- `rmf_fleet_msgs/FleetState` の直接パブリッシュ（fleet adapter API が内部で処理）

## Implementation Approach

1. `rmf_fleet_adapter` の Python API を調査 (`rmf_adapter` module)
2. `RobotCommandHandle` 実装（navigate → `NavigateToPose` action call）
3. `RobotUpdateHandle` でロボット位置を `odom` から取得・報告
4. `FleetAdapterHandle` でフリート管理
5. `rmf_demo.launch.py` で bridge + fleet adapter を一括起動
6. `rmf_fleet.yaml` でフリート構成を定義
7. Docker に Open-RMF 依存追加
8. `docker/README.md` に Open-RMF demo セクション追加
9. 動作確認: タスク dispatch → ロボット移動 → 完了報告

## File Structure

```
ros2_bridge/pybullet_fleet_ros/
├── pybullet_fleet_ros/
│   ├── fleet_adapter.py             # NEW: Open-RMF fleet adapter node
│   ├── rmf_command_handle.py        # NEW: RobotCommandHandle impl
│   └── ... (existing files unchanged)
├── launch/
│   └── rmf_demo.launch.py           # NEW: bridge + fleet adapter
├── config/
│   └── rmf_fleet.yaml               # NEW: fleet adapter config
└── ... (existing files unchanged)

docker/
├── Dockerfile.jazzy                  # MODIFY: add rmf_fleet_adapter dep
└── README.md                        # MODIFY: add Open-RMF section
```

## Success Criteria

- [ ] `docker compose up` で PyBulletFleet + ROS bridge + Open-RMF fleet adapter が起動
- [ ] Open-RMF にロボット5台が登録される（`ros2 topic echo /fleet_states`）
- [ ] `ros2 run rmf_demos_tasks dispatch_patrol` でパトロールタスクが dispatch される
- [ ] PyBulletFleet のロボットがタスクに応じて移動する
- [ ] タスク完了が Open-RMF に報告される
- [ ] EventBus `agent_added` で新規ロボットが自動的に fleet adapter に登録される
- [ ] `docker/README.md` に Open-RMF demo の手順が記載されている
- [ ] 既存のテスト・demo に影響なし

## References

- [rmf_demos_fleet_adapter](https://github.com/open-rmf/rmf_demos/tree/main/rmf_demos_fleet_adapter) — Reference fleet adapter implementation
- [rmf_fleet_adapter Python API](https://github.com/open-rmf/rmf_ros2/tree/main/rmf_fleet_adapter_python) — Python bindings
- [Open-RMF](https://github.com/open-rmf/rmf) — Fleet management framework
- [ros2-bridge/spec.md](../ros2-bridge/spec.md) — Phase 2 definition
- [plugin-architecture/spec.md](../plugin-architecture/spec.md) — EventBus design
