# Snapshot Replay — Design Spec

**Status:** Draft (not yet implemented)
**Date:** 2026-04-05
**Related:** USO integration skill (`integrating-with-uso`), DataMonitor

## Motivation

シミュレーション結果をリプレイ可能なデータとして保存し、後から再生・分析できるようにする。
USO (Unified Simulation Orchestrator) のスナップショット形式を参考にする。

## Design Decision: DataMonitor vs SnapshotLogger

### 当初の案

> DataMonitor に per-object 情報を追加し、リプレイデータの保存も担わせる

### 結論: 収集ポイントは統一、消費先は分離

DataMonitor はあくまで **表示用コンポーネント** であり、リプレイ用ログは別の `SnapshotLogger` に分離する。

**理由:**

| 懸念 | 説明 |
|------|------|
| 関心の分離 | DataMonitor は tkinter 表示。1ファイル上書き方式で履歴なし。リプレイには時系列ログが必要 |
| monitor=False 時 | GUI なしでもリプレイデータは保存したい。Monitor 依存だと壊れる |
| データ量 | 100台の per-object data (position, orientation, status) は集約 stats の数十倍。同じ JSON + tkinter に混ぜるとパフォーマンス問題 |

## Architecture

```
core_simulation.step()
    └── collect_state() → StateSnapshot (per-object + aggregate)
          ├── DataMonitor.write_data(snapshot.summary)  ← 集約表示 (既存)
          ├── SnapshotLogger.log(snapshot)              ← リプレイ用ログ (新規)
          └── SimulationRecorder._capture_frame()       ← ピクセル録画 (既存)
```

### データ収集ポイント

sim loop 内の `_update_monitor_data()` と同じ場所で per-object データも収集する。
`_moved_this_step` を使った delta snapshot 戦略は USO skill に定義済み。

### コンポーネント

| Component | Role | File |
|-----------|------|------|
| `StateSnapshot` | per-object + aggregate のデータ構造 | `snapshot.py` (新規) |
| `SnapshotLogger` | 時系列ログの保存 | `snapshot.py` (新規) |
| `ReplayController` | ログからの再生 | `replay.py` (新規) |
| `DataMonitor` | 集約表示 (既存、変更なし) | `data_monitor.py` |

## Snapshot Format (USO 準拠)

### Full Snapshot

```yaml
timestamp: 123.45
world:
  assets:
    robot_1:
      type: "robot"
      model: "robots/mobile_robot.urdf"
      position: [1.0, 2.0, 0.1]
      orientation: [0, 0, 0, 1]
      status: "moving"
      connected_to: "pallet_1"
```

### Delta Snapshot

```json
{
  "type": "delta_snapshot",
  "timestamp": 130.0,
  "updated_assets": {
    "robot_1": {"position": [5.0, 1.0, 0.0], "status": "moving"}
  },
  "removed_assets": [],
  "new_assets": {}
}
```

### Field Mapping (USO → PyBulletFleet)

| USO Field | PyBulletFleet Source |
|-----------|---------------------|
| `asset_id` | `SimObject.name` |
| `type` | Agent → "robot", SimObject → "object" |
| `model` | `Agent._urdf_path` or shape description |
| `position` | `SimObject.get_pose().position` → `[x, y, z]` |
| `orientation` | `SimObject.get_pose().orientation` → `[qx, qy, qz, qw]` |
| `status` | `ActionStatus` mapping (see below) |
| `connected_to` | `SimObject.get_attached_objects()` |

### Status Mapping

| PyBulletFleet State | USO Status |
|--------------------|------------|
| `agent.is_moving == True` | `"moving"` |
| `agent.is_moving == False`, no actions | `"idle"` |
| `ActionStatus.IN_PROGRESS` (Pick) | `"picking"` |
| `ActionStatus.IN_PROGRESS` (Drop) | `"dropping"` |
| `ActionStatus.FAILED` | `"error"` |
| SimObject (no agent) | `"static"` |

## Implementation Phases

### Phase 1: Snapshot Serialization (純 Python、外部依存なし)

- `SnapshotSerializer.to_full_snapshot(sim_core) → dict`
- `SnapshotSerializer.to_delta_snapshot(sim_core, moved_objects) → dict`
- `SnapshotDeserializer.from_full_snapshot(snapshot_dict, sim_core)` — spawn objects
- `SnapshotDeserializer.apply_delta_snapshot(delta_dict, sim_core)` — update poses
- テスト: round-trip (serialize → deserialize → compare)

### Phase 2: Replay Controller

- `ReplayController.load_log(log_path) → list[snapshot]`
- `ReplayController.play(speed=1.0)` — apply snapshots sequentially
- `ReplayController.seek(timestamp)` — jump to nearest full snapshot
- Physics OFF + kinematic control during replay
- ログフォーマット: Full snapshot 毎 N 秒 + Delta snapshot 毎 step

### Phase 3: USO Node Adapter (USO Master 必要)

- `PyBulletFleetNode(SimulationNode)` — Single mode (direct) / Distributed (ZeroMQ)
- See `integrating-with-uso` skill for details

## DataMonitor への将来的な拡張 (optional)

DataMonitor に per-object 表示を追加すること自体はリプレイとは独立した「inspector view」として有用。
例: 選択 agent の詳細、collision pair リスト。ただしリプレイ機能とは切り離して考える。

## Notes

- delta 検出には既存の `_moved_this_step` set を活用
- ログのシリアライズは MessagePack or CBOR でバイナリ効率化を検討
- `linear_velocity` / `angular_velocity` は現在未公開 → Phase 1 で `p.getBaseVelocity()` wrapper 追加を検討
