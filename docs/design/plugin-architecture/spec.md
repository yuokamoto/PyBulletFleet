# Plugin & Extensibility Architecture Specification

**Date:** 2026-03-21
**Status:** Draft

## Context

PyBulletFleetは100台以上のロボットをリアルタイムでシミュレーションする「キネマティクスファースト」フレームワーク。現在、外部ユーザーはライブラリとしてimportしてカスタムURDFやAction subclassを作成できるが、**名前ベースの登録・pip自動発見・ライフサイクルフック**が欠けている。外部パッケージとしてOSS配布可能な拡張システムが必要。

## Decision

**Gymnasium Registryパターンをベースに**、4カテゴリのentry_point登録 + Controller ABC (Strategy) + EventBusを追加する。

選定理由:
- 既存コードとの互換性が最も高い（後方互換100%）
- 100台フリートでのパフォーマンスオーバーヘッドが最小
- Pythonエコシステムの標準機構（`importlib.metadata.entry_points`）を活用
- 段階的実装が可能（5フェーズ）

却下した代替案:
- **Drake DiagramBuilder**: データフロー明示性は優れるが、100台で接続数爆発。全面リライト必要
- **ECS (Gazebo Ignition)**: 柔軟性最高だが、Agent/SimObjectのOOP構造を全廃する必要。v2.0候補
- **dm_control Task+Physics**: RL向け。マルチロボットフリートには1Task=1Worldだと表現力不足
- **Plugin ABC (Gazebo風)**: 設計が重い。「Actionだけ追加」でもPlugin class必要はオーバーエンジニアリング

## Requirements

### 拡張カテゴリ

| カテゴリ | entry_pointグループ | 登録されるもの | ユースケース |
|---------|---------------------|--------------|------------|
| Action | `pybullet_fleet.actions` | Action subclass | カスタムタスク（倉庫ピック、パトロール等） |
| Robot | `pybullet_fleet.robots` | AgentSpawnParamsファクトリ関数 | カスタムロボット定義（URDF + パラメータ） |
| Controller | `pybullet_fleet.controllers` | Controller subclass | 移動制御アルゴリズム（PID、Pure Pursuit等） |
| World | `pybullet_fleet.worlds` | Scene setup関数 | シーン定義（倉庫、工場フロア等）|

### Registry

- 3つの登録方法: `register()`, `@デコレータ`, `entry_points`
- 遅延ロード: entry_pointsは最初のget()まで読まない（起動高速化）
- ビルトインAction/Controllerもレジストリ経由で名前アクセス可能
- YAMLから名前で参照可能

### Controller ABC

- Strategy パターン: Agent は Controller を「持つ」(has-a)
- `compute(agent, dt) -> bool`: 1ステップの制御計算
- `on_goal_set()`, `on_stop()`, `reset()` コールバック
- Controller未設定 = 既存コードがそのまま動く（後方互換）
- 動的切り替え可能: `agent.set_controller()`
- ビルトイン: `TPIController`（既存ロジック切り出し）, `OmniOmniVelocityController`（cmd_vel用）

### EventBus

- `register_callback()` の進化版。完全後方互換
- イベントタイプ: pre_step, post_step, collision, agent_added/removed, action_started/completed, paused/resumed
- priority順実行。同一priorityはFIFO
- core_simulation.pyとaction.pyに発火ポイント追加

### Agent サブクラス

- 特別な仕組みは不要（通常のPython継承）
- ドキュメントでサポート範囲と注意点を記載
- マルチボディ制御等の高度なケース向け
- フックメソッド (`_on_pre_move`, `_on_post_move`, `_on_post_update`) を override 可能に

### 外部パッケージ配布

- `pip install pybullet-fleet-xxx` で自動登録
- pyproject.toml の entry_points で宣言
- テンプレートパッケージとドキュメントを提供
- pybullet-fleet >= 0.3.0 を依存に

## Constraints

- Python >= 3.10 (match pybullet_fleet requirement)
- パフォーマンス: EventBusのemit()は100台でステップ時間の1%未満
- Registry: グローバルステート → テスト間でclearが必要（autouse fixture）
- Controller._current_velocity等の内部状態アクセスは `_` prefix承知の上で許可（Controllerは準内部コンポーネント）
- 既存のAction, Agent, SimObject APIは変更しない（追加のみ）

## Out of Scope

- GUI/ダッシュボード for plugin管理
- プラグインのバージョン互換性チェック（semverに依存）
- サンドボックス/セキュリティ（信頼された拡張のみ）
- RL環境（Gymnasium Env wrapper）— 別機能として検討
- マーケットプレイス/パッケージインデックス

## Implementation Phases

| フェーズ | 内容 | 工数 | 前提 |
|---------|------|------|------|
| Phase 1 | Registry (4カテゴリ) + ビルトイン登録 + __init__.py公開API整理 | 小 | なし |
| Phase 2 | Controller ABC + TPIController + OmniOmniVelocityController | 中 | Phase 1 |
| Phase 3 | EventBus + core_simulation/action統合 | 中 | Phase 1と並行可 |
| Phase 4 | World Loader + YAML拡張 (ロボット/Controller名前解決) | 小 | Phase 1 |
| Phase 5 | テンプレートパッケージ + ドキュメント + examples | 小 | Phase 1-4 |

### ROS Bridgeとの関係

- ROS Bridge Phase 1 はpluginシステムなしでも動作可能
- Plugin Phase 2 完了後: ROS bridgeは `OmniOmniVelocityController` を使う設計に移行
- Plugin Phase 3 完了後: ROS bridgeは `agent_added`/`removed` イベントで自動handler管理

### EventBus導入時の移行メモ

現在 `bridge_node.py` では、Agent生成後に `_register_handler()` をpost-hookとして
手動ループで呼んでいる（`spawn_from_config` はループ内にフックを挟めないため）。

```python
# 現状 (post-hook パターン)
with self.sim.batch_spawn():
    for params in robots_params:
        agent = Agent.from_params(params, sim_core=self.sim)
        self._register_handler(agent)  # ROS層を付加
```

EventBus 導入後は `agent.spawned` イベントで自動化できる：

```python
# EventBus 導入後
sim.events.on("agent.spawned", lambda agent: bridge._register_handler(agent))
sim.events.on("agent.removed", lambda agent: bridge.remove_robot(agent.object_id))
agents = manager.spawn_from_config(robots_config)  # イベント自動発火
```

これにより `spawn_from_config` / `spawn_from_yaml` がbridge_nodeでも直接利用可能になり、
手動ループが不要になる。同様のパターンは USO snapshot 連携等にも適用可能。

### EventBus導入時のシミュレーション・ステッピング移行メモ

現在の ROS bridge (`bridge_node.py`) はシミュレーションのステッピングを
**ROS タイマー側が所有**する構成になっている：

1. `ROSSimulationCore.run_simulation()` → `initialize_simulation()` のみ実行し、即 return
2. ROS timer (`_step_callback`) が `sim.step_once()` を周期的に呼び出す
3. timer 内で `apply_cmd_vel()` → `step_once()` → `publish /clock` を順次実行

この設計は EventBus なしでも動作するワークアラウンドだが、以下の問題がある：

- **sim 内部ループの不在**: `run_simulation()` が即 return するため、
  sim 側の `register_callback()` で登録された周期コールバックが動作しない
- **step の所有者が曖昧**: sim core と ROS timer の両方が step を呼べる状態
- **スタンドアロンとの乖離**: ROS なしの `run_simulation()` とは全く異なるフロー

EventBus 導入後の理想構成:

```python
# step_once() 内部で自動 emit
def step_once(self):
    self.events.emit("pre_step", dt=self.dt)
    self._do_step()      # agents/actions update
    self.events.emit("post_step", dt=self.dt, sim_time=self.sim_time)

# ROS bridge は post_step にフックするだけ
sim.events.on("post_step", lambda **kw: bridge.publish_all_states(kw["sim_time"]))
sim.events.on("pre_step", lambda **kw: bridge.apply_all_cmd_vel())

# run_simulation() を普通に呼ぶ — ROS timer 不要
sim.run_simulation()
```

これにより:
- `run_simulation()` がステッピングの唯一の所有者に統一される
- ROS bridge は `run_simulation()` のイベントに subscribe するだけ
- スタンドアロン / ROS / USO すべて同じ `step_once()` パスを通る

## Open Questions

- [ ] Controller ABCで `agent._` prefixの内部状態にアクセスする設計は許容するか、public APIを追加すべきか
- [ ] World loaderでロボットも含めるか、ロボットは常に別定義にするか
- [ ] YAML config の `entities:` セクション追加時の既存config後方互換性
- [ ] entry_pointsの名前空間衝突（同名で別パッケージが登録した場合）の処理方針

## Success Criteria

- [ ] 外部パッケージからAction/Robot/Controller/Worldを `pip install` で追加できる
- [ ] YAMLで `type: "name"` で拡張を参照できる
- [ ] Controller差し替えでAgentの移動アルゴリズムが変わる
- [ ] EventBusでpre_step/post_step/collision/agent_added イベントを受信できる
- [ ] register_callback() が完全後方互換で動く
- [ ] 100台シミュレーションでpluginシステムのオーバーヘッド < 1%
- [ ] テンプレートパッケージが `pip install -e .` + entry_points で動作する
- [ ] 既存テストが全て通る（後方互換）

## References

- [Gymnasium Registry Pattern](https://gymnasium.farama.org/api/registry/)
- [Python entry_points](https://packaging.python.org/en/latest/specifications/entry-points/)
- [Drake DiagramBuilder](https://drake.mit.edu/doxygen_cxx/classdrake_1_1systems_1_1_diagram_builder.html) — v2.0候補
- [Gazebo Plugin Architecture](https://gazebosim.org/docs/latest/plugins/) — C++参考
- [robosuite Registry](https://robosuite.ai/) — 文字列ファクトリ参考
