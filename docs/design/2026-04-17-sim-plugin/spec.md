# SimPlugin ABC Specification

**Date:** 2026-04-17
**Status:** Draft

## Context

PyBulletFleetには `register_callback`、`EventBus`、`Controller` ABC といった拡張ポイントがあるが、
**YAMLだけでライフサイクル付きプラグインを登録できる仕組み**がない。
ROS Bridgeユーザーが `BridgeNode` をサブクラス化せずにシミュレーション動作をカスタマイズできるよう、
Gazeboの World Plugin に相当する `SimPlugin` ABC を追加する。

## Decision

**SimPlugin ABC (Approach A)** を採用。`on_init` / `on_step` を必須メソッド、`on_reset` / `on_shutdown` をオプションとする軽量ABCを新設し、YAML `plugins:` セクションから `importlib` で動的にロードする。

却下した代替案:
- **EventBus YAML (B)**: 既存 EventBus をYAML化。状態管理が不可、lifecycleフックなし
- **Callbacks YAML (C)**: 既存 `register_callback` をYAML化。init/shutdownフックなし、クラスベースの状態管理不可

## Requirements

- `SimPlugin` ABC: `on_init()`、`on_step(dt)` は abstract。`on_reset()`、`on_shutdown()` はデフォルト no-op
- コンストラクタで `sim_core` と `config` dict を受け取る
- YAML `plugins:` セクション: `class` (dotted path)、`frequency` (Hz, optional)、`config` (dict, optional)
- `importlib` による動的ロード（`entity_classes` と同パターン）
- ライフサイクル: `__init__`(spawn前) → `on_init`(spawn後) → `on_step`(毎ステップ) → `on_reset` / `on_shutdown`
- `frequency` 制御: Hz指定で呼び出し頻度を制限（`register_callback` と同パターン）
- Programmatic API: `sim.register_plugin(instance)` / `sim.register_plugin(PluginClass, config={}, frequency=10.0)`
- `sim.plugins` プロパティで読み取り専用のプラグイン一覧
- `on_step` 例外はログのみ（シミュレーション継続）、`__init__` / `on_init` 例外は即座に伝播

## Constraints

- `SimPlugin` は `SimObject` のサブクラスではない（別の継承ツリー）
- プラグイン内から `self.sim_core` で全APIにアクセス可能
- `entity_classes` と同じ `importlib` パターンを使用
- テストは全て `p.DIRECT` + `SimulationParams(gui=False, monitor=False)`
- `TYPE_CHECKING` guard 不要（SimPlugin → core_simulation の一方向依存のみ）

## Out of Scope

- プラグイン間の依存解決・順序制御
- プラグインのホットリロード
- EventBus イベント定義のYAML化
- GUI/ダッシュボードでのプラグイン管理
- プラグインのバージョン互換性チェック

## Usability — 外部ユーザー向け要件

SimPlugin は外部ユーザー（ROS を知らない Python ユーザー含む）にとって最も低い参入障壁を提供するべき。
以下を実装とともに整備する。

### 拡張ポイント選択ガイド

ドキュメントに「What do you want to do?」フローチャートを含める:

| やりたいこと | 推奨方法 |
|------------|---------|
| カスタムロボット型の追加 | `entity_classes` (YAML) |
| sim ロジック（衝突監視、動線分析、ログ等） | `SimPlugin` (YAML) |
| ROS topic / service の追加 | BridgeNode サブクラス or `register_callback` |
| sim ロジック + ROS 出力の両方 | SimPlugin → EventBus emit → BridgeNode subscribe → publish |

### 動くサンプル

`examples/plugins/` に最小限のプラグイン例を用意する:
- `fps_logger.py` — 最小の SimPlugin 実装（on_init + on_step のみ）
- 対応する YAML config
- README に使い方を記載

### エラーメッセージの充実

`importlib` ロード失敗時に外部ユーザーが迷わないよう、ヒント付きエラーメッセージを出す:
- `ModuleNotFoundError` → 「Did you `pip install` the package? Check the dotted path format: `package.module.ClassName`」
- `AttributeError` → 「Class `X` not found in module `Y`. Available names: ...」
- `TypeError` (非 SimPlugin) → 「`X` must be a SimPlugin subclass. Did you inherit from `SimPlugin`?」

### プラグインテストテンプレート

外部ユーザーが自作プラグインのテストを書けるよう、テストテンプレートを提供:
- `MockSimCore` の使い方
- `p.DIRECT` + `SimulationParams(gui=False, monitor=False)` パターン
- ドキュメントまたは `examples/plugins/test_fps_logger.py` として配置

## Open Questions

- [ ] プラグイン実行順序: YAML記述順 vs priority パラメータ。初期実装はYAML記述順

## Success Criteria

- [ ] `SimPlugin` ABC が定義され、`on_init` / `on_step` が abstract
- [ ] YAML `plugins:` セクションからプラグインがロード・実行される
- [ ] `frequency` パラメータでステップ頻度制御が動作する
- [ ] `on_reset` / `on_shutdown` が適切なタイミングで呼ばれる
- [ ] `on_step` の例外がシミュレーションをクラッシュさせない
- [ ] programmatic API (`register_plugin`) が動作する
- [ ] 既存テスト全て合格（後方互換）
- [ ] 新規テスト10件以上
- [ ] `examples/plugins/` に最小限のサンプルプラグインと YAML
- [ ] ドキュメントに拡張ポイント選択フローチャート
- [ ] importlib エラーメッセージにヒント付き

## References

- [既存 plugin-architecture 設計](../plugin-architecture/spec.md) — Registry + Controller + EventBus の全体設計
- [entity_registry.py](../../../pybullet_fleet/entity_registry.py) — `importlib` 動的ロードパターン
- [controller.py](../../../pybullet_fleet/controller.py) — `Controller` ABC パターン
- [events.py](../../../pybullet_fleet/events.py) — `EventBus` 実装
- [core_simulation.py](../../../pybullet_fleet/core_simulation.py) — 統合先
