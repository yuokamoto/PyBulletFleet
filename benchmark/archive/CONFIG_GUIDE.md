# Benchmark Configuration Guide

## Overview

`benchmark_config.yaml`を使用することで、パフォーマンスベンチマークの設定を一元管理できます。

## 使い方

### 基本的な使用

```bash
# デフォルト設定で実行
python benchmark/performance_benchmark.py --agents 1000 --duration 10

# 特定のシナリオを使用
python benchmark/performance_benchmark.py --agents 1000 --duration 10 --scenario no_collision

# カスタムconfigファイルを指定
python benchmark/performance_benchmark.py --agents 1000 --duration 10 --config my_config.yaml
```

### コマンドライン引数

```
--agents AGENTS           エージェント数（デフォルト: 1000）
--duration DURATION       シミュレーション時間（秒、デフォルト: 10.0）
--repetitions REPETITIONS 繰り返し回数（統計用、デフォルト: 3）
--gui                     GUI有効化（遅くなるが視覚化可能）
--config CONFIG           configファイルパス（デフォルト: benchmark/benchmark_config.yaml）
--scenario SCENARIO       シナリオ名（例: no_collision, collision_2d_10hz）
```

## Configuration File Structure

### Base Configuration

```yaml
simulation:
  timestep: 0.1  # シミュレーションタイムステップ（秒）
  speed: 0.0  # 0 = 最大速度（スリープなし）
  physics: false  # 物理演算の有効化（パフォーマンステストでは通常false）
  monitor: true  # モニタリングの有効化
  enable_monitor_gui: false  # モニタGUI（通常はfalse）
  enable_profiling: true  # プロファイリング出力
  log_level: "DEBUG"  # ログレベル
  collision_check_frequency: null  # 衝突判定頻度（Hz、null=毎ステップ）
  collision_check_2d: false  # 2D衝突判定（9近傍、falseだと3D 27近傍）
  ignore_structure_collision: true  # 構造物との衝突を無視
```

### Collision Check Frequency

衝突判定の頻度を制御する最も重要なパラメータ：

- **`null`**: 毎ステップ衝突判定（最も正確、最も遅い）
- **`10.0`**: 10Hz（秒間10回）で衝突判定
- **`1.0`**: 1Hz（秒間1回）で衝突判定
- **`0`**: 衝突判定を完全に無効化（最も速い）

### Collision Check Mode

衝突判定の次元を制御：

- **`collision_check_2d: true`**: 2D衝突判定（9近傍のみ）
  - XY平面のみチェック（Z軸方向の近傍を無視）
  - ~67%高速化
  - 用途: 地上走行ロボット（AGV）、固定Z位置のオブジェクト

- **`collision_check_2d: false`**: 3D衝突判定（27近傍）
  - XYZ全方向の近傍をチェック
  - 正確だが遅い
  - 用途: ドローン、リフト動作、3次元移動するオブジェクト

## Predefined Scenarios

### 1. `no_collision` - 衝突判定なし（最大速度）

```yaml
no_collision:
  simulation:
    collision_check_frequency: 0  # 衝突判定を完全に無効化
```

**用途**:
- シミュレーション基本性能の測定
- 衝突判定のオーバーヘッド測定の基準値

### 2. `collision_3d_full` - 3D衝突判定（毎ステップ）

```yaml
collision_3d_full:
  simulation:
    collision_check_frequency: null  # 毎ステップ
    collision_check_2d: false  # 3D (27近傍)
```

**用途**:
- 最も正確な衝突判定
- 3次元移動するロボットの評価
- 安全性が最優先のシナリオ

### 3. `collision_2d_10hz` - 2D衝突判定（10Hz）

```yaml
collision_2d_10hz:
  simulation:
    collision_check_frequency: 10.0  # 10Hz
    collision_check_2d: true  # 2D (9近傍)
```

**用途**:
- 地上走行ロボット（AGV）の最適化
- パフォーマンスと精度のバランス
- 実用的なプロダクション設定

## Performance Comparison

1000エージェント、10秒シミュレーションでの比較結果：

| シナリオ | RTF | ステップ時間 | 評価 |
|---------|-----|-------------|------|
| `no_collision` | 5.84x | 17.13ms | ✅ GOOD |
| `collision_2d_10hz` | 3.80x | 26.32ms | ⚠️ ACCEPTABLE |
| `collision_3d_full` | 2.51x | 39.83ms | ⚠️ ACCEPTABLE |

**RTF (Real-Time Factor)**: シミュレーション速度 / 実時間
- 5.84x = 実時間の5.84倍速でシミュレーション可能

**結論**:
- 衝突判定なし: **2.3倍高速** (vs 3D full)
- 2D 10Hz: **1.5倍高速** (vs 3D full)
- 2D最適化により、**53%のパフォーマンス向上** (39.83ms → 26.32ms)

## Custom Scenarios

独自のシナリオを追加：

```yaml
scenarios:
  # 超低頻度衝突判定
  low_freq_collision:
    simulation:
      collision_check_frequency: 1.0  # 1Hz
      collision_check_2d: true
    agents:
      num_agents: 2000
    benchmark:
      repetitions: 5

  # ハイブリッド: 2D高頻度
  hybrid_2d_high:
    simulation:
      collision_check_frequency: 30.0  # 30Hz
      collision_check_2d: true
    agents:
      num_agents: 500
```

## Tips for Performance Tuning

### 1. パフォーマンス優先

```yaml
simulation:
  collision_check_frequency: 0  # 衝突判定なし
  physics: false  # 物理演算なし
  enable_profiling: false  # プロファイリング無効
```

### 2. バランス型（推奨）

```yaml
simulation:
  collision_check_frequency: 10.0  # 10Hz
  collision_check_2d: true  # 2D最適化
  physics: false
```

### 3. 精度優先

```yaml
simulation:
  collision_check_frequency: null  # 毎ステップ
  collision_check_2d: false  # 3D正確
  physics: true  # 物理演算有効
```

## Profiling Output

`enable_profiling: true`の場合、各ステップの詳細なタイミング情報が出力されます：

```
[PROFILE] step 100: Agent.update=2.50ms, Callbacks=0.10ms, stepSimulation=1.20ms, Collisions=12.30ms, Monitor=0.05ms, Total=16.15ms
```

これにより、ボトルネックを特定できます：
- `Agent.update`: エージェント更新処理
- `Collisions`: 衝突判定処理
- `stepSimulation`: PyBullet物理ステップ

## Best Practices

1. **ベースライン測定**: まず`no_collision`で基本性能を測定
2. **比較テスト**: 複数シナリオで比較して最適設定を決定
3. **統計的信頼性**: `--repetitions 3`以上で平均値・標準偏差を取得
4. **プロセス分離**: 自動的に子プロセスで実行されるため、メモリリークの心配なし

## Troubleshooting

### パフォーマンスが低い場合

1. `collision_check_frequency: 0`でテストして基本性能を確認
2. 衝突判定のオーバーヘッドが大きい場合:
   - `collision_check_2d: true`に変更
   - `collision_check_frequency`を下げる（10Hz → 5Hz → 1Hz）
3. `enable_profiling: true`でボトルネックを特定

### メモリ使用量が大きい場合

1. `--repetitions 1`で単発テスト
2. エージェント数を減らす
3. `monitor: false`でモニタリング無効化

## See Also

- `README.md` - ベンチマークツール全体の説明
- `performance_benchmark.py` - メインベンチマークスクリプト
- `benchmark_config.yaml` - 設定ファイル
