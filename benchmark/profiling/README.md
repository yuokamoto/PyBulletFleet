# Profiling Tools

Located in `benchmark/profiling/` - these tools provide detailed CPU time and memory analysis of specific simulation components.

---

## Overview

プロファイリングツールは、シミュレーションの特定コンポーネントの詳細なパフォーマンス分析を行います。

**ツール一覧:**

| ツール | 目的 | 測定内容 |
|--------|------|---------|
| `simulation_profiler.py` | ステップ全体のコンポーネント分解 | Agent Update, Collision Check, PyBullet Step など |
| `collision_check.py` | 衝突検出の詳細分析 | Get AABBs, Spatial Hashing, AABB Filtering, Contact Points |
| `agent_update.py` | Agent.update() の詳細分析 | 5種類の分析手法（cProfile, Manual, PyBullet API, Stationary, Motion Modes） |
| `agent_manager_set_goal.py` | ゴール設定のプロファイリング | set_goal_pose() のオーバーヘッドと軌道計算 |

**使い分けガイド:**

1. **全体のボトルネック特定** → `simulation_profiler.py`
2. **衝突検出が遅い** → `collision_check.py`
3. **Agent Update が遅い** → `agent_update.py`
4. **ゴール設定が遅い** → `agent_manager_set_goal.py`

---

## simulation_profiler.py - シミュレーションステップのプロファイリング

**Purpose:** `step_once()` のコンポーネント別時間測定とボトルネック特定

シミュレーションの1ステップ全体を複数のコンポーネントに分解して測定します。

**performance_benchmark.py との違い:**
- `performance_benchmark.py`: 総合パフォーマンス測定（JSON出力、メモリ、CPU使用率）
- `simulation_profiler.py`: コンポーネント別の詳細分析（統計出力、プロファイリング）

### 測定コンポーネント

| コンポーネント | 説明 | 典型的な割合 |
|--------------|------|-------------|
| Agent Update | 全エージェントの状態更新（軌道追従、運動学） | 30-40% |
| Collision Check | 衝突検出（空間ハッシング、AABB） | 10-20% |
| PyBullet Step | 物理演算ステップ | 40-50% |
| Monitor Update | データモニター更新 | 1-5% |
| Other | その他のオーバーヘッド | <5% |

### 3種類の分析手法

| 手法 | コマンド | 目的 | 特徴 |
|------|---------|------|------|
| Built-in Profiling | `--test=builtin` (デフォルト) | コンポーネント時間配分 | ✅ step_once() から直接取得 |
| cProfile | `--test=cprofile` | 全関数のボトルネック | 関数レベルの詳細分析 |
| Motion Modes | `--test=motion_modes` | DIFFERENTIAL vs OMNIDIRECTIONAL | モード比較 |

### 使い方

```bash
# Built-in profiling（デフォルト）
python benchmark/profiling/simulation_profiler.py --agents=1000 --steps=100

# cProfile で詳細分析
python benchmark/profiling/simulation_profiler.py --agents=1000 --test=cprofile

# Motion Mode 比較
python benchmark/profiling/simulation_profiler.py --agents=1000 --test=motion_modes

# DIFFERENTIAL モードで測定
python benchmark/profiling/simulation_profiler.py --agents=1000 --mode=differential

# 全ての分析を実行
python benchmark/profiling/simulation_profiler.py --agents=1000 --test=all
```

### 出力例

```
Step Breakdown (OMNIDIRECTIONAL): 1000 agents (100 steps)
======================================================================

Agent Update:
  Mean:      12.45ms ( 35.2%)
  Median:    12.38ms
  StdDev:     0.87ms
  Range:  [ 11.23, 15.67]ms

Collision Check:
  Mean:       5.12ms ( 14.5%)
  Median:     5.08ms
  Range:  [  4.89,  6.23]ms

Pybullet Step:
  Mean:      15.34ms ( 43.4%)  ← 物理演算のコスト
  Median:    15.21ms
  Range:  [ 14.78, 17.12]ms

Total Step Time:
  Mean:      35.37ms (100.0%)
  Real-Time Factor: 28.3x
```

### 分析の使い分け

- **全体のボトルネック特定** → このツール
- Agent Update が遅い → `agent_update.py` で詳細分析
- Collision Check が遅い → `collision_check.py` で詳細分析

### 関連ファイル

- `pybullet_fleet/core_simulation.py`: `step_once()` の実装
- `agent_update.py`: Agent Update の詳細分析
- `collision_check.py`: Collision Check の詳細分析

---

## collision_check.py - 衝突検出の詳細分析

**Purpose:** 衝突検出処理のボトルネック特定と最適化検証

衝突検出は大規模シミュレーションの主要なボトルネックです。このツールは処理を4ステップに分解して測定します。

### 4ステップの分解

| ステップ | 説明 | 典型的な割合 |
|---------|------|-------------|
| Get AABBs | PyBullet からバウンディングボックス取得 | ~10% |
| Spatial Hashing | 空間グリッドの構築 | ~6% |
| AABB Filtering | 近接ペアの候補選定（27近傍探索） | **~75%** ← 最大のボトルネック |
| Contact Points | PyBullet で実際の衝突判定 | ~9% |

### 2種類の分析手法

| 手法 | コマンド | 目的 | 特徴 |
|------|---------|------|------|
| Built-in Profiling | `--test=builtin` (デフォルト) | 4ステップの時間配分を測定 | ✅ 実装から直接取得（正確） |
| cProfile | `--test=cprofile` | 関数レベルの詳細分析 | 内部関数の呼び出しを記録 |

### 使い方

```bash
# Built-in profiling（デフォルト、推奨）
python benchmark/profiling/collision_check.py --agents=1000 --iterations=100

# cProfile で詳細分析（関数レベル）
python benchmark/profiling/collision_check.py --agents=1000 --test=cprofile

# 両方実行
python benchmark/profiling/collision_check.py --agents=1000 --test=all
```

### 出力例

**Built-in Profiling:**
```
Collision Check Breakdown for 1000 Agents (Built-in Profiling)
======================================================================

Get Aabbs:
  Mean:      0.523ms ( 10.2%)
  Median:    0.512ms
  Range:  [ 0.489,  0.678]ms

Spatial Hashing:
  Mean:      0.312ms (  6.1%)
  
Aabb Filtering:
  Mean:      3.845ms ( 75.2%)  ← 最大のボトルネック
  
Contact Points:
  Mean:      0.432ms (  8.5%)

Total:
  Mean:      5.112ms (100.0%)

Advantages of Built-in Profiling:
  ✅ No implementation duplication (実装のコピー不要)
  ✅ Always measures the latest implementation (常に最新)
  ✅ Accurate dynamic cell size, 2D/3D mode measurement
  ✅ Minimal overhead (return_profiling flag only)
```

**cProfile（ステップ内の詳細）:**
```
ncalls  tottime  cumtime  関数
384500    1.850    1.850  AABB overlap check
 27000    0.650    0.650  dict.get (grid lookup)
 27000    0.420    0.420  tuple addition (neighbor_cell)
```

### 分析の使い分け

1. **ボトルネック特定** → `--test=builtin`
   - 4ステップのどこが遅いか一目瞭然
   
2. **ステップ内の詳細分析** → `--test=cprofile`
   - AABB Filtering の中で何が遅いか特定
   
3. **最適化効果の検証** → `--test=builtin`
   - オーバーヘッドなしで正確に before/after 比較

### 最適化のヒント

- AABB Filtering が 75% → 2D モードで 67%削減可能
- Collision ratio が 0.3% → フィルタリング精度向上の余地あり


### 関連ファイル

- `pybullet_fleet/core_simulation.py`: `check_collisions()`, `filter_aabb_pairs()` の実装
- `docs/PROFILING_GUIDE.md`: cProfile vs CPU Time の詳細説明

---

## agent_update.py - Agent.update() の詳細分析

**Purpose:** `Agent.update()` のボトルネック特定と最適化検証

`Agent.update()` は毎フレーム全エージェントに対して呼ばれる最も頻繁な処理です。このツールは5種類の分析手法でボトルネックを特定します。

### 5種類の分析手法

| 分析手法 | コマンド | 目的 | オーバーヘッド |
|---------|---------|------|---------------|
| cProfile | `--test=cprofile` | 全関数のボトルネック探し | 中（5-50%） |
| 手動タイミング | `--test=manual` | 特定メソッドの正確な測定 | 最小（<1%） |
| PyBullet API | `--test=pybullet` | C++ API のコスト測定 | 小（1-5%） |
| 静止 vs 移動 | `--test=stationary` | 移動・更新処理の有無による性能差 | なし |
| Motion Modes | `--test=motion_modes` | DIFFERENTIAL vs OMNIDIRECTIONAL | なし |

### 使い方

```bash
# 全ての分析を実行
python benchmark/profiling/agent_update.py --agents=1000 --updates=100

# cProfile のみ（ボトルネック探し）
python benchmark/profiling/agent_update.py --agents=1000 --test=cprofile

# 手動タイミングのみ（詳細測定）
python benchmark/profiling/agent_update.py --agents=1000 --updates=100 --test=manual

# PyBullet API 分析のみ
python benchmark/profiling/agent_update.py --agents=100 --updates=10 --test=pybullet

# 静止 vs 移動のみ
python benchmark/profiling/agent_update.py --agents=1000 --test=stationary

# Motion Mode 比較のみ
python benchmark/profiling/agent_update.py --agents=1000 --test=motion_modes
```

### 出力例

**Manual Timing:**
```
Component                 Mean (μs)    Median (μs)  Max (μs)     
----------------------------------------------------------------------
total                     120.50       118.30       250.10       
update_differential       80.20        78.50        180.00       
update_actions            35.10        34.00        90.00        
```

**PyBullet API:**
```
Function                                  Calls      Total (ms)   Avg (μs)    
---------------------------------------------------------------------------
resetBasePositionAndOrientation           1000       45.20        45.20       
getBasePositionAndOrientation             2000       30.50        15.25       
getBaseVelocity                           1000       12.30        12.30       
```

**Stationary vs Moving:**
```
Stationary agents (1000):
  Total time: 15.20 ms
  Per agent: 15.20 μs

Moving agents (1000):
  Total time: 120.50 ms
  Per agent: 120.50 μs

Overhead ratio (moving/stationary): 7.93x
Potential savings if 50% stationary: 67.85 ms
```

**Motion Modes:**
```
Motion Mode Comparison (1000 agents)
======================================================================

DIFFERENTIAL mode:
  Total time: 120.50 ms
  Per agent: 120.50 μs

OMNIDIRECTIONAL mode:
  Total time: 95.30 ms
  Per agent: 95.30 μs

Overhead ratio (DIFFERENTIAL/OMNIDIRECTIONAL): 1.26x
Performance difference: OMNIDIRECTIONAL is 26% faster
```

### 分析の使い分け

1. **ボトルネック特定** → `--test=cprofile`
   - 全関数を一覧して遅い部分を発見
   
2. **最適化効果の検証** → `--test=manual`
   - 最適化前後の正確な時間比較
   
3. **PyBullet API の最適化** → `--test=pybullet`
   - どの API が頻繁に呼ばれているか確認
   
4. **静止エージェントの性能影響を測定** → `--test=stationary`
   - ゴールなし（静止中）とゴールあり（移動中）の update() コスト比較
   - 移動・関節変化の更新処理の有無による性能差を確認

5. **Motion Mode の性能比較** → `--test=motion_modes`
   - DIFFERENTIAL vs OMNIDIRECTIONAL の update() コスト比較

### Manual Timing の仕組み

Monkey-patching を使って、各メソッドに `perf_counter()` を挿入します：

```python
# Monkey-patch Agent.update_differential()
original_update = Agent.update_differential

def patched_update(self, dt):
    t0 = time.perf_counter()
    result = original_update(self, dt)
    t1 = time.perf_counter()
    timings['update_differential'].append((t1 - t0) * 1000)  # ms
    return result

Agent.update_differential = patched_update
```

**特徴:**
- ✅ cProfile 不使用（オーバーヘッド最小）
- ✅ 特定メソッドのみ測定（ノイズなし）
- ✅ 最適化前後の正確な比較

### 関連ファイル

- `pybullet_fleet/agent.py`: `Agent.update()` の実装
- `docs/PROFILING_GUIDE.md`: Manual timing の詳細説明

---

## agent_manager_set_goal.py - ゴール設定のプロファイリング

**Purpose:** `AgentManager.set_goal_pose()` のボトルネック特定

複数エージェントにゴールを設定する操作を cProfile で分析します。

### 測定内容

- `agent_manager.set_goal_pose()` のオーバーヘッド（通常 <1%）
- `agent.set_goal_pose()` → `agent.set_path()` の呼び出し
- `_init_differential_rotation_trajectory()` の時間（**最大のボトルネック、通常 80-85%**）
- `_init_differential_forward_distance_trajectory()` の時間
- Rotation行列計算と軌道補間のコスト

### 使い方

```bash
# 基本実行（1000エージェント）
python benchmark/profiling/agent_manager_set_goal.py --agents=1000

# 少数エージェントでテスト
python benchmark/profiling/agent_manager_set_goal.py --agents=100
```

### 出力例

```
ncalls  tottime  cumtime  関数
  1000    0.001    0.109  agent_manager.set_goal_pose()
  1000    0.000    0.108  agent.set_goal_pose()
  1000    0.007    0.107  agent.set_path()
  1000    0.022    0.092  _init_differential_rotation_trajectory()  ← 84.4%
  1000    0.003    0.032  _init_differential_forward_distance_trajectory()
```

### 分析結果の読み方

```
agent_manager.set_goal_pose(): cumtime=0.109秒
  ├─ 自身のオーバーヘッド: tottime=0.001秒 (0.9%)
  └─ agent.set_goal_pose(): cumtime=0.108秒 (99.1%)
      ├─ 自身: tottime=0.000秒 (0%)
      └─ agent.set_path(): cumtime=0.107秒
          ├─ _init_differential_rotation_trajectory: 0.092秒 (84.4%) ← ボトルネック
          └─ _init_differential_forward_distance_trajectory: 0.032秒 (29.4%)
```

### 最適化のヒント

- `_init_differential_rotation_trajectory` が 84% を占める
- Rotation行列計算（`scipy.spatial.transform`）が重い
- 軌道補間（`two_point_interpolation`）も時間がかかる
- キャッシングや事前計算で改善可能

### 関連ファイル

- `pybullet_fleet/agent_manager.py`: `AgentManager.set_goal_pose()` の実装
- `pybullet_fleet/agent.py`: `Agent.set_path()` の実装

---

## プロファイリング手法の比較

### cProfile vs Manual Timing vs Built-in Profiling

| 手法 | オーバーヘッド | 精度 | 詳細度 | 使い分け |
|------|---------------|------|--------|---------|
| **cProfile** | 中（5-50%） | 中 | 高（全関数） | ボトルネック探し |
| **Manual Timing** | 最小（<1%） | 高 | 中（特定メソッド） | 最適化検証 |
| **Built-in Profiling** | 最小（<0.1%） | 高 | 中（特定ステップ） | 常時測定 |

### 詳細説明

#### cProfile
- **仕組み**: Python の標準プロファイラ、全関数呼び出しを記録
- **利点**: コード変更不要、包括的な分析
- **欠点**: オーバーヘッド大、ノイズが多い
- **用途**: 初期のボトルネック探し

**出力例:**
```
ncalls  tottime  cumtime  関数
  1000    0.022    0.092  _init_differential_rotation_trajectory()
384500    1.850    1.850  AABB overlap check
```

#### Manual Timing (Monkey-patching)
- **仕組み**: 特定メソッドに `perf_counter()` を挿入
- **利点**: オーバーヘッド最小、正確な測定
- **欠点**: コード変更が必要、特定メソッドのみ
- **用途**: 最適化前後の比較

**実装例:**
```python
original_method = Agent.update_differential

def patched_method(self, dt):
    t0 = time.perf_counter()
    result = original_method(self, dt)
    timings.append((time.perf_counter() - t0) * 1000)
    return result

Agent.update_differential = patched_method
```

#### Built-in Profiling
- **仕組み**: 実装に組み込まれた測定、`return_profiling=True` で有効化
- **利点**: オーバーヘッド最小、常に最新の実装を測定、二重管理不要
- **欠点**: 事前に実装が必要
- **用途**: 常時測定、本番環境でも使用可能

**実装例:**
```python
# core_simulation.py
def check_collisions(self, return_profiling: bool = False):
    timings = {}
    
    if return_profiling:
        t0 = time.perf_counter()
    # ... 処理 ...
    if return_profiling:
        timings['step1'] = (time.perf_counter() - t0) * 1000
    
    return result, timings
```

### 推奨フロー

1. **初期分析**: `simulation_profiler.py` で全体のボトルネック特定
2. **詳細分析**: cProfile でボトルネックの内部を分析
3. **最適化**: コード変更
4. **検証**: Manual Timing または Built-in Profiling で効果測定
5. **継続監視**: Built-in Profiling で本番環境でも測定

---

## パフォーマンス最適化ガイド

### ボトルネックの特定

**ステップ1: 全体像の把握**
```bash
python benchmark/profiling/simulation_profiler.py --agents=1000 --steps=100
```
→ Agent Update, Collision Check, PyBullet Step のどれが遅いか確認

**ステップ2: コンポーネント別詳細分析**

```bash
# Agent Update が遅い場合
python benchmark/profiling/agent_update.py --agents=1000 --test=cprofile

# Collision Check が遅い場合
python benchmark/profiling/collision_check.py --agents=1000 --test=builtin
```

**ステップ3: 最適化の実施と検証**

```bash
# Before
python benchmark/profiling/collision_check.py --agents=1000 > before.txt

# コード変更

# After
python benchmark/profiling/collision_check.py --agents=1000 > after.txt

# 比較
diff before.txt after.txt
```

### 典型的なボトルネック

#### 1. 衝突検出が遅い（Collision Check > 20%）

**対策:**
- `collision_check_2d=True` で2Dモードを有効化（67%削減）
- `collision_check_frequency=10.0` で頻度を下げる（10Hz）
- `ignore_static_collision=True` で構造物との衝突を無視

**検証:**
```bash
# Before: 3D, every step
python benchmark/profiling/collision_check.py --agents=1000

# After: 2D, 10Hz
# config.yaml で設定変更
python benchmark/profiling/collision_check.py --agents=1000
```

#### 2. Agent Update が遅い（Agent Update > 40%）

**対策:**
- 静止中のエージェントの更新処理をスキップ（移動していない場合は計算不要）
- PyBullet API 呼び出しを削減（キャッシング）

**検証:**
```bash
# 静止 vs 移動の差を確認
python benchmark/profiling/agent_update.py --agents=1000 --test=stationary

# PyBullet API のコストを確認
python benchmark/profiling/agent_update.py --agents=100 --test=pybullet
```

#### 3. ゴール設定が遅い（set_goal_pose > 100ms）

**対策:**
- 軌道計算のキャッシング
- 簡略化された軌道生成

**検証:**
```bash
python benchmark/profiling/agent_manager_set_goal.py --agents=1000
```

---

## トラブルシューティング

### プロファイリングログが表示されない

**原因:**
- `enable_profiling=False` になっている
- ログレベルが `DEBUG` になっていない

**解決策:**
```python
params = SimulationParams(
    enable_profiling=True,
    log_level="debug"
)
```

または
```yaml
# config.yaml
simulation:
  enable_profiling: true
  log_level: debug
```

### cProfile でセグフォルトが発生

**原因:**
- PyBullet の C++ 拡張との相性問題

**解決策:**
- Manual Timing または Built-in Profiling を使用
- `agent_update.py --test=manual` を試す
- `agent_update.py --test=stationary` を使用（cProfile 不使用）

### 測定結果にばらつきがある

**原因:**
- バックグラウンドプロセスの影響
- サーマルスロットリング

**解決策:**
- 複数回実行して統計を取る
- `--iterations` オプションで測定回数を増やす
- CPU使用率を確認

---

## See Also

- `../README.md` - ベンチマークスイート全体の概要
- `../PERFORMANCE_REPORT.md` - パフォーマンス分析レポート
- `../../docs/PROFILING_GUIDE.md` - プロファイリング手法の詳細説明
- `../../docs/PROFILING_CHANGES.md` - Built-in Profiling の実装詳細
- `../../pybullet_fleet/core_simulation.py` - プロファイリング機能の実装

---

**Last Updated:** 2026-01-11
**PyBullet Fleet Version:** Latest (Built-in Profiling support)
