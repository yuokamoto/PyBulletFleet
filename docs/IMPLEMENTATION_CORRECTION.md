# 重要な実装確認：SimObject vs Agent の更新メカニズム

**日付**: 2026-01-01
**発見者**: ユーザー指摘による重要な気づき

---

## 🚨 重要な発見

**ベンチマーク結果の解釈に重大な誤解がありました。**

### SimObjectの実装

```python
class SimObject:
    """静的オブジェクト用の基底クラス"""

    def __init__(self, body_id, ...):
        # 初期化のみ
        pass

    # ❌ update()メソッドは存在しない！
    # get_pose()とset_pose()は外部から明示的に呼ばれた時のみ実行
```

**重要**: SimObjectには`update()`メソッドが**ありません**。つまり：
- 毎フレーム自動的に呼ばれるコードはない
- `get_pose()`や`set_pose()`は外部から明示的に呼ばない限り実行されない
- 静的オブジェクト（棚、壁、ビンなど）は**放置すれば自動的にゼロコスト**

### Agentの実装

```python
class Agent(SimObject):
    """動的オブジェクト（ロボット）用のクラス"""

    def update(self, dt: float):
        """毎フレーム呼ばれる"""
        # アクション処理
        self._update_actions(dt)

        if not self.use_fixed_base:
            # ゴールに向かって移動
            if self._motion_mode == MotionMode.OMNIDIRECTIONAL:
                self._update_omnidirectional(dt)
            elif self._motion_mode == MotionMode.DIFFERENTIAL:
                self._update_differential(dt)

            # 内部でPyBullet APIを呼ぶ:
            # - getBasePositionAndOrientation()
            # - resetBaseVelocity()
            # - resetBasePositionAndOrientation()
```

**重要**: Agentには`update(dt)`メソッドが**あります**。つまり：
- 毎フレーム`update()`が呼ばれる
- 内部で自動的にPyBullet APIを呼ぶ
- ロボットの移動制御、軌道追従などを実行

---

## 📊 ベンチマークの問題点

### 現在のベンチマーク（非現実的）

```python
# test_overhead_analysis_v3.py の Test B: SimObject

# 更新フェーズ
for obj in objects:  # 10,000個のSimObject
    obj.get_pose()      # ← 実際のコードでは呼ばない！
    obj.set_pose(...)   # ← 実際のコードでは呼ばない！

# 結果: 177.72ms
# これは「もし全SimObjectを毎フレーム明示的に更新したら」の架空のコスト
```

### 実際の使用方法

```python
# 実際のシミュレーションコード

class Simulation:
    def __init__(self):
        # 静的オブジェクト（SimObject）
        self.shelves = [...]  # 9,000個の棚
        self.bins = [...]     # ビン

        # 動的オブジェクト（Agent）
        self.robots = [...]   # 1,000台のロボット

    def update(self, dt):
        # ❌ SimObjectは更新しない（update()メソッドがない）
        # for shelf in self.shelves:
        #     shelf.update()  # ← このメソッドは存在しない

        # ✅ Agentのみ更新
        for robot in self.robots:
            robot.update(dt)  # ← これが実際に呼ばれる

        # SimObjectの位置を読む必要がある場合のみ:
        # shelf_pose = self.shelves[0].get_pose()  # たまに
```

---

## 💡 正しいパフォーマンス分析

### オブジェクトタイプ別の実際のコスト

| タイプ | クラス | update()呼び出し | 実際のコスト/フレーム |
|--------|--------|-----------------|---------------------|
| **静的（棚）** | SimObject | ❌ なし | **0μs** |
| **静的（壁）** | SimObject | ❌ なし | **0μs** |
| **静的（ビン）** | SimObject | ❌ なし | **0μs** ※ |
| **動的（ロボット）** | Agent | ✅ あり | **25.3μs** |

※ビンがピックされる時だけ`get_pose()`/`set_pose()`が呼ばれる（まれ）

### 現実的なASRS倉庫シナリオ

```
構成（10,000オブジェクト）:
├─ 棚/ラック: 8,000個 (SimObject) → update不要
├─ ビン: 1,000個 (SimObject) → update不要
└─ ロボット: 1,000台 (Agent) → update必要

毎フレームの更新コスト:
├─ SimObject 9,000個: 0ms (何もしない)
└─ Agent 1,000台: 1,000 × 25.3μs = 25.3ms

合計: 25.3ms
```

**目標**: 10ms
**実測**: 25.3ms
**差**: 2.5倍遅い（以前の想定27倍から大幅改善！）

---

## 🎯 修正された最適化戦略

### 以前の誤った想定

```
❌ 全10,000オブジェクトを毎フレーム更新
   → 252.60ms（目標の25倍遅い）
   → 大規模なアーキテクチャ変更が必要
```

### 正しい理解

```
✅ 実際は1,000台のロボットのみ更新
   → 25.3ms（目標の2.5倍遅い）
   → 軽微な最適化で達成可能
```

### 新しい最適化アプローチ

#### ❌ 不要になった最適化
1. **静的/動的分離** - 既に実装されている（SimObject vs Agent）
2. **空間分割** - オーバーキル（1,000台なら不要）
3. **更新頻度調整** - オーバーキル

#### ✅ 必要な最適化（軽微）

##### Option 1: Agent.update()の最適化（推奨）

```python
class Agent:
    def update(self, dt):
        # 現在: 25.3μs/ロボット
        # 目標: 10μs/ロボット (2.5倍高速化)

        # 最適化候補:
        # 1. 静止中のロボットはupdate()をスキップ
        if not self._is_moving:
            return  # ← 早期リターン

        # 2. 毎フレームのPyBullet呼び出しを削減
        # 3. ベクトル演算の最適化
```

**期待効果**:
- 静止ロボットスキップ（50%が静止と仮定）: 25.3ms → **12.7ms**
- さらなる最適化: 12.7ms → **8-10ms** ✅ **目標達成**

##### Option 2: C++拡張（条件付き）

Agent.update()のコア部分のみC++化:
```cpp
// 軌道計算をC++で実装
void update_differential_motion(AgentState* state, float dt) {
    // ベクトル演算をSIMD命令で高速化
    // PyBullet APIは既にC++なので変更不要
}
```

**期待効果**: 25.3ms → **10-15ms**

---

## 📋 修正されたベンチマーク提案

### 現実的なベンチマーク

```python
def test_realistic_simulation(num_static: int, num_dynamic: int):
    """
    現実的なシミュレーションシナリオをベンチマーク

    Args:
        num_static: 静的オブジェクト数（SimObject）
        num_dynamic: 動的オブジェクト数（Agent）
    """
    # Setup
    shelves = [SimObject(...) for _ in range(num_static)]
    robots = [Agent(...) for _ in range(num_dynamic)]

    # Benchmark update loop (realistic)
    t0 = time.perf_counter()

    # SimObjectは更新しない（update()メソッドがない）
    # for shelf in shelves:
    #     shelf.update()  # ← 存在しない

    # Agentのみ更新
    for robot in robots:
        robot.update(dt=0.01)

    elapsed = time.perf_counter() - t0

    return {
        "static_count": num_static,
        "dynamic_count": num_dynamic,
        "update_time_ms": elapsed * 1000,
        "per_dynamic_us": (elapsed / num_dynamic) * 1e6,
    }

# 実行
result = test_realistic_simulation(
    num_static=9000,   # SimObject（棚、ビン）
    num_dynamic=1000   # Agent（ロボット）
)

# 期待結果:
# {
#     "static_count": 9000,
#     "dynamic_count": 1000,
#     "update_time_ms": 25.3,      # 1,000 × 25.3μs
#     "per_dynamic_us": 25.3,
# }
```

---

## 🔍 Agent.update()の詳細プロファイリング

次のステップとして、Agent.update()の内訳を調査：

```python
def profile_agent_update():
    """Agent.update()の詳細プロファイリング"""

    import cProfile
    import pstats

    # 1,000台のロボットでプロファイル
    robots = create_test_robots(1000)

    profiler = cProfile.Profile()
    profiler.enable()

    for robot in robots:
        robot.update(dt=0.01)

    profiler.disable()

    stats = pstats.Stats(profiler)
    stats.sort_stats('cumulative')
    stats.print_stats(20)  # Top 20 functions

    # 調査項目:
    # - _update_actions(): どれくらい時間がかかる？
    # - _update_differential(): ボトルネックは？
    # - PyBullet API呼び出し: 何回？どの関数？
    # - ベクトル演算: NumPy vs Python list
```

---

## 📊 比較表：誤解 vs 現実

| 項目 | 以前の誤解 | 現実 |
|------|-----------|------|
| **SimObjectの更新** | 毎フレーム177.72ms | 0ms（update()なし） |
| **総更新時間** | 252.60ms（全10,000個） | 25.3ms（1,000台のみ） |
| **目標との差** | 25倍遅い | 2.5倍遅い |
| **必要な最適化** | 大規模アーキテクチャ変更 | 軽微な最適化 |
| **実装難易度** | 高（3-4週間） | 低（1週間） |

---

## 🎬 結論

### 重要な気づき

1. **SimObjectは自動更新されない**
   - `update()`メソッドが存在しない
   - 静的オブジェクトは自然にゼロコスト
   - 既に「静的/動的分離」が実装されている状態

2. **実際のボトルネックはAgent.update()のみ**
   - 1,000台のロボット × 25.3μs = 25.3ms
   - 目標10msに対して2.5倍（以前の想定27倍から大幅に改善）

3. **最適化は軽微で済む**
   - 大規模なアーキテクチャ変更は不要
   - Agent.update()の最適化のみで目標達成可能

### 次のステップ

1. **Agent.update()の詳細プロファイリング** ← 優先
   - どの部分が遅いか特定
   - 静止ロボットのスキップ実装
   - 不要な計算の削除

2. **現実的なベンチマーク作成**
   - SimObject（静的）+ Agent（動的）の混合
   - 実際の使用パターンを反映

3. **軽微な最適化実装**
   - 静止判定の早期リターン
   - ベクトル演算の最適化
   - キャッシング戦略

4. **C++拡張（オプション）**
   - Agent.update()のみ対象
   - スコープが小さく実装しやすい

---

**重要**: この発見により、以前のドキュメント（PERFORMANCE_ANALYSIS.md、PERFORMANCE_SUMMARY_JA.md）の分析は**SimObjectを誤解した前提**に基づいています。実際の最適化戦略は**はるかに簡単**です。

**バージョン**: 1.0
**最終更新**: 2026-01-01
**修正理由**: ユーザー指摘によるSimObject実装の正しい理解
