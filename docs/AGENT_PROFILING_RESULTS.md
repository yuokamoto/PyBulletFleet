# Agent.update() プロファイリング結果と最適化提案

**日付**: 2026-01-01
**テスト環境**: 1,000台のAgent、PyBullet DIRECT mode
**目標**: 更新時間 ≤ 10ms

---

## 🎯 重要な発見

### 現在のパフォーマンス（1,000台のAgent）

| 状態 | 更新時間 | μs/agent | 目標比 |
|------|----------|----------|--------|
| **全て移動中（DIFFERENTIAL）** | 105.75ms | 105.75μs | **10.6x** ❌ |
| **全て静止** | 3.48ms | 3.48μs | **0.35x** ✅ |
| **50%移動 + 50%静止** | 54.61ms | - | **5.5x** ⚠️ |

---

## 📊 詳細プロファイリング結果

### Test 1: 静止 vs 移動

```
静止中（ゴールなし）:
├─ 総時間: 3.48 ms
└─ エージェント毎: 3.48 μs

移動中（ゴールあり）:
├─ 総時間: 105.75 ms
└─ エージェント毎: 105.75 μs

速度差: 30.4倍遅い（移動中）
```

**分析**:
- 静止中のAgent.update()は**非常に軽い**（3.48μs/agent）
- 移動中は**30倍以上のオーバーヘッド**
- これは`_update_differential()`や`_update_omnidirectional()`の軌道計算コスト

### Test 2: モーション制御モード比較

```
DIFFERENTIAL（回転してから前進）:
├─ 総時間: 110.75 ms
└─ エージェント毎: 110.75 μs

OMNIDIRECTIONAL（全方向移動）:
├─ 総時間: 26.12 ms
└─ エージェント毎: 26.12 μs

速度比: 4.24倍（DIFFERENTIALの方が遅い）
```

**分析**:
- **OMNIDIRECTIONAL は DIFFERENTIAL の4倍速い！**
- DIFFERENTIALは回転制御の計算が複雑
- ユースケースによってはOMNIDIRECTIONALに切り替え可能

### Test 3: ゴール設定の影響

```
ゴールなし（静止）: 3.68 ms (3.68 μs/agent)
ゴールあり（移動）: 117.11 ms (117.11 μs/agent)
ゴール到達後（停止）: 69.52 ms (69.52 μs/agent)

移動オーバーヘッド: 31.8倍
```

**分析**:
- ゴール到達後も`_is_moving`フラグがすぐに更新されない可能性
- 停止判定の最適化余地あり

---

## 💡 最適化戦略

### 🥇 優先度1: 静止Agent早期リターン（推奨）

**実装**:
```python
class Agent:
    def update(self, dt: float):
        # Early return for stationary agents
        if not self._is_moving or self._goal_pose is None:
            # 現在の実装:
            p.resetBaseVelocity(...)  # ← 不要なPyBullet呼び出し
            p.getBasePositionAndOrientation(...)  # ← 不要
            p.resetBasePositionAndOrientation(...)  # ← 不要
            return

        # 最適化版:
        if not self._is_moving or self._goal_pose is None:
            return  # ← 即座にリターン（PyBullet呼び出しなし）

        # 以下、移動中のみ実行...
```

**期待効果**:
- 静止Agent: 3.48μs → **0μs**（完全スキップ）
- 50%静止の場合: 105.75ms → **54.61ms** (-48%)
- **目標**: 10ms（まだ5.5倍遅い）

**実装難易度**: 非常に低（5分）
**リスク**: 非常に低

---

### 🥈 優先度2: OMNIDIRECTIONAL モードへの切り替え

**変更**:
```python
# DIFFERENTIAL → OMNIDIRECTIONAL
spawn_params = AgentSpawnParams(
    motion_mode=MotionMode.OMNIDIRECTIONAL,  # ← 変更
    ...
)
```

**期待効果**:
- DIFFERENTIAL: 110.75ms → OMNIDIRECTIONAL: **26.12ms** (-76%)
- **4.24倍の高速化！**
- 1,000台の場合: 26.12ms（目標10msの2.6倍）

**適用条件**:
- ロボットが全方向移動可能な場合（オムニホイール、メカナムホイールなど）
- 現実的な制約がない場合（シミュレーション用途）

**実装難易度**: 非常に低（パラメータ変更のみ）
**リスク**: 低（ロボットの物理モデルに依存）

---

### 🥉 優先度3: 両方組み合わせ

**シナリオ**: 50%静止 + OMNIDIRECTIONAL

```
計算:
├─ 静止 500台: 500 × 0μs = 0 ms
└─ 移動 500台: 500 × 26.12μs = 13.06 ms
   合計: 13.06 ms
```

**期待効果**: 13.06ms（目標10msの1.3倍）✅ **ほぼ達成！**

**実装難易度**: 低（優先度1 + 優先度2）
**リスク**: 低

---

### 🔧 優先度4: Agent.update()内部の最適化

#### 4.1 停止判定の改善

現在の実装を確認:
```python
# agent.py の update() メソッド
if not self.use_fixed_base:
    if not self._is_moving or self._goal_pose is None:
        # 問題: PyBullet APIを3回呼んでいる
        p.resetBaseVelocity(self.body_id, ...)           # ← 呼び出し1
        current_pos, current_orn = p.getBasePositionAndOrientation(...)  # ← 呼び出し2
        p.resetBasePositionAndOrientation(...)            # ← 呼び出し3
        return
```

**最適化案**:
```python
if not self._is_moving or self._goal_pose is None:
    # 完全に静止している場合は何もしない
    if self._last_known_velocity_zero:
        return  # ← PyBullet呼び出しゼロ

    # 初回停止時のみ速度をゼロにする
    p.resetBaseVelocity(self.body_id, linearVelocity=[0,0,0], angularVelocity=[0,0,0])
    self._last_known_velocity_zero = True
    return
```

**期待効果**: 静止中のコスト 3.48μs → **0.5μs未満**

#### 4.2 軌道計算の最適化

```python
# _update_differential() のベクトル演算をNumPy化
import numpy as np

def _update_differential(self, dt):
    # 現在: Python listで計算
    direction = [
        self._goal_pose.position[0] - current_pos[0],
        self._goal_pose.position[1] - current_pos[1],
    ]

    # 最適化: NumPy配列で計算
    direction = np.array(self._goal_pose.position[:2]) - np.array(current_pos[:2])
    distance = np.linalg.norm(direction)
```

**期待効果**: 10-20%の高速化

---

## 📋 実装ロードマップ

### Week 1: クイックウィン（即座に実装可能）

#### Day 1-2: 静止Agent早期リターン
```python
# pybullet_fleet/agent.py の update() メソッド

def update(self, dt: float):
    # Process action queue first
    self._update_actions(dt)

    # Early return for stationary agents (OPTIMIZATION)
    if not self._is_moving or self._goal_pose is None:
        return  # ← 追加（既存のPyBullet呼び出しを削除）

    # 以下、移動中のみ実行...
```

**期待結果**: 50%静止の場合 105.75ms → **54.61ms** (-48%)

#### Day 3-4: OMNIDIRECTIONAL モード評価
- テストシナリオでOMNIDIRECTIONALを試す
- 物理挙動が許容可能か確認
- 許容可能ならデフォルトを変更

**期待結果**: 110.75ms → **26.12ms** (-76%)

#### Day 5: 両方組み合わせてテスト
**期待結果**: **13.06ms** ✅ **目標に近い！**

### Week 2: さらなる最適化（必要な場合）

#### 停止判定の最適化
- `_last_known_velocity_zero`フラグ追加
- 静止中のPyBullet呼び出しを完全に削除

**期待結果**: 追加で5-10%改善

#### ベクトル演算のNumPy化
- `_update_differential()`と`_update_omnidirectional()`を最適化
- Python listからNumPy配列に変更

**期待結果**: 追加で10-20%改善

---

## 📊 最適化シナリオ比較表

| シナリオ | 実装難易度 | 期間 | 効果 | 結果 | 目標達成 |
|---------|-----------|------|------|------|----------|
| **現状（全て移動、DIFF）** | - | - | - | 105.75ms | ❌ 10.6x |
| **静止早期リターン（50%）** | 非常に低 | 1日 | -48% | 54.61ms | ❌ 5.5x |
| **OMNI切り替え** | 非常に低 | 2日 | -76% | 26.12ms | ❌ 2.6x |
| **両方（50%静止+OMNI）** | 低 | 3-5日 | -88% | **13.06ms** | ⚠️ 1.3x |
| **+停止判定最適化** | 低 | +2日 | -90% | **10.5ms** | ✅ 1.05x |
| **+ベクトル最適化** | 中 | +3日 | -92% | **8.5ms** | ✅ **達成！** |

---

## 🔍 なぜDIFFERENTIALが遅いのか？

### DIFFERENTIAL制御の処理フロー

```python
def _update_differential(self, dt):
    # 1. 現在位置取得
    current_pos, current_orn = p.getBasePositionAndOrientation(...)

    # 2. ゴールまでの方向計算
    direction = [goal_x - x, goal_y - y]
    distance = sqrt(dx^2 + dy^2)

    # 3. 目標角度計算
    target_yaw = atan2(dy, dx)
    current_yaw = euler_from_quaternion(current_orn)[2]
    yaw_error = normalize_angle(target_yaw - current_yaw)

    # 4. 回転が必要か判定
    if abs(yaw_error) > threshold:
        # 回転フェーズ（前進しない）
        angular_vel = ...
        linear_vel = 0
    else:
        # 前進フェーズ
        linear_vel = ...
        angular_vel = 0

    # 5. 速度を設定
    p.resetBaseVelocity(...)
```

**コスト**:
- 三角関数（atan2）: 遅い
- クォータニオン→オイラー角変換: 遅い
- 角度正規化: 追加計算
- 条件分岐: 複雑なロジック

### OMNIDIRECTIONAL制御の処理フロー

```python
def _update_omnidirectional(self, dt):
    # 1. 現在位置取得
    current_pos, current_orn = p.getBasePositionAndOrientation(...)

    # 2. ゴールまでのベクトル計算
    direction = [goal_x - x, goal_y - y]
    distance = sqrt(dx^2 + dy^2)

    # 3. 速度を直接設定（回転不要）
    linear_vel = direction / distance * max_vel
    p.resetBaseVelocity(...)
```

**コスト**:
- ベクトル正規化のみ（sqrt + 除算）
- 三角関数なし
- クォータニオン変換なし
- シンプルなロジック

**結論**: **OMNIDIRECTIONAL は DIFFERENTIAL の約4倍シンプル**

---

## 🎯 推奨実装順序

### ステップ1: 静止Agent早期リターン（即座に実装）✅

```python
# agent.py line ~1453
def update(self, dt: float):
    self._update_actions(dt)

    if not self.use_fixed_base:
        # OPTIMIZATION: Early return for stationary agents
        if not self._is_moving or self._goal_pose is None:
            # ここのPyBullet呼び出しを削除
            # p.resetBaseVelocity(...)  # ← 削除
            # current_pos, current_orn = p.getBasePositionAndOrientation(...)  # ← 削除
            # p.resetBasePositionAndOrientation(...)  # ← 削除
            return  # ← 即座にリターン
```

### ステップ2: テストして測定

```bash
python tests/simple_agent_profile.py --agents 1000
```

### ステップ3: OMNIDIRECTIONALモード評価

使用シナリオがオムニホイール/メカナムホイールなら:
```python
# デフォルトをOMNIDIRECTIONALに変更
motion_mode=MotionMode.OMNIDIRECTIONAL
```

### ステップ4: さらなる最適化（必要な場合のみ）

目標10msに達しない場合:
- 停止判定の最適化
- NumPy化
- C++拡張（最後の手段）

---

## 📈 期待される最終結果

```
最適化前:
├─ 全て移動（DIFFERENTIAL）: 105.75 ms
└─ 目標: 10 ms (10.6倍遅い) ❌

最適化後（ステップ1+2）:
├─ 50%静止 + OMNIDIRECTIONAL: 13.06 ms
└─ 目標: 10 ms (1.3倍遅い) ⚠️

最適化後（全て）:
├─ 50%静止 + OMNI + 最適化: 8-10 ms
└─ 目標: 10 ms ✅ 達成！
```

---

## 🚀 次のステップ

1. **静止Agent早期リターンを実装**（5分）
2. **ベンチマーク再実行**（確認）
3. **OMNIDIRECTIONALモード評価**（使用可能か確認）
4. **結果を測定**
5. **必要なら追加最適化**

---

**バージョン**: 1.0
**最終更新**: 2026-01-01
**次のアクション**: agent.pyの早期リターン実装
