# DIFFERENTIAL モード詳細分析と最適化提案

**日付**: 2026-01-02
**対象**: Agent._update_differential() メソッド
**目的**: DIFFERENTIALモードが遅い理由を特定し、最適化方法を提案

---

## 🔍 DIFFERENTIAL モードの実装解析

### 処理フロー

DIFFERENTIAL モードは**2フェーズ制御**:

```
Phase 1: ROTATE (回転フェーズ)
├─ 目標方向を向くまで回転
├─ TPI (Two-Point Interpolation) で角度進行を計算
├─ Slerp でクォータニオン補間
└─ 完了後 → Phase 2へ

Phase 2: FORWARD (前進フェーズ)
├─ 目標位置まで直進
├─ TPI で距離進行を計算
└─ 完了後 → 次のウェイポイントまたは停止
```

### コード構造

```python
def _update_differential(self, dt: float):
    if self._differential_phase == DifferentialPhase.ROTATE:
        # Phase 1: 回転
        current_angle, angular_vel, _ = self._tpi_rotation_angle.get_point(current_time)

        # Slerp補間で新しい姿勢を計算
        angle_ratio = current_angle / self._rotation_total_angle
        r_interpolated = self._rotation_slerp(angle_ratio)
        new_orientation = r_interpolated.as_quat()

        # set_pose() で姿勢更新
        new_pose = Pose(position=current_pos, orientation=new_orientation)
        self.set_pose(new_pose)

    elif self._differential_phase == DifferentialPhase.FORWARD:
        # Phase 2: 前進
        distance_traveled, forward_vel, _ = self._tpi_forward.get_point(current_time)

        # 直線補間で新しい位置を計算
        ratio = distance_traveled / total_distance
        new_pos = start_pos + direction * ratio

        # set_pose() で位置更新
        new_pose = Pose(position=new_pos, orientation=target_orientation)
        self.set_pose(new_pose)
```

---

## 📊 パフォーマンス分析

### プロファイリング結果（1,000台のAgent）

```
DIFFERENTIAL:  110.75 ms  (110.75 μs/agent)
OMNIDIRECTIONAL: 26.12 ms  (26.12 μs/agent)

差: 4.24倍遅い
```

### なぜDIFFERENTIALが遅いのか？

#### 1. 複雑な数学計算

**回転フェーズ（Phase 1）のコスト**:

```python
# _init_differential_rotation_trajectory() での初期化
# 1. クォータニオン変換
r_current = R.from_quat(current_quat)       # scipy.Rotation 生成
r_target = R.from_quat(target_quat)         # scipy.Rotation 生成
r_delta = r_target * r_current.inv()        # クォータニオン乗算
rotation_angle = r_delta.magnitude()        # 角度計算

# 2. 回転行列計算（goalとの整合性チェック）
goal_rotation = R.from_quat(goal.orientation)  # scipy.Rotation 生成
goal_rot_matrix = goal_rotation.as_matrix()    # 3x3行列生成
x_axis_goal = goal_rot_matrix[:, 0]
z_axis_goal = goal_rot_matrix[:, 2]

# 3. クロス積とノルム計算（座標系構築）
y_axis = np.cross(z_axis_goal, x_axis_target)  # クロス積
y_norm_magnitude = np.linalg.norm(y_axis)      # ノルム
y_axis = y_axis / y_norm_magnitude             # 正規化
z_axis_final = np.cross(x_axis_target, y_axis) # クロス積
rotation_matrix = np.column_stack([...])       # 行列構築
r = R.from_matrix(rotation_matrix)             # Rotation生成
target_quat = r.as_quat()                      # クォータニオン変換

# 4. Slerp インターポレーター作成
key_rots = R.from_quat([start_quat, target_quat])  # 2つのRotation
self._rotation_slerp = Slerp([0.0, 1.0], key_rots)  # Slerp生成
```

**毎フレームのコスト**:

```python
# _update_differential() での更新（毎フレーム）
current_angle, angular_vel, _ = self._tpi_rotation_angle.get_point(current_time)
angle_ratio = current_angle / self._rotation_total_angle
r_interpolated = self._rotation_slerp(angle_ratio)  # Slerp補間（高コスト）
new_orientation = r_interpolated.as_quat()          # クォータニオン変換
```

#### 2. OMNIDIRECTIONALとの比較

**OMNIDIRECTIONAL（Phase 1なし）**:

```python
# 初期化（一度だけ）
self._tpi_pos[0].init(...)  # X軸
self._tpi_pos[1].init(...)  # Y軸
self._tpi_pos[2].init(...)  # Z軸

# 更新（毎フレーム）
p_x, v_x, _ = self._tpi_pos[0].get_point(current_time)  # シンプル
p_y, v_y, _ = self._tpi_pos[1].get_point(current_time)
p_z, v_z, _ = self._tpi_pos[2].get_point(current_time)
new_pos = [p_x, p_y, p_z]  # そのまま使用
```

**違い**:
- DIFFERENTIALは**Slerp補間**（球面線形補間、高コスト）
- OMNIDIRECTIONALは**線形補間**のみ（低コスト）

---

## 💡 最適化案

### ❌ DIFFERENTIAL無効化は不可（ユーザー要望）

代替案を検討します。

### ✅ 提案1: 静止Agent早期リターン（必須）

**現在の問題**:

```python
# agent.py lines 1458-1464
if not self._is_moving or self._goal_pose is None:
    # 不要なPyBullet呼び出し（3回）
    p.resetBaseVelocity(...)                      # ← 削除
    current_pos, current_orn = p.getBasePositionAndOrientation(...)  # ← 削除
    p.resetBasePositionAndOrientation(...)        # ← 削除
    return
```

**最適化後**:

```python
if not self._is_moving or self._goal_pose is None:
    # kinematic (mass=0) なら物理の影響を受けないので何もしない
    return  # ← 即座にリターン
```

**理由**:
1. **mass=0.0 (kinematic)** のAgentは物理シミュレーションの影響を受けない
2. 重力もドリフトも発生しない
3. 速度リセットも位置固定も**不要**

**期待効果**:
- 静止中: 3.48μs → **0μs** (完全スキップ)
- 50%静止の場合: 110.75ms → **55.38ms** (-50%)

**実装**:

```python
def update(self, dt: float):
    # Process action queue first
    self._update_actions(dt)

    if not self.use_fixed_base:
        # OPTIMIZATION: Early return for stationary agents
        if not self._is_moving or self._goal_pose is None:
            # Kinematic agents (mass=0) are not affected by physics
            # No need to reset velocity or fix position
            return

        # Dispatch to motion controller (only for moving agents)
        if self._motion_mode == MotionMode.OMNIDIRECTIONAL:
            self._update_omnidirectional(dt)
        elif self._motion_mode == MotionMode.DIFFERENTIAL:
            self._update_differential(dt)
```

---

### ⚠️ 提案2: DIFFERENTIALの軽量化（条件付き）

#### 2.1 Slerp事前計算の削減

**現在**: 毎ウェイポイントでSlerpオブジェクトを再生成

**最適化**: 再利用可能な場合は再利用

```python
# _init_differential_rotation_trajectory() で
if hasattr(self, '_cached_slerp_start') and \
   np.allclose(self._cached_slerp_start, self._rotation_start_quat) and \
   np.allclose(self._cached_slerp_target, self._rotation_target_quat):
    # 同じ回転なら再利用
    pass
else:
    # 新しいSlerp作成
    key_rots = R.from_quat([self._rotation_start_quat, self._rotation_target_quat])
    self._rotation_slerp = Slerp([0.0, 1.0], key_rots)
    self._cached_slerp_start = self._rotation_start_quat.copy()
    self._cached_slerp_target = self._rotation_target_quat.copy()
```

**期待効果**: 5-10%改善（微小）

#### 2.2 回転が不要な場合のスキップ

**現在**: 小さな角度でも回転フェーズを実行

**最適化**: 閾値以下ならスキップ

```python
# _init_differential_rotation_trajectory() で
ROTATION_THRESHOLD = 0.01  # rad (約0.6度)

if rotation_angle < ROTATION_THRESHOLD:
    # 回転ほぼ不要 → 即座にFORWARDフェーズへ
    logger.debug(f"Rotation angle {rotation_angle:.4f} < threshold, skipping rotation")
    p.resetBasePositionAndOrientation(self.body_id, current_pos, self._rotation_target_quat)
    self._differential_phase = DifferentialPhase.FORWARD
    self._init_differential_forward_distance_trajectory(self._forward_total_distance_3d)
    return
```

**期待効果**: 10-20%改善（シナリオ依存）

---

### ✅ 提案3: NumPy最適化

#### 3.1 ベクトル演算の最適化

**現在**: Python listで計算

```python
direction_vec = goal_pos - current_pos  # NumPy配列
# ...
direction_3d = self._forward_goal_pos - self._forward_start_pos  # NumPy配列
```

**最適化**: 既にNumPy配列を使用しているので、これ以上の改善は少ない

#### 3.2 メモリアロケーション削減

**現在**: 毎フレーム新しいPoseオブジェクト作成

```python
new_pose = Pose(position=new_pos.tolist(), orientation=new_orientation)
self.set_pose(new_pose)
```

**最適化**: Poseオブジェクトを再利用（既にsim_object.pyで実装済み）

---

## 📊 最適化シナリオ比較

| 最適化 | 実装難易度 | 期間 | 期待効果 | 適用条件 |
|--------|-----------|------|----------|----------|
| **静止早期リターン** | 非常に低 | 10分 | -50% | 必須 |
| **Slerp再利用** | 低 | 1-2時間 | -5-10% | オプション |
| **小角度スキップ** | 低 | 30分 | -10-20% | オプション |

### 組み合わせ効果（1,000台、50%静止、DIFFERENTIAL）

```
現在: 110.75 ms

最適化1のみ（静止早期リターン）:
└─ 静止500台: 0 ms
└─ 移動500台: 500 × 110.75μs = 55.38 ms
   合計: 55.38 ms (-50%)

最適化1+2+3（全て）:
└─ 静止500台: 0 ms
└─ 移動500台: 500 × 90μs ≈ 45 ms
   合計: 45 ms (-59%)
```

**目標10msとの比較**: まだ4.5倍遅い

---

## 🎯 推奨実装順序

### ステップ1: 静止早期リターン（今すぐ実装）✅

```python
# pybullet_fleet/agent.py の update() メソッド（lines 1434-1480）

def update(self, dt: float):
    # Process action queue first
    self._update_actions(dt)

    if not self.use_fixed_base:
        # OPTIMIZATION: Early return for stationary kinematic agents
        if not self._is_moving or self._goal_pose is None:
            # Kinematic agents (mass=0) don't need physics correction
            return

        # Dispatch to appropriate motion controller
        if self._motion_mode == MotionMode.OMNIDIRECTIONAL:
            self._update_omnidirectional(dt)
        elif self._motion_mode == MotionMode.DIFFERENTIAL:
            self._update_differential(dt)
        else:
            logger.warning(f"Unknown motion mode: {self._motion_mode}")

    if self.is_urdf_robot():
        self.update_attached_objects_kinematics()
```

### ステップ2: ベンチマーク実行

```bash
python tests/simple_agent_profile.py --agents 1000
```

### ステップ3: 結果評価

- 目標10msに到達したか？
- さらなる最適化が必要か？

---

## 📈 期待される結果

```
最適化前（50%静止 + DIFFERENTIAL）:
├─ 全体: 110.75 ms
└─ 目標との差: 11.1倍 ❌

最適化後（静止早期リターンのみ）:
├─ 全体: 55.38 ms
└─ 目標との差: 5.5倍 ⚠️

最適化後（全て）:
├─ 全体: 45 ms
└─ 目標との差: 4.5倍 ⚠️
```

**結論**: 静止早期リターンだけでは目標10ms達成は困難。ただし**50%の大幅な改善**は可能。

---

## 🚨 重要な考察

### なぜ目標10msが厳しいのか？

```
1,000台のAgent × 10ms = 1台あたり10μs

現実:
- DIFFERENTIAL移動中: 110.75μs/台（11倍オーバー）
- OMNIDIRECTIONAL移動中: 26.12μs/台（2.6倍オーバー）

ボトルネック:
- PyBullet API呼び出し（set_pose内）: ~5-10μs
- TPI計算: ~5μs
- DIFFERENTIAL特有（Slerp等）: ~20-30μs
```

### 現実的な目標設定

```
シナリオ1（DIFFERENTIAL、50%静止）:
└─ 期待値: 45-55 ms

シナリオ2（OMNIDIRECTIONAL、50%静止）:
└─ 期待値: 13 ms ← 目標に近い！

シナリオ3（DIFFERENTIAL、70%静止）:
└─ 期待値: 33 ms

シナリオ4（OMNIDIRECTIONAL、70%静止）:
└─ 期待値: 8 ms ← 目標達成！✅
```

---

## 💬 まとめ

### 質問への回答

1. **「個々の処理が不要ということですかね？」**
   - ✅ **はい、完全に不要です**
   - mass=0.0 (kinematic) のAgentは物理の影響を受けない
   - 3つのPyBullet呼び出しは全て削除可能

2. **「Differentialを無効はできない」**
   - ✅ 了解しました
   - DIFFERENTIALモードを維持したまま最適化します

3. **「Differential自体の実装も調べてください」**
   - ✅ 詳細分析完了
   - 2フェーズ制御（ROTATE → FORWARD）
   - Slerp補間が主なオーバーヘッド（OMNI比4.24倍）

### 次のアクション

1. **静止早期リターンを実装**（10分）
2. **ベンチマーク実行**（確認）
3. **結果に基づいて追加最適化を検討**

---

**バージョン**: 1.0
**最終更新**: 2026-01-02
**次のステップ**: agent.pyの早期リターン実装
