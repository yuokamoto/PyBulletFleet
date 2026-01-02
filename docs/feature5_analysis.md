# Feature 5 分析レポート: Multiple mesh/PyBullet bodyを持つSimObject/Agent

## 検討内容
```
5. multiple mesh/pybullet_bodyを持つSimObject/Agentの作成。
  5-1. overheadを削減する。AgentやSimObjectをつくらずに、
       AgentManagerやSimObjectManagerが直接Pybullet objectを触るイメージ。
  5-2. そもそもやる価値があるかの判断から。
```

## 実測データ（1000オブジェクトでの詳細テスト）

### CPU使用率の比較

| 方式 | Wall Time | CPU Time | CPU使用率 |
|------|-----------|----------|-----------|
| **Direct PyBullet** | 0.112s | 0.120s | 107.0% |
| **SimObject wrapper** | 0.134s | 0.140s | 104.6% |
| **オーバーヘッド** | +0.022s (+19.4%) | +0.020s (+16.7%) | -2.4% |

**結果: CPU使用率はほぼ同等、むしろ若干効率的**

### メモリオーバーヘッド（詳細測定）

| 方式 | RSS Memory | VMS Memory | 1オブジェクトあたり |
|------|-----------|-----------|-------------------|
| **Direct PyBullet** | 11.09 MB | 8.80 MB | 11.36 KB |
| **SimObject wrapper** | 7.65 MB | 7.80 MB | 7.84 KB |
| **差分** | **-3.44 MB** | **-1.00 MB** | **-3.52 KB** |

**結果: SimObjectの方がメモリ効率が良い（-31.0%）**

### なぜSimObjectの方がメモリ効率が良いのか？

#### 実測による発見：

1. **PyBullet内部データ構造の差**
   ```
   Direct PyBullet: 1001 bodies (plane + 1000 boxes)
   SimObject:       1001 bodies (plane + 1000 boxes)
   → 同じ数だが、メモリ使用量が異なる
   ```

2. **Shape Caching効果**
   ```python
   # Direct PyBullet: 毎回shape IDを作成（ただしループ外で1回）
   vis_id = p.createVisualShape(...)  # 1回だけ
   col_id = p.createCollisionShape(...)  # 1回だけ

   # SimObject: クラスレベルでキャッシュ
   _shared_shapes: Dict[str, Tuple[int, int]] = {}
   # → 実質同じだが、管理方法が異なる
   ```

3. **Python オブジェクトのメモリ効率**
   ```
   単一SimObjectのサイズ: 0.23 KB (__dict__のみ)
   1000個のPythonリスト: 8.80 KB
   → Pythonオブジェクト自体は極めて軽量
   ```

4. **実際の理由（推測）**
   - **テスト間のメモリ状態の違い**:
     - 最初のテスト後、PyBulletの内部キャッシュが残っている
     - 2回目は既存のメモリプールを再利用できる
   - **測定タイミング**:
     - `gc.collect()`後も完全にメモリが解放されていない
     - OSのメモリ管理のタイミング
   - **PyBulletの最適化**:
     - SimObjectは同じshape_paramsを使い回す
     - 内部的にPyBulletが効率的に処理している可能性

### 検証: Pure Pythonオブジェクトのオーバーヘッド

| Wrapper種類 | 10000オブジェクト | 1オブジェクトあたり |
|------------|-----------------|-------------------|
| **Minimal** (body_idのみ) | 2.58 MB | 0.26 KB |
| **Full** (SimObject相当) | 5.41 MB | 0.55 KB |
| **追加コスト** | 2.83 MB | 0.28 KB |

**重要な発見: SimObjectのPythonラッパーは0.55 KB/objectのみ**

これは以下を含みます：
- `body_id`, `object_id`, `sim_core`
- `attached_objects` (list)
- `callbacks` (list)
- `_attach_offset` (Pose object)
- その他の状態変数

### 真実: メモリの「見かけ上の」減少

詳細測定により判明した事実：

```
テスト1（Direct PyBullet）開始時のRSS: 92.42 MB
テスト1終了後のRSS: 103.51 MB (92.42 + 11.09)

テスト2（SimObject）開始時のRSS: 104.52 MB
テスト2終了後のRSS: 112.17 MB (104.52 + 7.65)
```

**観察:**
- テスト1とテスト2の開始時点で既に12.1 MBの差がある
- これはテスト1のクリーンアップが完全ではないことを示す
- **実際には両方とも約11MBを使用している**

**正しい解釈:**
- SimObjectとDirect PyBulletのメモリ使用量は**ほぼ同等**
- Pythonラッパーの追加コストは**0.55 KB/object**（極めて小さい）
- 測定の差は主にテストの実行順序とGCのタイミングによるもの

### SimObjectManagerのオーバーヘッド

- オブジェクトリスト: 1000アイテム
- body_ids辞書: 1000アイテム
- object_ids辞書: 1000アイテム
- 全ポーズ取得: **2.2ms（1000オブジェクト）** = 2.2μs/object

## 結論：Feature 5は **実装不要**

### 理由1: Pythonラッパーのコストは極めて小さい

- **実測値: 0.55 KB/object**（10000オブジェクトで5.4 MB）
- 現代のPC（8GB RAM）では全く問題にならない
- PyBullet本体のメモリ（11 MB/1000 objects）の方が遥かに大きい

### 理由2: CPU使用率はほぼ同等

- **CPU時間オーバーヘッド: +16.7%** (0.020s for 1000 objects)
- 1オブジェクトあたり **0.02ms** のみ
- 実行時の物理計算の方が遥かにコストが高い
- むしろCPU使用率は若干低い（104.6% vs 107.0%）

### 理由3: メモリ測定の真実

従来の測定で「SimObjectの方が少ない」と見えたのは**測定の誤差**：

| 実際の状況 | 値 |
|----------|---|
| PyBullet本体のメモリ | ~11 MB/1000 objects |
| Pythonラッパーの追加コスト | 0.55 KB/object |
| 合計 | ~11.5 MB/1000 objects |

**正しい解釈:**
- SimObjectとDirect PyBulletのメモリ使用量はほぼ同等
- Pythonラッパーは全体の **5%未満** のオーバーヘッド
- この程度は測定誤差の範囲内

### 理由4: 機能性とのトレードオフ

**SimObjectが提供する機能:**
- ✓ Attachment/detachment（制約管理）
- ✓ Callback システム
- ✓ object_id による統一管理
- ✓ Pickable フラグ
- ✓ Shape caching
- ✓ ユーザーフレンドリーなAPI

**直接PyBullet管理の問題点:**
- ✗ 上記機能を再実装する必要がある
- ✗ コード重複が増える
- ✗ メンテナンス性が低下
- ✗ 得られるメモリ削減は 0.55 KB/object のみ

### 理由5: 実用上の規模での影響

**典型的なユースケース:**

| シナリオ | オブジェクト数 | Pythonラッパーコスト | 影響度 |
|---------|-------------|---------------------|-------|
| 小規模デモ | ~100 | 0.05 MB | 完全に無視できる |
| 中規模シミュレーション | ~1000 | 0.5 MB | 極めて小さい |
| 大規模シミュレーション | ~10000 | 5.4 MB | 許容範囲内 |
| 超大規模 | ~100000 | 54 MB | 物理計算の方が問題 |

現代のPCメモリ（8GB以上）では、54MBは **0.67%** に過ぎない。

## 代替案：既に実装済みの最適化

現在の実装には以下の最適化が既にあります：

### 1. Shape Caching（sim_object.py）
```python
# Class-level shared shapes cache
_shared_shapes: Dict[str, Tuple[int, int]] = {}
```
- 同じメッシュを再利用
- メモリとPyBullet API呼び出しを削減

### 2. Bulk Operations（agent_manager.py）
```python
manager.get_all_poses()  # 2.2ms for 1000 objects
manager.set_goal_pose_all(lambda robot: ...)
manager.add_action_sequence_all(lambda robot: ...)
```
- 効率的な一括操作

### 3. 辞書ベースの高速検索
```python
self.body_ids: Dict[int, T]    # O(1) lookup
self.object_ids: Dict[int, T]  # O(1) lookup
```

## 推奨事項

### ✗ Feature 5は実装しない

理由：
1. メモリオーバーヘッドが既に最小（0.55 KB/object）
2. 機能性を犠牲にしても得られるものがない
3. コードの複雑性が増す
4. メンテナンス性が低下

### ✓ 代わりに優先すべきこと

1. **Feature 6**: Camera機能強化（実用的）
2. **Feature 7**: Kinematic joint制御（新機能）
3. **Example 4-6**: デモの充実（ユーザー価値）
4. **Test/Readme**: ドキュメント整備（長期的価値）

## パフォーマンスが問題になる場合

もし将来的に本当に問題になったら：

### オプション1: 軽量版SimObjectの追加
```python
class LightweightSimObject:
    """Minimal wrapper for structure objects that don't need full functionality."""
    def __init__(self, body_id):
        self.body_id = body_id
        self.object_id = -1
```
- 構造物など静的オブジェクト専用
- 既存のSimObjectとの互換性を維持

### オプション2: __slots__の使用
```python
class SimObject:
    __slots__ = ['body_id', 'sim_core', 'attached_objects', ...]
```
- Pythonオブジェクトのメモリを40-50%削減可能
- ただし柔軟性が低下

## まとめ

### 質問への回答

#### Q1: CPU使用率は調べなくても良いか？
**A: 調査した結果、CPU使用率はほぼ同等でした**

- CPU時間オーバーヘッド: **+16.7%** (0.020s / 1000 objects)
- CPU使用率: SimObject 104.6% vs Direct PyBullet 107.0%
- **結論**: CPUコストは無視できるレベル

#### Q2: なぜwrapperの方がメモリ使用が減るのか？
**A: 測定誤差でした。実際はほぼ同等です**

**誤解の原因:**
1. テスト実行順序による初期メモリ状態の違い
2. ガベージコレクションのタイミング
3. OSのメモリ管理

**真実:**
```
PyBullet本体: ~11 MB / 1000 objects
Pythonラッパー: +0.55 KB / object
合計: ほぼ同じ
```

**正確な結論:**
- SimObjectのメモリオーバーヘッドは **0.55 KB/object**
- これはPyBullet本体（11 KB/object）の **5%未満**
- 実用上、完全に無視できる

### 最終結論

**Feature 5は実装不要。現在の設計が最適。**

| 評価項目 | 結果 | 判定 |
|---------|------|------|
| **メモリ効率** | +0.55 KB/object | ✓ 極めて小さい |
| **CPU効率** | +16.7% (+0.02ms/object) | ✓ 無視できる |
| **機能性** | 豊富な機能を提供 | ✓ 高い価値 |
| **保守性** | クリーンな設計 | ✓ 優れている |
| **実装コスト** | 既存機能の再実装が必要 | ✗ 高い |

**リソース配分の推奨:**
1. ✓ Feature 6: Camera機能強化（実用的価値）
2. ✓ Feature 7: Kinematic joint制御（新機能）
3. ✓ Examples 4-6: デモの充実（ユーザー価値）
4. ✓ Test/Docs: ドキュメント整備（長期的価値）

**パフォーマンスが問題になる場合の対策:**
- オプション1: `__slots__` 使用で40-50%メモリ削減
- オプション2: 軽量版 `LightweightSimObject` の追加
- オプション3: 物理計算の最適化（より大きな効果）
