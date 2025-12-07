# Tests

このディレクトリには、PyBulletFleetの動作確認とテスト用のスクリプトが含まれています。

## テストファイル

### 自動テスト

#### `test_agent_movement.py` ✅ 推奨
**目的**: エージェントの移動機能の自動ユニットテスト

**特徴**:
- GUIなし（`p.DIRECT`モード）で高速実行
- 2つのテストケース:
  1. 基本移動テスト: Omnidirectionalロボットの単一ゴール移動
  2. パスフォロイングテスト: Differential driveロボットの4ウェイポイント正方形パス
- 明確な成功/失敗判定（XY平面距離を使用）
- CI/CDや開発時の回帰テストに適している

**実行方法**:
```bash
python tests/test_agent_movement.py
```

**期待される結果**:
```
Basic Movement Test: ✓ PASS
Path Following Test: ✓ PASS
```

---

### 手動テスト・デモ

#### `test_robot_movement.py`
**目的**: シンプルな1ロボットの移動デモ（GUI）

**特徴**:
- GUI表示で視覚的に動作確認
- 単一のOmnidirectionalロボットが(0,0,0.5)から(2,0,0.5)へ移動
- シンプルで理解しやすい
- デバッグや動作確認に便利

**実行方法**:
```bash
python tests/test_robot_movement.py
```

#### `test_path_following.py`
**目的**: パスフォロイングのGUIデモ（直接PyBullet接続版）

**特徴**:
- `MultiRobotSimulationCore`を使わずに直接`p.connect(p.GUI)`
- 1台のロボットが正方形パスを辿る
- `examples/path_following_demo.py`で代替可能
- デバッグや比較用に保存

**実行方法**:
```bash
python tests/test_path_following.py
```

#### `test_path_simple.py`
**目的**: シンプルなパスフォロイングテスト（実験用）

**特徴**:
- 最小限のコードでパスフォロイングを実装
- デバッグ用の簡易版
- `test_path_following.py`と同様だが、よりシンプル

**実行方法**:
```bash
python tests/test_path_simple.py
```

#### `test_path_with_simcore.py`
**目的**: `MultiRobotSimulationCore`を使ったパスフォロイングテスト

**特徴**:
- `MultiRobotSimulationCore`と`sim.run_simulation()`の使用例
- コールバックを使った更新ロジック
- `examples/path_following_demo.py`のベースとなった実験ファイル
- デバッグ出力が豊富

**実行方法**:
```bash
python tests/test_path_with_simcore.py
```

---

## テスト実行時の注意

### パッケージのインポート
テストファイルはトップレベルから実行されることを想定しています。
`tests/`ディレクトリから実行する場合は、パスの調整が必要な場合があります。

### 依存関係
- PyBullet
- NumPy
- pybullet_data

### トラブルシューティング

**ロボットが表示されない場合**:
- `MultiRobotSimulationCore`を使う場合、`sim.run_simulation()`または`sim.enable_rendering()`を呼ぶ必要があります
- 直接PyBullet接続の場合は自動的に表示されます

**z座標がずれる場合**:
- 重力の影響で`mass > 0`のロボットは落下します
- XY平面距離でゴール判定を行っているため、機能的には問題ありません

---

## 推奨される使い方

1. **開発時の動作確認**: `test_agent_movement.py`を実行
2. **視覚的なデバッグ**: `test_robot_movement.py`または`test_path_following.py`
3. **本格的なデモ**: `examples/path_following_demo.py`を使用

---

## 参考

- メインのデモは`examples/`ディレクトリにあります
- `examples/path_following_demo.py`: Omnidirectional vs Differential driveの比較デモ
- `examples/100robots_grid_demo.py`: 100台のロボットのグリッドデモ
