# Collision Detection Benchmarks

このディレクトリには、PyBulletFleetの衝突検出システムのベンチマークスクリプトが含まれています。

## ベンチマークファイル

### 1. collision_methods_config_based.py ⭐ **推奨**
**目的**: 実運用configファイルを使用した衝突検出メソッドの比較

**特徴**:
- 実際のconfigファイル（`config_physics_off.yaml`, `config_physics_on.yaml`等）を使用
- Physics OFF/ON/Hybridモードの比較
- 実運用に最も近い条件でのベンチマーク
- 複数ロボット＋移動＋接触シミュレーション

**使用方法**:
```bash
python collision_methods_config_based.py
```

**関連ドキュメント**:
- `COLLISION_METHODS_CONFIG_BASED_RESULTS.md` - ベンチマーク結果と分析

---

### 2. collision_detection_methods_benchmark.py 📊 **基礎リファレンス**
**目的**: PyBullet APIの基本的な衝突検出メソッドの比較

**特徴**:
- `getContactPoints()` vs `getClosestPoints()`の直接比較
- 静的オブジェクトでの基礎パフォーマンス測定
- PyBullet APIの基本動作理解に最適
- 最も単純な条件でのベンチマーク

**使用方法**:
```bash
python collision_detection_methods_benchmark.py
```

**関連ドキュメント**:
- `COLLISION_DETECTION_METHODS_ANALYSIS.md` - API比較分析

---

## ベンチマーク結果サマリー

### 推奨構成（100 objects, 500 steps）

| Configuration | Method | Avg Time | Collisions | 推奨用途 |
|--------------|--------|----------|------------|---------|
| **Physics OFF** | CLOSEST_POINTS | **1.244ms** ✅ | 8.7件 | **通常運用（推奨）** |
| Physics ON | CONTACT_POINTS | 1.408ms | 4.2件 | 物理検証・デバッグ |
| Hybrid | Both | 1.270ms | 8.7件 | 高度な用途 |

**結論**:
- **Physics OFF + CLOSEST_POINTS**が最速（約1.75倍高速）
- `collision_margin=0.02m`により事前検出が有効
- Kinematics主体のシミュレーションに最適

---

## 削除されたベンチマーク（Git履歴に残存）

以下のファイルは開発過程で作成されましたが、機能が重複するため削除されました：

### collision_methods_realistic.py
- PyBulletFleetクラスを直接使用した統合テスト
- `collision_methods_config_based.py`で代替可能

### collision_methods_with_movement.py
- 移動オブジェクトでの実験的ベンチマーク
- 最初の実装（509行）
- `collision_methods_config_based.py`が全機能を包含

必要に応じてGit履歴から復元可能：
```bash
git log --all --full-history -- benchmark/collision_methods_*.py
git show <commit-hash>:PyBulletFleet/benchmark/collision_methods_realistic.py
```

---

## 関連ドキュメント

- **設計ドキュメント**: `../docs/COLLISION_DETECTION_DESIGN.md`
- **Config使用ガイド**: `../config/README.md`
- **パフォーマンスガイド**: `PERFORMANCE_OPTIMIZATION_GUIDE.md`

---

## Quick Start

### 最も包括的なベンチマークを実行
```bash
# 推奨：Config比較ベンチマーク
python collision_methods_config_based.py

# 基礎API理解用
python collision_detection_methods_benchmark.py
```

### 独自ベンチマークの作成
```python
from pybullet_fleet import PyBulletFleet

# Config指定で簡単に実行
fleet = PyBulletFleet(config_path="../config/config_physics_off.yaml")
fleet.step()
collisions = fleet.check_collisions()
```

---

*Last Updated: 2026-01-19*
*PyBulletFleet Collision Detection System*
