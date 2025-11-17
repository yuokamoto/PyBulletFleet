# PyBulletFleet

General-purpose PyBullet simulation library for multi-robot fleets .

## Quick Start

```bash
# 1. Install package
cd PyBulletFleet
pip install -e .

# 2. Run demo
python examples/100robots_demo.py


```

## Overview

This package provides reusable simulation components for multi-robot PyBullet environments. 
## Features

- **MultiRobotSimulationCore**: Main simulation engine with configurable rendering, and monitoring
- **Robot**: Goal-based navigation for mobile and static robots
- **RobotManager**: Multi-robot coordination and grid-based spawning
- **DataMonitor**: Real-time performance monitoring

## Directory Structure

```
PyBulletFleet/
├── pybullet_fleet/
│   ├── __init__.py
│   ├── core_simulation.py             # Main simulation engine
│   ├── robot.py                       # Robot implementation (mobile & static)
│   ├── robot_manager.py               # Multi-robot management
│   ├── collision_visualizer.py        # Collision visualization
│   ├── data_monitor.py                # Performance monitoring
│   └── tools.py                       # Grid utilities
├── examples/
│   └── 100robots_demo.py              # 100 robots demo
├── config/
│   └── config.yaml                     # Default configuration
├── robots/
│   ├── mobile_robot.urdf
│   └── arm_robot.urdf
├── setup.py                            # Standard Python package setup
└── DESIGN.md                           # Architecture documentation
```

## Installation

Install in development mode:

```bash
cd PyBulletFleet
pip install -e .
```

This makes the package importable from anywhere while keeping it editable.

## Configuration

Edit `config/config.yaml` to customize simulation behavior.

### Keyboard Controls (GUI Mode)

When running with `gui: true`, the following keyboard shortcuts are available:

| Key | Function | Description |
|-----|----------|-------------|
| `SPACE` | Pause/Resume | Toggle simulation pause |
| `v` | Visual shapes | Toggle visual shapes ON/OFF |
| `c` | Collision shapes | Toggle collision wireframes ON/OFF |
| `t` | Transparency | Toggle structure transparency ON/OFF |

**⚠️ Transparency Performance Note:**
- The `t` key toggles transparency **one object at a time** in the registered structure bodies
- For scenes with **hundreds or thousands of objects**, the transparency toggle may be **slow**
- The initial transparency state can be set via `enable_structure_transparency: true/false` in config.yaml
- **Recommendation**: Set the desired transparency in the config file rather than toggling at runtime for large scenes

## Core Components

See [DESIGN.md](DESIGN.md) for detailed architecture documentation.

### 1. MultiRobotSimulationCore

Main simulation engine for PyBullet-based simulations.

**Key Features:**
- PyBullet engine initialization and management
- GUI/headless mode switching
- Simulation speed control
- Automatic/manual camera positioning
- Collision detection and visualization
- Performance monitoring
- Log level management


### 2. Robot

Robot.

**Key Features:**
- Automatic navigation to goal pose
- Velocity and acceleration limits
- Pose control
- Kinematic teleportation

### 3. RobotManager

Manager for creating and coordinating multiple robots.

**Key Features:**
- Grid-based batch spawning
- Probabilistic robot type selection
- Automatic robot placement
- Multi-robot management
- Bulk goal setting

**Spawning Methods:**
- `spawn_robots_grid()`: Spawn single robot type in grid
- `spawn_robots_grid_probabilistic()`: Spawn mixed robot types with probabilities


## Examples

### Recommended Demos (New API)

#### 100robots_grid_demo.py
**概要:**  
シンプルな単一タイプのグリッド生成デモ。`RobotManager.spawn_robots_grid()` を使用。

**特徴:**
- YAML設定ファイルベース
- 自動グリッド配置
- ゴールベースナビゲーション
- 共有シェイプ最適化

**用途:** 初心者向け、単一タイプのロボット群

```bash
python examples/100robots_grid_demo.py
```

#### 100robots_probabilistic_demo.py
**概要:**  
確率ベースの混合ロボット生成デモ。辞書設定で複数タイプを確率指定。

**特徴:**
- 辞書ベースのロボットタイプ設定
- 確率による重み付け選択（例: 50% mobile, 20% arm, 30% スキップ）
- 確率 < 100% 時のグリッド位置自動スキップ
- robot.user_data でのタイプ追跡
- 複数の設定例（100%カバー、スパース倉庫、可変密度）

**用途:** 高度な用途、混合ロボットタイプ、スパース配置

```bash
python examples/100robots_probabilistic_demo.py
```

**使用例:**
```python
# 確率でロボットタイプを定義
robot_type_params = {
    'mobile_robot': (mobile_spawn_params, 0.50),  # 50% の確率
    'arm_robot': (arm_spawn_params, 0.20)          # 20% の確率
    # 残り 30% は自動スキップ（ロボットを配置しない）
}

robots = robot_manager.spawn_robots_grid_probabilistic(
    num_positions=100,
    grid_params=grid_params,
    robot_type_params=robot_type_params
)
# 結果: ~50 mobile + ~20 arm + ~30 空き位置
```

#### robot_urdf_demo.py
**概要:**  
Robot クラスの基本チュートリアル。Mesh と URDF の両方をサポート。

**特徴:**
- Robot.from_mesh() の例
- Robot.from_urdf() の例
- SimObject 継承（attach/detach）
- URDF ロボットの関節制御

**用途:** Robot クラスの基本学習

```bash
python examples/robot_urdf_demo.py
```

### Legacy Demo

#### 100robots_demo.py
**概要:**  
旧 API（URDFObject ベース）のデモ。後方互換性のための参照用。

**注意:**  
このデモはレガシー API を使用しています。新規プロジェクトでは上記のデモを使用してください。

```bash
python examples/100robots_demo.py
```


## Todo
- ~~Merge MeshObject and URDFObject~~ ✅ Done (Robot class)
- ~~Update examples to use Robot and RobotManager~~ ✅ Done

## Dependencies

- Python 3.6+
- PyBullet
- NumPy
- PyYAML


## Documentation

- **README.md** (this file): User guide and API reference
- **DESIGN.md**: Architecture and design documentation