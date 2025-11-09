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
- Automatic robot placement
- Multi-robot management
- Bulk goal setting


## Examples

### 100robots_demo.py

**Overview:**  
Demonstrates 100 mixed robots (arm + mobile) in grid layout with random motion.

**Features:**
- Load robots from URDF files
- Simulate 100 robots simultaneously
- Random motion (forward/rotation)
- Grid-based placement
- Performance monitoring

**Expected Behavior:**
- 100 robots arranged in 10x10 grid
- Mobile robots move randomly with forward/rotation
- Arm robots move joints randomly
- Data monitor shows FPS and step time

**Note:**  
This demo uses legacy API (`URDFObject`, `grid_spawn_*` functions). For new projects, use `Robot` and `RobotManager` instead.


## Todo
- Merge MeshObject and URDFObject
- Update examples to use Robot and RobotManager

## Dependencies

- Python 3.6+
- PyBullet
- NumPy
- PyYAML


## Documentation

- **README.md** (this file): User guide and API reference
- **DESIGN.md**: Architecture and design documentation