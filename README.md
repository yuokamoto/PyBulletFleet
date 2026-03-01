# PyBulletFleet

General-purpose PyBullet simulation library for multi-robot fleets .

## Quick Start

```bash
# 1. Install package
cd PyBulletFleet
pip install -e .

# 2. Run demo
python examples/100robots_grid_demo.py
```

## Overview

This package provides reusable simulation components for multi-robot PyBullet environments.
## Features

- **MultiRobotSimulationCore**: Main simulation engine with configurable rendering and monitoring
- **Agent**: Goal-based navigation for mobile and static agents with action system support
- **AgentManager**: Multi-agent coordination and grid-based spawning
- **DataMonitor**: Real-time performance monitoring
- **Action System**: High-level actions (MoveTo, Pick, Drop, Wait) with state machine
- **Collision Detection**: Spatial hash-based efficient collision checking

## Directory Structure

```
PyBulletFleet/
├── pybullet_fleet/
│   ├── __init__.py
│   ├── core_simulation.py             # Main simulation engine
│   ├── agent.py                       # Agent implementation with action system
│   ├── agent_manager.py               # Multi-agent management
│   ├── action.py                      # Action system (MoveTo, Pick, Drop, Wait)
│   ├── sim_object.py                  # Base simulation object
│   ├── geometry.py                    # Pose, Path geometry classes
│   ├── collision_visualizer.py        # Collision visualization
│   ├── data_monitor.py                # Performance monitoring
│   └── tools.py                       # Utility functions
├── examples/
│   ├── 100robots_grid_demo.py         # Grid-based multi-agent demo
│   ├── 100robots_cube_patrol_demo.py  # Patrol behavior demo
│   ├── path_following_demo.py         # Path following with different motion modes
│   ├── action_system_demo.py          # Action system demonstration
│   ├── pick_drop_arm_demo.py          # Single arm pick & drop
│   ├── pick_drop_arm_100robots_demo.py # 100 arms pick & drop
│   ├── pick_drop_arm_action_demo.py   # Arm with action system
│   ├── pick_drop_mobile_100robots_demo.py # 100 mobile robots pick & drop
│   ├── mobile_manipulator_demo.py     # Mobile manipulator demo
│   ├── collision_features_demo.py     # Collision detection features
│   ├── memory_profiling_demo.py       # Memory profiling utilities
│   └── robot_demo.py                  # Basic robot demo
├── config/
│   ├── config.yaml                    # Default configuration
│   └── 100robots_config.yaml          # 100 robots configuration
├── robots/
│   ├── mobile_robot.urdf
│   └── arm_robot.urdf
├── mesh/
│   ├── cube.obj                       # Cube mesh for objects
│   └── 11pallet.obj                   # Pallet mesh
├── setup.py                           # Standard Python package setup
├── DESIGN.md                          # Architecture documentation
└── README.md                          # This file
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


### 2. Agent

Agent with goal-based navigation and action system.

**Key Features:**
- Automatic navigation to goal pose
- Velocity and acceleration limits
- Action system (MoveTo, Pick, Drop, Wait)
- Pose control and kinematic teleportation
- Object attachment/detachment

### 3. AgentManager

Manager for creating and coordinating multiple agents.

**Key Features:**
- Grid-based batch spawning
- Mixed agent type spawning with probabilities
- Automatic agent placement
- Multi-agent management
- Goal update callback system

**Spawning Methods (SimObjectManager base — works on both managers):**
- `spawn_objects_grid()`: Spawn single type in grid
- `spawn_grid_mixed()`: Spawn mixed types with probabilities
- `spawn_grid_counts()`: Spawn specific counts for each type
- `spawn_objects_batch()`: Batch spawn with explicit poses

**AgentManager convenience aliases:**
- `spawn_agents_grid()` → `spawn_objects_grid()`
- `spawn_agents_grid_mixed()` → `spawn_grid_mixed()`
- `spawn_agent_grid_counts()` → `spawn_grid_counts()`


## Examples

### Core Demos

#### 100robots_grid_demo.py
**Overview:**
Basic grid-based multi-agent demo using `AgentManager.spawn_agents_grid()`.

**Features:**
- YAML config file based
- Automatic grid placement
- Goal-based navigation
- Shared shape optimization

**Use Case:** Getting started with multi-agent simulations

```bash
python examples/100robots_grid_demo.py
```

#### 100robots_cube_patrol_demo.py
**Overview:**
100 agents patrolling between cubes with collision avoidance.

**Features:**
- Patrol behavior implementation
- Collision detection and avoidance
- Dynamic goal switching
- Spatial hash optimization

**Use Case:** Multi-agent coordination, patrol behaviors

```bash
python examples/100robots_cube_patrol_demo.py
```

#### path_following_demo.py
**Overview:**
Demo comparing two motion modes (omnidirectional and differential drive).

**Features:**
- Omnidirectional: Can move in any direction without rotation
- Differential Drive: Rotates toward target direction then moves forward
- Predefined path following (circular and square patterns)
- Realistic motion with velocity/acceleration limits
- Waypoint management using Path dataclass

**Use Case:** Motion planning, comparing different drive types

```bash
python examples/path_following_demo.py
```

### Action System Demos

#### action_system_demo.py
**Overview:**
Demonstration of the high-level action system.

**Features:**
- MoveTo action (goal-based navigation)
- Pick action (object attachment)
- Drop action (object detachment)
- Wait action (timed delays)
- State machine-based action execution

**Use Case:** Learning the action system API

```bash
python examples/action_system_demo.py
```

#### pick_drop_arm_demo.py
**Overview:**
Single arm robot performing pick and drop operations.

**Features:**
- URDF-based arm robot
- Pick and drop sequence
- Object attachment to specific links
- Pose-based manipulation

**Use Case:** Single manipulator control

```bash
python examples/pick_drop_arm_demo.py
```

#### pick_drop_arm_action_demo.py
**Overview:**
Arm robot using action system for pick and drop.

**Features:**
- Action-based pick and drop
- High-level task specification
- Automatic action sequencing

**Use Case:** Action system with manipulators

```bash
python examples/pick_drop_arm_action_demo.py
```

#### pick_drop_arm_100robots_demo.py
**Overview:**
100 arm robots performing synchronized pick and drop operations.

**Features:**
- Multi-manipulator coordination
- Grid-based arm placement
- Simultaneous pick and drop
- Performance optimization

**Use Case:** Large-scale manipulation tasks

```bash
python examples/pick_drop_arm_100robots_demo.py
```

#### pick_drop_mobile_100robots_demo.py
**Overview:**
100 mobile robots picking and dropping objects.

**Features:**
- Mobile base manipulation
- Navigation with object carrying
- Multi-agent pick and drop coordination

**Use Case:** Mobile manipulation fleets

```bash
python examples/pick_drop_mobile_100robots_demo.py
```

#### mobile_manipulator_demo.py
**Overview:**
Mobile manipulator demonstration.

**Features:**
- Combined mobile base and arm control
- Navigation while manipulating
- Coordinated motion

**Use Case:** Mobile manipulation systems

```bash
python examples/mobile_manipulator_demo.py
```

### Utility Demos

#### collision_features_demo.py
**Overview:**
Demonstration of collision detection features.

**Features:**
- Spatial hash visualization
- Collision pair detection
- Performance metrics
- Debug visualization

**Use Case:** Understanding collision system

```bash
python examples/collision_features_demo.py
```

#### memory_profiling_demo.py
**Overview:**
Memory profiling utilities demonstration.

**Features:**
- Memory usage tracking
- Performance profiling
- Resource monitoring

**Use Case:** Performance optimization

```bash
python examples/memory_profiling_demo.py
```

#### robot_demo.py
**Overview:**
Basic robot creation and control demo.

**Features:**
- Agent.from_mesh() examples
- Agent.from_urdf() examples
- Basic navigation
- Pose control

**Use Case:** Learning Agent class basics

```bash
python examples/robot_demo.py
```


## Dependencies

- Python 3.8+
- PyBullet
- NumPy
- PyYAML

## Development Setup

### Install development dependencies

```bash
pip install -e ".[dev]"
```

### Pre-commit hooks

Install pre-commit hooks for automatic code formatting and linting:

```bash
pip install pre-commit
pre-commit install
```

Run manually on all files:

```bash
pre-commit run --all-files
```

### Code quality tools

**Format code:**
```bash
black pybullet_fleet examples
```

**Lint code:**
```bash
flake8 pybullet_fleet
```

**Type check:**
```bash
pyright pybullet_fleet
```

### CI/CD

GitHub Actions automatically runs linting and type checking on every push and pull request. Tests are currently commented out and will be enabled once test suite is created.

## Documentation

- **README.md** (this file): User guide and API reference
- **DESIGN.md**: Architecture and design documentation
