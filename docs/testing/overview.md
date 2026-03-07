# Tests

This directory contains scripts for verifying and testing PyBulletFleet functionality.

## Test Files

### Automated Tests

#### `test_agent_movement.py` ✅ Recommended
**Purpose**: Automated unit tests for agent movement functionality

**Features**:
- Fast execution without GUI (`p.DIRECT` mode)
- Two test cases:
  1. Basic movement test: Single-goal movement of an Omnidirectional robot
  2. Path following test: 4-waypoint square path of a Differential drive robot
- Clear pass/fail determination (using XY-plane distance)
- Suitable for CI/CD and regression testing during development

**How to run**:
```bash
python tests/test_agent_movement.py
```

**Expected output**:
```
Basic Movement Test: ✓ PASS
Path Following Test: ✓ PASS
```

---

### Manual Tests / Demos

#### `test_robot_movement.py`
**Purpose**: Simple single-robot movement demo (GUI)

**Features**:
- Visual verification with GUI display
- A single Omnidirectional robot moves from (0,0,0.5) to (2,0,0.5)
- Simple and easy to understand
- Convenient for debugging and verifying behavior

**How to run**:
```bash
python tests/test_robot_movement.py
```

#### `test_path_following.py`
**Purpose**: Path following GUI demo (direct PyBullet connection version)

**Features**:
- Directly uses `p.connect(p.GUI)` without `MultiRobotSimulationCore`
- A single robot follows a square path
- Can be replaced by `examples/path_following_demo.py`
- Kept for debugging and comparison purposes

**How to run**:
```bash
python tests/test_path_following.py
```

#### `test_path_simple.py`
**Purpose**: Simple path following test (experimental)

**Features**:
- Minimal code implementing path following
- Simplified version for debugging
- Similar to `test_path_following.py`, but simpler

**How to run**:
```bash
python tests/test_path_simple.py
```

#### `test_path_with_simcore.py`
**Purpose**: Path following test using `MultiRobotSimulationCore`

**Features**:
- Usage example of `MultiRobotSimulationCore` and `sim.run_simulation()`
- Update logic using callbacks
- Experimental file that served as the base for `examples/path_following_demo.py`
- Rich debug output

**How to run**:
```bash
python tests/test_path_with_simcore.py
```

---

## Notes When Running Tests

### Package Imports
Test files are intended to be run from the top-level directory.
If running from the `tests/` directory, path adjustments may be necessary.

### Dependencies
- PyBullet
- NumPy
- pybullet_data

### Troubleshooting

**If the robot is not displayed**:
- When using `MultiRobotSimulationCore`, you need to call `sim.run_simulation()` or `sim.enable_rendering()`
- When connecting directly to PyBullet, it is displayed automatically

**If the z-coordinate is off**:
- Robots with `mass > 0` will fall due to gravity
- Since goal determination uses XY-plane distance, this is not a functional issue

---

## Recommended Usage

1. **Verification during development**: Run `test_agent_movement.py`
2. **Visual debugging**: Use `test_robot_movement.py` or `test_path_following.py`
3. **Full-featured demos**: Use `examples/path_following_demo.py`

---

## Reference

- Main demos are in the `examples/` directory
- `examples/path_following_demo.py`: Comparison demo of Omnidirectional vs Differential drive
- `examples/100robots_grid_demo.py`: Grid demo with 100 robots
