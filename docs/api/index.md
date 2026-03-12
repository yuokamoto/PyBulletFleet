# API Reference

API documentation is auto-generated from source code docstrings using Sphinx autodoc.

```{toctree}
:hidden:

generated/pybullet_fleet
```

## Full API Documentation

See [Full Module Reference](generated/pybullet_fleet) for all classes, methods, and parameters extracted from docstrings.

## Module Overview

| Module | Description |
|--------|-------------|
| `pybullet_fleet.core_simulation` | Main simulation engine (`CoreSimulation`) |
| `pybullet_fleet.agent` | Robot agent with kinematics and path following |
| `pybullet_fleet.agent_manager` | Agent lifecycle and goal management |
| `pybullet_fleet.action` | Action system for pick/drop/move operations |
| `pybullet_fleet.sim_object` | Static and dynamic simulation objects |
| `pybullet_fleet.config_utils` | YAML configuration loading |
| `pybullet_fleet.geometry` | Geometric utilities (poses, transforms) |
| `pybullet_fleet.collision_visualizer` | *(Deprecated)* Collision visualization overlays |
| `pybullet_fleet.data_monitor` | Runtime data monitoring |
| `pybullet_fleet.logging_utils` | Logging configuration (`LazyLogger`) |
| `pybullet_fleet.tools` | Utility functions |
| `pybullet_fleet.types` | Type definitions and enums |

:::{tip}
To regenerate API docs after code changes:
```bash
cd docs && sphinx-apidoc -o api/generated ../pybullet_fleet --force && make html
```
:::
