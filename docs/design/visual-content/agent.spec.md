# Visual Content — Agent Technical Specification

## Objective

Add animated GIFs and model-catalog PNGs to PyBulletFleet documentation (Phase 1).

## Phase 1 Deliverables

### 1. Demo GIFs (3 files)

| File | Robots | Camera |
|------|--------|--------|
| `docs/media/100robots_grid_mixed.gif` | ~100 mixed (60% mobile, 20% arm, 20% panda) | Overhead orbit, 800×600 |
| `docs/media/100robots_grid_single.gif` | 100 mobile_robot.urdf | Overhead orbit, 800×600 |
| `docs/media/100robots_grid_arm.gif` | 100 arm_robot.urdf | Overhead orbit, 800×600 |

Parameters: 60 frames @ 15fps (4 seconds), camera orbits ~30° during clip.
Target size: < 5 MB each.

### 2. Model Catalog PNGs (7 files)

```
docs/media/models/{model_name}.png
```

Models to capture (from `robots/` directory):
- `mobile_robot.urdf` → `mobile_robot.png`
- `arm_robot.urdf` → `arm_robot.png`
- `simple_cube.urdf` → `simple_cube.png`
- `mobile_manipulator.urdf` → `mobile_manipulator.png`
- `rail_arm_robot.urdf` → `rail_arm_robot.png`
- `franka_panda/panda.urdf` (pybullet_data) → `panda.png`
- `kuka_iiwa/model.urdf` (pybullet_data) → `kuka_iiwa.png`

Resolution: 320×240 each. White/light background. Camera positioned to show full model.

### 3. Scripts

| Script | Purpose |
|--------|---------|
| `scripts/capture_demo.py` | Generate demo GIFs/MP4s with configurable robot type/count |
| `scripts/capture_model_catalog.py` | Generate individual model PNGs |

### 4. Documentation Embedding (Phase 1)

| File | Media | Location |
|------|-------|----------|
| `README.md` | `100robots_grid_mixed.gif` | Below badges, above "What is PyBulletFleet?" |
| `docs/examples/robot-models.md` | Model catalog PNGs in a grid | "Available Models" section |
| `docs/getting-started/quickstart.md` | `100robots_grid_mixed.gif` | Top of page or "Run a demo" section |

Markdown syntax: `![alt text](../media/100robots_grid_mixed.gif)`
README relative path: `![alt text](docs/media/100robots_grid_mixed.gif)`

### 5. Tutorial GIF Embedding (Phase 3)

| File | Media | Location |
|------|-------|----------|
| `docs/examples/action-system.md` | `action_system.gif` | Page top |
| `docs/examples/arm-pick-drop.md` | `pick_drop_arm.gif` | Page top |
| `docs/examples/collision-features.md` | `collision_features.gif` | Page top |
| `docs/examples/path-following.md` | `path_following.gif` | Page top |
| `docs/examples/mobile-manipulator.md` | `mobile_manipulator.gif` | Page top |
| `docs/examples/rail-arm.md` | `rail_arm.gif` | Page top |

Each GIF followed by: `▶ Watch full demo on YouTube` (link TBD in Phase 3).

## Technical Approach

### Frame Capture

```python
import pybullet as p

width, height = 800, 600
view = p.computeViewMatrix(cameraEyePosition, targetPosition, upVector)
proj = p.computeProjectionMatrixFOV(fov=60, aspect=width/height, nearVal=0.1, farVal=100)
_, _, rgba, _, _ = p.getCameraImage(width, height, view, proj, renderer=p.ER_TINY_RENDERER)
```

### GIF Assembly

```python
import imageio
import numpy as np

frames = []  # list of (H, W, 3) uint8 arrays
imageio.mimsave(output_path, frames, fps=15, loop=0)
```

### Palette Optimization

If GIF exceeds 5 MB, reduce:
1. Frame count (45 instead of 60)
2. Resolution (640×480)
3. Color quantization via `imageio` `quantizer='nq'`

## File References

- Robot URDFs: `robots/*.urdf`
- PyBullet data models: `pybullet_data.getDataPath()` → `franka_panda/`, `kuka_iiwa/`
- Existing examples (for camera angle reference): `examples/100robots_grid_demo.py`
- README: `README.md`
- Robot models tutorial: `docs/examples/robot-models.md`
- Quickstart: `docs/getting-started/quickstart.md`
- pyproject.toml: `include = ["pybullet_fleet", "pybullet_fleet.*"]` (media excluded from pip)

## Acceptance Criteria

1. `scripts/capture_demo.py` generates 3 GIFs headlessly (p.DIRECT)
2. `scripts/capture_model_catalog.py` generates 7 PNGs headlessly
3. All GIFs < 5 MB, all media total < 20 MB
4. README.md displays hero GIF on GitHub
5. robot-models.md displays model catalog
6. `make verify` passes (no regressions)
7. No new runtime dependencies (imageio is dev-only)
