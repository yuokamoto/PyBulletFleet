# Visual Content for PyBulletFleet Documentation

## Problem

PyBulletFleet's README, tutorials, and example documentation contain **zero** visual content.
For a robotics simulation framework, this makes it difficult for visitors to grasp the project's
value within seconds. Screenshots and animations are the most effective way to communicate
"100+ robots at real-time speed" at a glance.

## Design

### Capture Method

PyBullet `getCameraImage()` + `ER_TINY_RENDERER` in `p.DIRECT` mode (headless).
No GUI or display server required. Validated in prototype: 800×600, 60 frames @ 15fps → 2.2 MB GIF.

### Phase 1 — Hero GIFs + Model Catalog (current scope)

| Deliverable | Format | Content | Placement |
|-------------|--------|---------|-----------|
| `100robots_grid_mixed.gif` | GIF 800×600, 3-5s loop | arm + mobile mixed fleet (~100 robots) | README top, docs index |
| `100robots_grid_single.gif` | GIF 800×600, 3-5s loop | 100 mobile robots patrolling | README "Performance" area |
| `100robots_grid_arm.gif` | GIF 800×600, 3-5s loop | 100 arm robots working | README "Robot Models" area |
| Model catalog PNGs | PNG 320×240 each | Individual robot model thumbnails | docs/examples/robot-models.md |

**File structure:**

```
docs/media/
├── 100robots_grid_mixed.gif    ← mixed arm+mobile (Phase 1)
├── 100robots_grid_single.gif   ← mobile only (Phase 1)
├── 100robots_grid_arm.gif      ← arm only (Phase 1)
├── action_system.gif           ← tutorial GIFs (Phase 3)
├── pick_drop_arm.gif
├── collision_features.gif
├── path_following.gif
├── mobile_manipulator.gif
├── rail_arm.gif
└── models/                     ← robot model catalog PNGs (Phase 1)
    ├── mobile_robot.png
    ├── arm_robot.png
    ├── panda.png
    ├── kuka_iiwa.png
    ├── mobile_manipulator.png
    ├── rail_arm_robot.png
    └── simple_cube.png

scripts/
├── capture_demo.py             ← demo GIF/MP4 generator
└── capture_model_catalog.py    ← model thumbnail generator
```

**Size budget:** < 5 MB per GIF, < 20 MB total. No Git LFS needed at this stage.

**pip install unaffected:** `pyproject.toml` `include = ["pybullet_fleet", "pybullet_fleet.*"]`
excludes `docs/` from the published package.

### Phase 2 — SimulationRecorder

Framework-integrated recording tool. Full spec: `docs/design/simulation-recorder/spec.md`

```python
# Python API — explicit opt-in
sim.start_recording("docs/media/demo.gif", duration=4.0)
sim.run_simulation(duration=5.0)

# Environment variable — zero code changes to any demo
# RECORD=output.gif python examples/scale/100robots_grid_demo.py
```

`capture_demo.py` will be refactored to invoke demos via `RECORD` env var
instead of spawning fake standalone scenes.

### Phase 3 — Tutorial GIFs + YouTube (future)

All tutorial pages get a GIF (Robot Models catalog is the exception — PNG grid).

| Page | Media | Content |
|------|-------|---------|
| action-system.md | `action_system.gif` | Robot executing action queue |
| pick-drop-arm.md | `pick_drop_arm.gif` | Arm pick → drop motion |
| collision-features.md | `collision_features.gif` | Collision detection visualization |
| path-following.md | `path_following.gif` | Path-following trajectory |
| mobile-manipulator.md | `mobile_manipulator.gif` | Mobile manipulator motion |
| rail-arm.md | `rail_arm.gif` | Rail arm reciprocating motion |

**YouTube links** for longer demos (30s+, with narration):
- GIF (3-5s, silent, loop) → inline first impression
- Below each GIF: `▶ Watch full demo on YouTube` link for detailed walkthrough

## Dependencies

- `imageio` (already in dev dependencies)
- `Pillow` (already installed)
- `numpy` (already installed)
- `pybullet` (core dependency)

## Constraints

- All GIFs generated headless (`p.DIRECT`) — no display server dependency
- GIF < 5 MB each (optimize frame count and palette)
- No Git LFS unless total media exceeds 20 MB
- Scripts are idempotent (re-running regenerates identical output)
