# Visual Content Phase 1 — Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use executing-plans to implement this plan task-by-task.

**Goal:** Add demo GIFs and model-catalog PNGs to README and docs.

**Architecture:** Two standalone scripts (`capture_demo.py`, `capture_model_catalog.py`) use PyBullet `getCameraImage` in headless `p.DIRECT` mode to generate media files under `docs/media/`. Markdown embeds reference these files with relative paths.

**Tech Stack:** PyBullet (ER_TINY_RENDERER), imageio (GIF), Pillow/numpy (PNG), Python ≥ 3.10

---

### Task 1: Create `scripts/capture_demo.py` — demo GIF generator (SERIAL)

**Files:**
- Create: `scripts/capture_demo.py`

**Step 1: Write the script**

The script:
- Accepts `--mode` (mixed / mobile / arm), `--frames` (default 60), `--fps` (default 15), `--width`/`--height`, `--output`
- Connects `p.DIRECT`
- Loads a ground plane
- Spawns ~100 robots on a grid (10×10, spacing 2.0)
  - `mixed`: 60% `robots/mobile_robot.urdf`, 20% `robots/arm_robot.urdf`, 20% `franka_panda/panda.urdf`
  - `mobile`: 100% `robots/mobile_robot.urdf`
  - `arm`: 100% `robots/arm_robot.urdf`
- For each frame: orbits camera ~30° total, captures via `getCameraImage(ER_TINY_RENDERER)`, appends RGBA→RGB numpy array
- Saves GIF via `imageio.mimsave(path, frames, fps=fps, loop=0)`

Camera setup:
```python
center = [grid_center_x, grid_center_y, 0.3]
radius = grid_extent * 1.2
for i in range(num_frames):
    angle = start_angle + (i / num_frames) * orbit_degrees
    eye = [center[0] + radius * cos(angle), center[1] + radius * sin(angle), height]
    view = p.computeViewMatrix(eye, center, [0, 0, 1])
    proj = p.computeProjectionMatrixFOV(50, w/h, 0.1, 200)
```

For arm robots, apply random joint positions each frame for visual variety.

**Step 2: Test script runs and produces output**

```bash
python scripts/capture_demo.py --mode mixed --output docs/media/100robots_grid_mixed.gif
python scripts/capture_demo.py --mode mobile --output docs/media/100robots_grid_single.gif
python scripts/capture_demo.py --mode arm --output docs/media/100robots_grid_arm.gif
```

Verify: each GIF exists, < 5 MB, correct dimensions.

---

### Task 2: Create `scripts/capture_model_catalog.py` — model thumbnail generator (SERIAL, after Task 1)

**Files:**
- Create: `scripts/capture_model_catalog.py`

**Step 1: Write the script**

The script:
- Accepts `--output-dir` (default `docs/media/models`), `--width`/`--height` (default 320×240)
- For each model in a curated list:
  - Connects `p.DIRECT`, loads ground plane
  - Loads model URDF (using `resolve_model` or direct pybullet_data path)
  - Computes AABB to auto-frame camera (distance = max(aabb_size) * 2.5)
  - Captures single frame
  - Saves as PNG via Pillow
  - Disconnects

Models list:
```python
MODELS = [
    ("mobile_robot", "robots/mobile_robot.urdf", False),
    ("arm_robot", "robots/arm_robot.urdf", True),
    ("simple_cube", "robots/simple_cube.urdf", False),
    ("mobile_manipulator", "robots/mobile_manipulator.urdf", False),
    ("rail_arm_robot", "robots/rail_arm_robot.urdf", True),
    ("panda", "franka_panda/panda.urdf", True),       # pybullet_data
    ("kuka_iiwa", "kuka_iiwa/model.urdf", True),       # pybullet_data
]
```

**Step 2: Test script runs**

```bash
python scripts/capture_model_catalog.py
ls -la docs/media/models/
```

Verify: 7 PNG files, each ~100-500 KB.

---

### Task 3: Generate all media files (SERIAL, after Tasks 1-2)

**Step 1: Create docs/media directories**

```bash
mkdir -p docs/media/models
```

**Step 2: Run both scripts**

```bash
python scripts/capture_demo.py --mode mixed --output docs/media/100robots_grid_mixed.gif
python scripts/capture_demo.py --mode mobile --output docs/media/100robots_grid_single.gif
python scripts/capture_demo.py --mode arm --output docs/media/100robots_grid_arm.gif
python scripts/capture_model_catalog.py
```

**Step 3: Verify sizes**

```bash
du -sh docs/media/*.gif docs/media/models/*.png
# Each GIF < 5 MB, total < 20 MB
```

---

### Task 4: Embed demo GIF in README.md (SERIAL, after Task 3)

**Files:**
- Modify: `README.md` — lines 1-8 (between badges and "What is PyBulletFleet?")

**Step 1: Add GIF below badges**

Insert after the badges line and blank line, before `## What is PyBulletFleet?`:

```markdown
<p align="center">
  <img src="docs/media/100robots_grid_mixed.gif" alt="100 robots simulation" width="800">
</p>
```

**Step 2: Verify on GitHub**

Check that the relative path `docs/media/100robots_grid_mixed.gif` will resolve correctly from GitHub repo root.

---

### Task 5: Add model catalog to `docs/examples/robot-models.md` (SERIAL, after Task 3)

**Files:**
- Modify: `docs/examples/robot-models.md` — add image grid in an appropriate section

**Step 1: Add model thumbnails**

Find a good insertion point (after the model table or in a new "Visual Catalog" subsection). Add:

```markdown
## Visual Catalog

| Model | Preview |
|-------|---------|
| mobile_robot | ![mobile_robot](../media/models/mobile_robot.png) |
| arm_robot | ![arm_robot](../media/models/arm_robot.png) |
| panda | ![panda](../media/models/panda.png) |
| kuka_iiwa | ![kuka_iiwa](../media/models/kuka_iiwa.png) |
| mobile_manipulator | ![mobile_manipulator](../media/models/mobile_manipulator.png) |
| rail_arm_robot | ![rail_arm_robot](../media/models/rail_arm_robot.png) |
| simple_cube | ![simple_cube](../media/models/simple_cube.png) |
```

---

### Task 6: Add demo GIF to quickstart.md (SERIAL, after Task 3)

**Files:**
- Modify: `docs/getting-started/quickstart.md` — top of page

**Step 1: Add GIF**

Insert at top after the heading:

```markdown
![100 robots demo](../media/100robots_grid_mixed.gif)
```

---

### Task 7: Run `make verify` (SERIAL, final)

**Step 1: Run verification**

```bash
make verify
```

Expected: all tests pass, linter clean, no regressions.

**Step 2: Check docs build**

```bash
make docs
```

Expected: no warnings (media files referenced correctly).

---

## Task Dependencies

```
Task 1 (demo script) ──→ Task 3 (generate media) ──→ Task 4 (README embed)
Task 2 (catalog script) ─┘                        ├─→ Task 5 (robot-models.md)
                                                   ├─→ Task 6 (quickstart.md)
                                                   └─→ Task 7 (verify)
```

**SERIAL tasks (must be in order):** Tasks 1 → 2 → 3 → 4/5/6 → 7
**PARALLEL tasks:** Tasks 4, 5, 6 can be done concurrently after Task 3
