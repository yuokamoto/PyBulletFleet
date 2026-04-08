#!/usr/bin/env python3
"""
capture_model_catalog.py — Generate PNG thumbnails for each robot model.

Captures a single frame per model headlessly using PyBullet's getCameraImage
(ER_TINY_RENDERER) in p.DIRECT mode.

Covers **all** local and ``pybullet_data`` models registered in
:data:`pybullet_fleet.robot_models.KNOWN_MODELS`.

Usage::

    python scripts/capture_model_catalog.py
    python scripts/capture_model_catalog.py --output-dir docs/media/models --width 320 --height 240
"""
import argparse
import math
import os
from typing import List

import numpy as np
import pybullet as p
import pybullet_data
from PIL import Image

# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

parser = argparse.ArgumentParser(description="Generate robot model catalog PNGs")
parser.add_argument("--output-dir", default="docs/media/models", help="Output directory (default: docs/media/models)")
parser.add_argument("--width", type=int, default=320, help="Image width (default: 320)")
parser.add_argument("--height", type=int, default=240, help="Image height (default: 240)")
args = parser.parse_args()

# ---------------------------------------------------------------------------
# Project root
# ---------------------------------------------------------------------------

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

# ---------------------------------------------------------------------------
# Model definitions
#
# Each entry: (name, rel_path, use_fixed_base, is_pybullet_data)
# Mirrors the local + pybullet_data tiers of KNOWN_MODELS in robot_models.py.
# ---------------------------------------------------------------------------

MODELS = [
    # --- Tier 0: local (robots/ directory) ---
    ("mobile_robot", "robots/mobile_robot.urdf", False, False),
    ("arm_robot", "robots/arm_robot.urdf", True, False),
    ("simple_cube", "robots/simple_cube.urdf", False, False),
    ("mobile_manipulator", "robots/mobile_manipulator.urdf", False, False),
    ("rail_arm_robot", "robots/rail_arm_robot.urdf", True, False),
    # --- Tier 1: pybullet_data (always available) ---
    ("panda", "franka_panda/panda.urdf", True, True),
    ("kuka_iiwa", "kuka_iiwa/model.urdf", True, True),
    ("husky", "husky/husky.urdf", False, True),
    ("racecar", "racecar/racecar.urdf", False, True),
    ("a1", "a1/a1.urdf", False, True),
    ("laikago", "laikago/laikago.urdf", False, True),
    ("aliengo", "aliengo/aliengo.urdf", False, True),
    ("mini_cheetah", "mini_cheetah/mini_cheetah.urdf", False, True),
    ("minitaur", "quadruped/minitaur.urdf", False, True),
    ("xarm6", "xarm/xarm6_with_gripper.urdf", True, True),
    # pybullet_data: objects / furniture
    ("table", "table/table.urdf", True, True),
    ("table_square", "table_square/table_square.urdf", True, True),
    ("tray", "tray/tray.urdf", False, True),
    ("plane", "plane.urdf", True, True),
    ("kiva_shelf", "kiva_shelf/model.sdf", True, True),
    ("wsg50_gripper", "gripper/wsg50_one_motor_gripper_new.sdf", True, True),
    ("domino", "domino/domino.urdf", False, True),
    ("jenga", "jenga/jenga.urdf", False, True),
    ("lego", "lego/lego.urdf", False, True),
    ("mug", "objects/mug.urdf", False, True),
]

# ---------------------------------------------------------------------------
# Display poses for arm robots (joint index → angle in radians).
# These show the arm in a natural, visually appealing configuration.
# ---------------------------------------------------------------------------

DISPLAY_POSES: dict = {
    "kuka_iiwa": {0: 0.3, 1: -0.5, 2: 0.2, 3: -1.2, 4: 0.0, 5: 0.8, 6: 0.0},
    "panda": {0: 0.3, 1: -0.5, 2: 0.2, 3: -1.5, 4: 0.0, 5: 1.2, 6: 0.8},
    "arm_robot": {0: 0.3, 1: -0.6, 2: 0.2, 3: -1.0, 4: 0.0, 5: 0.8},
    "mobile_manipulator": {0: 0.3, 1: -0.6, 2: 0.2, 3: -1.0, 4: 0.0, 5: 0.8},
    "rail_arm_robot": {1: 0.3, 2: -0.6, 3: 0.2, 4: -1.0, 5: 0.0, 6: 0.8},
    "xarm6": {0: 0.3, 1: -0.5, 2: 0.2, 3: -1.2, 4: 0.0, 5: 0.8},
}


def _compute_full_aabb(body_id: int):
    """Compute the combined AABB across all links of a body."""
    aabb_min_all = list(p.getAABB(body_id, -1)[0])
    aabb_max_all = list(p.getAABB(body_id, -1)[1])
    for j in range(p.getNumJoints(body_id)):
        link_min, link_max = p.getAABB(body_id, j)
        for i in range(3):
            aabb_min_all[i] = min(aabb_min_all[i], link_min[i])
            aabb_max_all[i] = max(aabb_max_all[i], link_max[i])
    return aabb_min_all, aabb_max_all


def _load_model(full_path: str, use_fixed_base: bool) -> List[int]:
    """Load a URDF or SDF file. Returns list of body IDs."""
    ext = os.path.splitext(full_path)[1].lower()
    if ext == ".sdf":
        return list(p.loadSDF(full_path))
    else:
        return [p.loadURDF(full_path, [0, 0, 0], useFixedBase=use_fixed_base)]


def capture_model(
    name: str, rel_path: str, use_fixed_base: bool, is_pybullet_data: bool, width: int, height: int
) -> np.ndarray:
    """Load a model and capture a single frame. Returns RGB array (H, W, 3)."""
    client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client)
    p.setGravity(0, 0, -9.81, physicsClientId=client)

    # Resolve path
    if is_pybullet_data:
        full_path = os.path.join(pybullet_data.getDataPath(), rel_path)
    else:
        full_path = os.path.join(PROJECT_ROOT, rel_path)

    body_ids = _load_model(full_path, use_fixed_base)
    body_id = body_ids[0]  # use first body for framing

    # Set joints to display pose (model-specific or generic fallback)
    num_joints = p.getNumJoints(body_id)
    preset = DISPLAY_POSES.get(name, {})
    for j in range(num_joints):
        info = p.getJointInfo(body_id, j)
        if info[2] not in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
            continue
        if j in preset:
            target = preset[j]
        else:
            lower, upper = info[8], info[9]
            if lower < upper:
                target = lower + 0.3 * (upper - lower)
            else:
                target = 0.3 if info[2] == p.JOINT_REVOLUTE else 0.0
        p.resetJointState(body_id, j, target)

    # Compute combined AABB across all bodies / links for proper framing
    aabb_min_all = [float("inf")] * 3
    aabb_max_all = [float("-inf")] * 3
    for bid in body_ids:
        bmin, bmax = _compute_full_aabb(bid)
        for i in range(3):
            aabb_min_all[i] = min(aabb_min_all[i], bmin[i])
            aabb_max_all[i] = max(aabb_max_all[i], bmax[i])

    center = [(aabb_min_all[i] + aabb_max_all[i]) / 2 for i in range(3)]
    size = [aabb_max_all[i] - aabb_min_all[i] for i in range(3)]
    max_dim = max(size)

    # Camera distance based on model size
    distance = max(max_dim * 2.5, 0.8)
    cam_height = center[2] + max_dim * 0.6

    # Camera angle: 45° azimuth, elevated
    angle = math.radians(45)
    eye = [
        center[0] + distance * math.cos(angle),
        center[1] + distance * math.sin(angle),
        cam_height,
    ]

    view_matrix = p.computeViewMatrix(
        cameraEyePosition=eye,
        cameraTargetPosition=center,
        cameraUpVector=[0, 0, 1],
    )
    proj_matrix = p.computeProjectionMatrixFOV(fov=45, aspect=width / height, nearVal=0.01, farVal=50)

    _, _, rgba, _, _ = p.getCameraImage(
        width, height, viewMatrix=view_matrix, projectionMatrix=proj_matrix, renderer=p.ER_TINY_RENDERER
    )

    rgb = np.array(rgba, dtype=np.uint8).reshape(height, width, 4)[:, :, :3]

    p.disconnect(physicsClientId=client)
    return rgb


def main():
    os.makedirs(args.output_dir, exist_ok=True)

    print(f"Generating model catalog PNGs ({args.width}x{args.height})...")
    for name, rel_path, fixed, is_pb in MODELS:
        print(f"  {name}...", end=" ", flush=True)
        try:
            rgb = capture_model(name, rel_path, fixed, is_pb, args.width, args.height)
            output_path = os.path.join(args.output_dir, f"{name}.png")
            Image.fromarray(rgb).save(output_path, optimize=True)
            file_size_kb = os.path.getsize(output_path) / 1024
            print(f"OK ({file_size_kb:.0f} KB)")
        except Exception as e:
            print(f"FAILED: {e}")

    print(f"\nDone. Files saved to {args.output_dir}/")


if __name__ == "__main__":
    main()
