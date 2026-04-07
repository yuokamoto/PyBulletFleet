#!/usr/bin/env python3
"""
capture_model_catalog.py — Generate PNG thumbnails for each robot model.

Captures a single frame per model headlessly using PyBullet's getCameraImage
(ER_TINY_RENDERER) in p.DIRECT mode.

Usage::

    python scripts/capture_model_catalog.py
    python scripts/capture_model_catalog.py --output-dir docs/media/models --width 320 --height 240
"""
import argparse
import math
import os

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
# Model definitions: (name, urdf_path, use_fixed_base, is_pybullet_data)
# ---------------------------------------------------------------------------

MODELS = [
    ("mobile_robot", "robots/mobile_robot.urdf", False, False),
    ("arm_robot", "robots/arm_robot.urdf", True, False),
    ("simple_cube", "robots/simple_cube.urdf", False, False),
    ("mobile_manipulator", "robots/mobile_manipulator.urdf", False, False),
    ("rail_arm_robot", "robots/rail_arm_robot.urdf", True, False),
    ("panda", "franka_panda/panda.urdf", True, True),
    ("kuka_iiwa", "kuka_iiwa/model.urdf", True, True),
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


def capture_model(
    name: str, urdf_path: str, use_fixed_base: bool, is_pybullet_data: bool, width: int, height: int
) -> np.ndarray:
    """Load a model and capture a single frame. Returns RGB array (H, W, 3)."""
    client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    # Load model
    if is_pybullet_data:
        full_path = urdf_path  # pybullet_data search path handles it
    else:
        full_path = os.path.join(PROJECT_ROOT, urdf_path)

    body_id = p.loadURDF(full_path, [0, 0, 0], useFixedBase=use_fixed_base)

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

    # Compute combined AABB across all links for proper framing
    aabb_min, aabb_max = _compute_full_aabb(body_id)
    center = [(aabb_min[i] + aabb_max[i]) / 2 for i in range(3)]
    size = [aabb_max[i] - aabb_min[i] for i in range(3)]
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

    p.disconnect()
    return rgb


def main():
    os.makedirs(args.output_dir, exist_ok=True)

    print(f"Generating model catalog PNGs ({args.width}x{args.height})...")
    for name, urdf_path, fixed, is_pb in MODELS:
        print(f"  {name}...", end=" ", flush=True)
        try:
            rgb = capture_model(name, urdf_path, fixed, is_pb, args.width, args.height)
            output_path = os.path.join(args.output_dir, f"{name}.png")
            Image.fromarray(rgb).save(output_path, optimize=True)
            file_size_kb = os.path.getsize(output_path) / 1024
            print(f"OK ({file_size_kb:.0f} KB)")
        except Exception as e:
            print(f"FAILED: {e}")

    print(f"\nDone. Files saved to {args.output_dir}/")


if __name__ == "__main__":
    main()
