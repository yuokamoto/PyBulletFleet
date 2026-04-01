# Robot Models Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use executing-plans to implement this plan task-by-task.

**Goal:** Add a URDF resolver and auto-detection system so demos can use real-world robot models (Panda, KUKA, Husky, etc.) with a `--robot` flag while keeping existing behavior as default.

**Architecture:** A single new module `pybullet_fleet/robot_models.py` provides `resolve_urdf()` (registry-based name → path resolver) and `auto_detect_profile()` (PyBullet introspection → RobotProfile dataclass). Demos add an optional `--robot` CLI arg. No new required dependencies.

**Tech Stack:** Python 3.10+, PyBullet, pybullet_data (bundled), optional: robot_descriptions, ament_index_python

---

## Task 1: `resolve_urdf()` — Tier 1 (pybullet_data) [SERIAL]

**Files:**
- Create: `pybullet_fleet/robot_models.py`
- Create: `tests/test_robot_models.py`

**Step 1: Write failing tests**

```python
# tests/test_robot_models.py
"""Tests for robot model resolution and auto-detection."""

import os
import pytest
import pybullet_data

from pybullet_fleet.robot_models import resolve_urdf, KNOWN_MODELS, list_available_models


# Collect Tier 1 model names from registry (always available)
_TIER1_MODELS = [name for name, entry in KNOWN_MODELS.items() if entry.tier == "pybullet_data"]
_ROS_MODELS = [name for name, entry in KNOWN_MODELS.items() if entry.tier == "ros"]
_RD_MODELS = [name for name, entry in KNOWN_MODELS.items() if entry.tier == "robot_descriptions"]


class TestResolveUrdf:
    """resolve_urdf() resolves robot names to absolute URDF paths."""

    @pytest.mark.parametrize("model_name", _TIER1_MODELS)
    def test_tier1_model_resolves_to_existing_file(self, model_name):
        path = resolve_urdf(model_name)
        assert os.path.isfile(path), f"{model_name} resolved to {path} but file does not exist"
        assert path.endswith((".urdf", ".sdf"))

    @pytest.mark.parametrize("model_name", _ROS_MODELS)
    def test_ros_model_raises_with_install_hint(self, model_name):
        """Tier 2 models raise FileNotFoundError with apt install hint (no ROS env in tests)."""
        with pytest.raises(FileNotFoundError, match="apt install"):
            resolve_urdf(model_name)

    @pytest.mark.parametrize("model_name", _RD_MODELS)
    def test_robot_descriptions_model_raises_with_install_hint(self, model_name):
        """Tier 3 models raise FileNotFoundError with pip install hint."""
        with pytest.raises(FileNotFoundError, match="pip install"):
            resolve_urdf(model_name)

    def test_absolute_path_returned_as_is(self):
        path = resolve_urdf("/tmp/some_robot.urdf")
        assert path == "/tmp/some_robot.urdf"

    def test_relative_path_returned_as_is(self):
        path = resolve_urdf("robots/arm_robot.urdf")
        assert path == "robots/arm_robot.urdf"

    def test_unknown_name_raises_file_not_found(self):
        with pytest.raises(FileNotFoundError, match="nonexistent_robot"):
            resolve_urdf("nonexistent_robot")


class TestListAvailableModels:
    """list_available_models() reports model availability."""

    def test_returns_dict_with_all_known_models(self):
        result = list_available_models()
        assert isinstance(result, dict)
        assert set(result.keys()) == set(KNOWN_MODELS.keys())

    def test_tier1_models_all_available(self):
        result = list_available_models()
        for name in _TIER1_MODELS:
            assert result[name]["available"] is True, f"{name} should be available"
            assert result[name]["tier"] == "pybullet_data"
```

**Step 2: Run tests to verify they fail**

Run: `pytest tests/test_robot_models.py -v`
Expected: FAIL (ModuleNotFoundError: No module named 'pybullet_fleet.robot_models')

**Step 3: Write minimal implementation**

```python
# pybullet_fleet/robot_models.py
"""Robot model URDF resolution and auto-detection.

Provides a 3-tier URDF resolution system:
- Tier 1 (pybullet_data): Always available with pybullet install
- Tier 2 (ROS packages): Available when ROS description packages installed
- Tier 3 (robot_descriptions): Available via `pip install robot_descriptions`

Usage::

    from pybullet_fleet.robot_models import resolve_urdf

    urdf = resolve_urdf("panda")           # Tier 1: pybullet_data
    urdf = resolve_urdf("robots/arm.urdf") # Direct path: returned as-is
"""

import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import pybullet_data

from pybullet_fleet.logging_utils import get_lazy_logger

logger = get_lazy_logger(__name__)


@dataclass(frozen=True)
class ModelEntry:
    """Registry entry for a known robot model."""

    rel_path: str
    tier: str  # "pybullet_data", "ros", "robot_descriptions"
    install_hint: str = ""


# ---------------------------------------------------------------------------
# Known models registry
# ---------------------------------------------------------------------------

KNOWN_MODELS: Dict[str, ModelEntry] = {
    # Tier 1: pybullet_data (always available with pybullet)
    "panda": ModelEntry("franka_panda/panda.urdf", "pybullet_data"),
    "kuka_iiwa": ModelEntry("kuka_iiwa/model.urdf", "pybullet_data"),
    "husky": ModelEntry("husky/husky.urdf", "pybullet_data"),
    "racecar": ModelEntry("racecar/racecar.urdf", "pybullet_data"),
    "table": ModelEntry("table/table.urdf", "pybullet_data"),
    "tray": ModelEntry("tray/tray.urdf", "pybullet_data"),
    "plane": ModelEntry("plane.urdf", "pybullet_data"),
    "a1": ModelEntry("a1/a1.urdf", "pybullet_data"),
    "laikago": ModelEntry("laikago/laikago.urdf", "pybullet_data"),
    # Tier 2: ROS packages
    "ur5e": ModelEntry(
        "ur_description/urdf/ur5e.urdf",
        "ros",
        "apt install ros-${ROS_DISTRO}-ur-description",
    ),
    "turtlebot3": ModelEntry(
        "turtlebot3_description",
        "ros",
        "apt install ros-${ROS_DISTRO}-turtlebot3-description",
    ),
    "fetch": ModelEntry(
        "fetch_description",
        "ros",
        "apt install ros-${ROS_DISTRO}-fetch-description",
    ),
    # Tier 3: robot_descriptions (pip)
    "tiago": ModelEntry(
        "tiago_description",
        "robot_descriptions",
        "pip install robot_descriptions",
    ),
    "pr2": ModelEntry(
        "pr2_description",
        "robot_descriptions",
        "pip install robot_descriptions",
    ),
}


def resolve_urdf(name_or_path: str) -> str:
    """Resolve a robot name or file path to an absolute URDF path.

    Resolution order:
    1. If ``name_or_path`` contains ``/`` or ``\\`` or ends with ``.urdf``/``.sdf``,
       treat as a direct file path and return as-is.
    2. Look up in ``KNOWN_MODELS`` registry and resolve by tier.
    3. Raise ``FileNotFoundError`` with install instructions.

    Args:
        name_or_path: Robot name (e.g. "panda", "husky") or file path.

    Returns:
        Absolute path to the URDF file.

    Raises:
        FileNotFoundError: If the model cannot be resolved.
    """
    # Direct path — return as-is
    if os.sep in name_or_path or "/" in name_or_path or name_or_path.endswith((".urdf", ".sdf")):
        return name_or_path

    # Known model lookup
    if name_or_path in KNOWN_MODELS:
        entry = KNOWN_MODELS[name_or_path]
        return _resolve_by_tier(name_or_path, entry)

    # Unknown name
    available = ", ".join(sorted(KNOWN_MODELS.keys()))
    raise FileNotFoundError(
        f"Unknown robot model '{name_or_path}'. "
        f"Available models: {available}"
    )


def _resolve_by_tier(name: str, entry: ModelEntry) -> str:
    """Resolve a ModelEntry based on its tier."""
    if entry.tier == "pybullet_data":
        return os.path.join(pybullet_data.getDataPath(), entry.rel_path)

    if entry.tier == "ros":
        return _resolve_ros(name, entry)

    if entry.tier == "robot_descriptions":
        return _resolve_robot_descriptions(name, entry)

    raise FileNotFoundError(f"Unknown tier '{entry.tier}' for model '{name}'")


def _resolve_ros(name: str, entry: ModelEntry) -> str:
    """Resolve via ROS package share directory."""
    try:
        from ament_index_python.packages import get_package_share_directory

        pkg_name = entry.rel_path.split("/")[0]
        pkg_dir = get_package_share_directory(pkg_name)
        # If rel_path has sub-paths, join them
        parts = entry.rel_path.split("/")[1:]
        if parts:
            return os.path.join(pkg_dir, *parts)
        # Search for .urdf files in the package
        for root, _dirs, files in os.walk(pkg_dir):
            for f in files:
                if f.endswith(".urdf"):
                    return os.path.join(root, f)
        raise FileNotFoundError(f"No URDF found in package '{pkg_name}'")
    except ImportError:
        pass

    # Fallback: try rospkg (ROS1)
    try:
        import rospkg

        rospack = rospkg.RosPack()
        pkg_name = entry.rel_path.split("/")[0]
        pkg_dir = rospack.get_path(pkg_name)
        parts = entry.rel_path.split("/")[1:]
        if parts:
            return os.path.join(pkg_dir, *parts)
        for root, _dirs, files in os.walk(pkg_dir):
            for f in files:
                if f.endswith(".urdf"):
                    return os.path.join(root, f)
        raise FileNotFoundError(f"No URDF found in package '{pkg_name}'")
    except ImportError:
        pass

    raise FileNotFoundError(
        f"Robot '{name}' requires a ROS package. "
        f"Install with: {entry.install_hint}"
    )


def _resolve_robot_descriptions(name: str, entry: ModelEntry) -> str:
    """Resolve via robot_descriptions pip package."""
    try:
        import importlib

        mod = importlib.import_module(f"robot_descriptions.{entry.rel_path}")
        urdf_path = getattr(mod, "URDF_PATH", None)
        if urdf_path and os.path.isfile(urdf_path):
            return urdf_path
        raise FileNotFoundError(f"URDF_PATH not found in robot_descriptions.{entry.rel_path}")
    except ImportError:
        raise FileNotFoundError(
            f"Robot '{name}' requires the robot_descriptions package. "
            f"Install with: {entry.install_hint}"
        )


def list_available_models() -> Dict[str, dict]:
    """Check availability of all known models.

    Returns:
        Dict mapping model name to {tier, available, path_or_error}.
    """
    result = {}
    for name, entry in KNOWN_MODELS.items():
        try:
            path = resolve_urdf(name)
            result[name] = {
                "tier": entry.tier,
                "available": os.path.isfile(path),
                "path": path,
            }
        except FileNotFoundError as e:
            result[name] = {
                "tier": entry.tier,
                "available": False,
                "error": str(e),
            }
    return result
```

**Step 4: Run tests to verify they pass**

Run: `pytest tests/test_robot_models.py -v`
Expected: ALL PASS

**Step 5: Commit**

```bash
git add pybullet_fleet/robot_models.py tests/test_robot_models.py
git commit -m "feat: add resolve_urdf and robot model registry (Tier 1)"
```

---

## Task 2: `RobotProfile` auto-detection [SERIAL — depends on Task 1]

**Files:**
- Modify: `pybullet_fleet/robot_models.py`
- Modify: `tests/test_robot_models.py`

**Step 1: Write failing tests**

Add to `tests/test_robot_models.py`:

```python
import pybullet as p

from pybullet_fleet.robot_models import (
    RobotProfile,
    auto_detect_profile,
    resolve_urdf,
    KNOWN_MODELS,
)

_TIER1_MODELS = [name for name, entry in KNOWN_MODELS.items() if entry.tier == "pybullet_data"]


class TestAutoDetectProfile:
    """auto_detect_profile() introspects URDF via PyBullet."""

    @pytest.fixture(autouse=True)
    def _setup_pybullet(self):
        self.client = p.connect(p.DIRECT)
        yield
        p.disconnect(self.client)

    @pytest.mark.parametrize("model_name", _TIER1_MODELS)
    def test_tier1_model_profile_has_valid_fields(self, model_name):
        """Every Tier 1 model loads and produces a valid profile."""
        path = resolve_urdf(model_name)
        profile = auto_detect_profile(path, self.client)
        assert profile.urdf_path == path
        assert profile.robot_type in ("arm", "mobile", "mobile_manipulator", "unknown")
        assert profile.num_joints > 0
        assert len(profile.joint_max_velocities) == len(profile.movable_joint_names)

    def test_panda_ee_link_is_panda_hand(self):
        """Panda EE heuristic should find 'panda_hand'."""
        profile = auto_detect_profile(resolve_urdf("panda"), self.client)
        assert profile.ee_link_name == "panda_hand"
        assert profile.robot_type == "arm"

    def test_husky_detected_as_mobile_no_ee(self):
        profile = auto_detect_profile(resolve_urdf("husky"), self.client)
        assert profile.robot_type == "mobile"
        assert profile.ee_link_name is None

    def test_override_merges_with_detected(self):
        profile = auto_detect_profile(
            resolve_urdf("panda"), self.client, ee_link_name="custom_link"
        )
        assert profile.ee_link_name == "custom_link"
        assert profile.robot_type == "arm"  # still auto-detected

    def test_override_robot_type(self):
        profile = auto_detect_profile(
            resolve_urdf("panda"), self.client, robot_type="mobile_manipulator"
        )
        assert profile.robot_type == "mobile_manipulator"
```

**Step 2: Run tests to verify they fail**

Run: `pytest tests/test_robot_models.py::TestAutoDetectProfile -v`
Expected: FAIL (ImportError: cannot import name 'RobotProfile')

**Step 3: Write implementation**

Add to `pybullet_fleet/robot_models.py`:

```python
import pybullet as p


@dataclass
class RobotProfile:
    """Auto-detected robot profile from URDF introspection.

    All fields can be overridden after detection or via
    ``auto_detect_profile(..., **overrides)``.
    """

    urdf_path: str
    robot_type: str = "unknown"  # "arm", "mobile", "mobile_manipulator"
    ee_link_name: Optional[str] = None
    movable_joint_names: List[str] = field(default_factory=list)
    joint_lower_limits: List[float] = field(default_factory=list)
    joint_upper_limits: List[float] = field(default_factory=list)
    joint_max_velocities: List[float] = field(default_factory=list)
    num_joints: int = 0


_EE_KEYWORDS = ("hand", "ee", "end_effector", "tool", "gripper", "tcp")


def auto_detect_profile(
    urdf_path: str,
    physics_client: int,
    **overrides,
) -> RobotProfile:
    """Load a URDF temporarily and introspect its structure.

    Args:
        urdf_path: Absolute path to the URDF file.
        physics_client: PyBullet physics client ID.
        **overrides: Override any auto-detected field.

    Returns:
        RobotProfile with detected (or overridden) values.
    """
    body_id = p.loadURDF(urdf_path, useFixedBase=True, physicsClientId=physics_client)
    try:
        profile = _introspect_body(body_id, urdf_path, physics_client)
    finally:
        p.removeBody(body_id, physicsClientId=physics_client)

    # Apply overrides
    for key, value in overrides.items():
        if hasattr(profile, key):
            object.__setattr__(profile, key, value)
        else:
            logger.warning("Unknown RobotProfile field: %s", key)

    return profile


def _introspect_body(body_id: int, urdf_path: str, physics_client: int) -> RobotProfile:
    """Extract joint info, detect EE link and robot type."""
    n_joints = p.getNumJoints(body_id, physicsClientId=physics_client)

    movable_names = []
    lower_limits = []
    upper_limits = []
    max_velocities = []
    link_names = []
    has_continuous = False
    has_revolute_or_prismatic = False
    ee_link: Optional[str] = None

    # Build parent→children map for leaf detection
    child_indices = set()
    for i in range(n_joints):
        info = p.getJointInfo(body_id, i, physicsClientId=physics_client)
        parent_idx = info[16]
        if parent_idx >= 0:
            child_indices.add(parent_idx)

    for i in range(n_joints):
        info = p.getJointInfo(body_id, i, physicsClientId=physics_client)
        joint_type = info[2]
        joint_name = info[1].decode("utf-8")
        link_name = info[12].decode("utf-8")
        lower = info[8]
        upper = info[9]
        max_vel = info[11]

        link_names.append(link_name)

        # Track joint types
        if joint_type == p.JOINT_REVOLUTE or joint_type == p.JOINT_PRISMATIC:
            has_revolute_or_prismatic = True
            movable_names.append(joint_name)
            lower_limits.append(float(lower))
            upper_limits.append(float(upper))
            max_velocities.append(float(max_vel))
        elif joint_type == p.JOINT_CONTINUOUS:  # wheels
            has_continuous = True

        # EE detection: keyword match
        if ee_link is None:
            for kw in _EE_KEYWORDS:
                if kw in link_name.lower():
                    ee_link = link_name
                    break

    # Fallback: leaf link (no children) among movable links
    if ee_link is None and has_revolute_or_prismatic:
        for i in range(n_joints - 1, -1, -1):
            if i not in child_indices:
                info = p.getJointInfo(body_id, i, physicsClientId=physics_client)
                jtype = info[2]
                if jtype in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
                    ee_link = info[12].decode("utf-8")
                    break

    # Robot type heuristic
    if has_revolute_or_prismatic and not has_continuous:
        robot_type = "arm"
    elif has_continuous and not has_revolute_or_prismatic:
        robot_type = "mobile"
    elif has_continuous and has_revolute_or_prismatic:
        robot_type = "mobile_manipulator"
    else:
        robot_type = "unknown"

    # For mobile: no EE
    if robot_type == "mobile":
        ee_link = None

    return RobotProfile(
        urdf_path=urdf_path,
        robot_type=robot_type,
        ee_link_name=ee_link,
        movable_joint_names=movable_names,
        joint_lower_limits=lower_limits,
        joint_upper_limits=upper_limits,
        joint_max_velocities=max_velocities,
        num_joints=n_joints,
    )
```

**Step 4: Run tests**

Run: `pytest tests/test_robot_models.py -v`
Expected: ALL PASS

**Step 5: Commit**

```bash
git add pybullet_fleet/robot_models.py tests/test_robot_models.py
git commit -m "feat: add RobotProfile auto-detection from URDF"
```

---

## Task 3: Export and optional dependency [SERIAL — depends on Task 1]

**Files:**
- Modify: `pybullet_fleet/__init__.py`
- Modify: `pyproject.toml`

**Step 1: Add exports to `__init__.py`**

Add after the config utilities import block:

```python
# Robot model resolution
from pybullet_fleet.robot_models import resolve_urdf, list_available_models, RobotProfile, auto_detect_profile
```

Add to `__all__`:

```python
    # Robot models
    "resolve_urdf",
    "list_available_models",
    "RobotProfile",
    "auto_detect_profile",
```

**Step 2: Add optional dependency to `pyproject.toml`**

After the `docs` optional group:

```toml
models = [
    "robot_descriptions>=1.10.0",
]
```

**Step 3: Run full test suite**

Run: `pytest tests/ -x -q`
Expected: 866+ passed

**Step 4: Commit**

```bash
git add pybullet_fleet/__init__.py pyproject.toml
git commit -m "feat: export robot_models, add [models] optional dep"
```

---

## Task 4: Add `--robot` to arm demos [PARALLEL — independent of Task 5]

**Files:**
- Modify: `examples/arm/pick_drop_arm_demo.py`
- Modify: `examples/arm/pick_drop_arm_action_demo.py`
- Modify: `examples/arm/pick_drop_arm_ee_demo.py`
- Modify: `examples/arm/pick_drop_arm_ee_action_demo.py`

**Step 1: Add argparse + resolve_urdf to `pick_drop_arm_demo.py`**

Replace the URDF path block (lines ~13-33) with:

```python
import argparse

parser = argparse.ArgumentParser(description="Robot arm pick & drop demo")
parser.add_argument("--robot", default=None, help="Robot name (e.g. panda, kuka_iiwa) or URDF path")
args = parser.parse_args()

# ... after sim_core created ...

if args.robot:
    from pybullet_fleet.robot_models import resolve_urdf, auto_detect_profile
    arm_urdf = resolve_urdf(args.robot)
    profile = auto_detect_profile(arm_urdf, sim_core.client)
    print(f"Using {args.robot}: {profile.robot_type}, EE={profile.ee_link_name}, "
          f"joints={len(profile.movable_joint_names)}")
else:
    arm_urdf = os.path.join(os.path.dirname(__file__), "../robots/arm_robot.urdf")
```

Apply the same pattern to the other 3 arm demos. Each demo keeps its own default joint targets for the built-in arm; when `--robot` is used, the demo may need adapted joint targets (or skip if incompatible).

**Step 2: Test manually**

Run: `python examples/arm/pick_drop_arm_demo.py --robot panda`
Expected: Panda loads and runs pick & place (joint targets may not be tuned yet)

Run: `python examples/arm/pick_drop_arm_demo.py`
Expected: Old behavior unchanged

**Step 3: Commit**

```bash
git add examples/arm/pick_drop_arm_demo.py examples/arm/pick_drop_arm_action_demo.py \
        examples/arm/pick_drop_arm_ee_demo.py examples/arm/pick_drop_arm_ee_action_demo.py
git commit -m "feat: add --robot flag to arm demos"
```

---

## Task 5: Add `--robot` to mobile/fleet demos [PARALLEL — independent of Task 4]

**Files:**
- Modify: `examples/scale/100robots_grid_demo.py`
- Modify: `examples/scale/100robots_cube_patrol_demo.py`
- Modify: `examples/mobile/path_following_demo.py`

**Step 1: Add `--robot` to `100robots_grid_demo.py`**

The demo already has argparse. Add:

```python
parser.add_argument("--robot", default=None, help="Robot name (e.g. husky, racecar) or URDF path")
```

And in the URDF resolution section, replace:

```python
mobile_urdf = os.path.join(os.path.dirname(__file__), "..", mobile_urdf_path)
```

With:

```python
if args.robot:
    from pybullet_fleet.robot_models import resolve_urdf
    mobile_urdf = resolve_urdf(args.robot)
else:
    mobile_urdf = os.path.join(os.path.dirname(__file__), "..", mobile_urdf_path)
```

Apply similar pattern to the other fleet demos.

**Step 2: Test manually**

Run: `python examples/scale/100robots_grid_demo.py --robot husky --mode single`
Expected: 100 Huskys spawn and patrol

**Step 3: Commit**

```bash
git add examples/scale/100robots_grid_demo.py examples/scale/100robots_cube_patrol_demo.py \
        examples/mobile/path_following_demo.py
git commit -m "feat: add --robot flag to mobile/fleet demos"
```

---

## Task 6: Add `pybullet_data.getDataPath()` direct usage example [PARALLEL]

**Files:**
- Create: `examples/models/panda_pybullet_data_demo.py`

**Step 1: Write standalone demo using pybullet_data directly**

```python
#!/usr/bin/env python3
"""
panda_pybullet_data_demo.py
Demo: Using pybullet_data models directly (Panda arm + Table + Tray).

Shows how to use pybullet_data.getDataPath() for URDF resolution
without the resolve_urdf() helper.
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pybullet_data

from pybullet_fleet.agent import Agent, AgentSpawnParams, IKParams
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.geometry import Pose
from pybullet_fleet.sim_object import SimObject
from pybullet_fleet.types import MotionMode

DATA_PATH = pybullet_data.getDataPath()

params = SimulationParams(gui=True, timestep=0.1, physics=False, target_rtf=10)
sim = MultiRobotSimulationCore(params)

# Load Panda arm directly from pybullet_data
panda_urdf = os.path.join(DATA_PATH, "franka_panda/panda.urdf")
panda = Agent.from_urdf(
    urdf_path=panda_urdf,
    pose=Pose.from_xyz(0, 0, 0),
    use_fixed_base=True,
    sim_core=sim,
    ik_params=IKParams(),
)

# Load table from pybullet_data
table_urdf = os.path.join(DATA_PATH, "table/table.urdf")
table = SimObject.from_urdf(
    urdf_path=table_urdf,
    pose=Pose.from_xyz(0.5, 0, 0),
    sim_core=sim,
    is_static=True,
)

print(f"Panda joints: {panda.num_joints}")
print(f"Panda EE candidates: use auto_detect_profile() to find")

sim.run_simulation()
```

**Step 2: Test manually**

Run: `python examples/models/panda_pybullet_data_demo.py`
Expected: Panda and table render in GUI

**Step 3: Commit**

```bash
git add examples/models/panda_pybullet_data_demo.py
git commit -m "feat: add Panda pybullet_data direct usage demo"
```

---

## Task 7: Final verification [SERIAL — depends on all above]

**Step 1: Run full test suite**

Run: `make verify`
Expected: lint + 866+ tests pass

**Step 2: Run key demos**

```bash
python examples/arm/pick_drop_arm_demo.py --robot panda
python examples/scale/100robots_grid_demo.py --robot husky --mode single
python examples/models/panda_pybullet_data_demo.py
```

**Step 3: Final commit (if any cleanup needed)**

```bash
git add -A
git commit -m "chore: robot-models cleanup and verification"
```

---

## Task Dependencies

- **SERIAL**: Task 1 → Task 2 → Task 3 (core module build-up)
- **PARALLEL**: Task 4, Task 5, Task 6 (independent demo changes, all depend on Task 3)
- **SERIAL**: Task 7 (final verification, depends on all)
