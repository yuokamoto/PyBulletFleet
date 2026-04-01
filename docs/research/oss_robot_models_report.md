---
orphan: true
---

# Open-Source Robot URDF/Mesh Models for PyBullet Simulation

**Date:** 2026-03-28
**Purpose:** Survey of OSS robot description packages suitable for PyBulletFleet multi-robot simulation

---

```{contents}
:depth: 2
```

---

## 1. Robot Arms

### 1.1 Franka Emika Panda (Franka Robotics)

| Field | Value |
|---|---|
| **Manufacturer** | Franka Robotics (formerly Franka Emika) |
| **GitHub (ROS1)** | https://github.com/frankarobotics/franka_ros (`franka_description` subpackage) |
| **GitHub (ROS2)** | https://github.com/frankaemika/franka_ros2 |
| **PyBullet-ready** | **YES** — `pybullet_data` ships `franka_panda/` with pre-built URDF |
| **License** | Apache-2.0 ✅ |
| **DOF** | 7 (+2 finger gripper = 9 actuated) |
| **Mesh format** | DAE (visual), STL (collision) |
| **robot_descriptions.py** | `panda_description` (URDF), `panda_mj_description` (MJCF) |
| **Stars** | franka_ros: 464 ⭐ |
| **Notes** | Most popular research arm. PyBullet includes friction-anchor-enhanced gripper. Extensive community PyBullet examples exist. Very lightweight meshes. |

### 1.2 Universal Robots UR5e / UR5 / UR10e (Universal Robots)

| Field | Value |
|---|---|
| **Manufacturer** | Universal Robots |
| **GitHub (ROS1)** | https://github.com/ros-industrial/universal_robot (1.4k ⭐, Apache-2.0/BSD-3-Clause) |
| **GitHub (ROS2)** | https://github.com/UniversalRobots/Universal_Robots_ROS2_Description (299 ⭐, BSD-3-Clause) |
| **PyBullet-ready** | **YES** — `pybullet_ur5_robotiq` provides PyBullet-ready UR5 with Robotiq gripper |
| **License** | URDF/code: BSD-3-Clause / Apache-2.0 ✅. **⚠️ UR20/UR30/UR15/UR18/UR8Long meshes** have UR-specific graphical doc license (usable but not fully OSI-compliant) |
| **DOF** | 6 (all UR models) |
| **Mesh format** | STL (collision), DAE (visual) |
| **robot_descriptions.py** | `ur5e_description`, `ur10e_description`, `ur3e_description`, `ur5_description`, etc. |
| **Stars** | ros-industrial: 1.4k ⭐ |
| **Notes** | Industry standard collaborative arms. UR3/UR5/UR10 and e-series variants. Classic UR3/5/10 meshes are fully BSD-3-Clause. Newer UR15/18/20/30 meshes require UR graphical license. For PyBulletFleet, stick to UR5e/UR10e (fully free meshes). Kuka iiwa URDF also in pybullet_data. |
| **PyBullet Community** | https://github.com/ElectronicElephant/pybullet_ur5_robotiq (299 ⭐, BSD-2-Clause) — UR5 + Robotiq 85/140 with IK, gym-styled API |

### 1.3 KUKA iiwa 14 / LBR iiwa (KUKA)

| Field | Value |
|---|---|
| **Manufacturer** | KUKA |
| **GitHub (ROS)** | https://github.com/ros-industrial/kuka_experimental (331 ⭐, Apache-2.0) |
| **PyBullet-ready** | **YES** — `pybullet_data` ships `kuka_iiwa/` with ready-to-use URDF and SDF |
| **License** | Apache-2.0 ✅ |
| **DOF** | 7 |
| **Mesh format** | STL, OBJ |
| **robot_descriptions.py** | `iiwa14_description` (URDF, BSD-3-Clause), `iiwa14_mj_description` (MJCF) |
| **Notes** | Extremely common in PyBullet examples and tutorials. Erwin Coumans used it extensively in official PyBullet demos. Lightweight meshes. Also includes `kuka_lwr/` in bullet3/data. Multiple KUKA models available (KR3, KR5, KR6, KR10, KR16, KR120, KR150, KR210) in kuka_experimental. |

### 1.4 Kinova Gen3 / Gen3 Lite / Jaco2 (Kinova Robotics)

| Field | Value |
|---|---|
| **Manufacturer** | Kinova Robotics |
| **GitHub (ROS2)** | https://github.com/Kinovarobotics/ros2_kortex (112 ⭐, `kortex_description` subpackage) |
| **PyBullet-ready** | Partial — URDF available, community conversions exist |
| **License** | BSD-3-Clause ✅ (via ros2_kortex) |
| **DOF** | Gen3: 6 or 7 DOF; Gen3 Lite: 6 DOF |
| **Mesh format** | STL (collision + visual) |
| **robot_descriptions.py** | `gen3_description`, `gen3_lite_description`, `j2n6s300_description`, etc. (BSD-3-Clause) |
| **Notes** | Gen3 with Robotiq 2F-85 / 2F-140 gripper options. Active development (2026). Includes MoveIt2 configs. MuJoCo Menagerie also has `kinova_gen3/` (BSD-3-Clause). |

### 1.5 ROBOTIS OpenMANIPULATOR-X (ROBOTIS)

| Field | Value |
|---|---|
| **Manufacturer** | ROBOTIS |
| **GitHub** | https://github.com/ROBOTIS-GIT/open_manipulator (606 ⭐) |
| **License** | Apache-2.0 ✅ |
| **DOF** | 4 (+1 gripper) |
| **Mesh format** | STL |
| **robot_descriptions.py** | `open_manipulator_x_description` (Apache-2.0) |
| **Notes** | Lightweight, education-focused. Dynamixel servos. Very low mesh complexity — excellent for 100-robot scenarios. Multiple new variants: OMX-F, OMX-L, OMY-3M, OMY-F3M (all Apache-2.0). Active development (released 4.1.3 last week as of writing). |

---

## 2. Mobile Robots

### 2.1 TurtleBot3 (ROBOTIS)

| Field | Value |
|---|---|
| **Manufacturer** | ROBOTIS |
| **GitHub** | https://github.com/ROBOTIS-GIT/turtlebot3 (1.9k ⭐) — includes `turtlebot3_description` |
| **License** | Apache-2.0 ✅ |
| **Type** | Differential drive, 2-wheeled |
| **Variants** | Burger, Waffle, Waffle Pi |
| **Mesh format** | STL |
| **PyBullet-ready** | Community conversions exist; simple URDF |
| **robot_descriptions.py** | Not currently listed (but URDF is straightforward) |
| **Notes** | THE standard educational mobile robot. Extremely simple geometry — ideal for fleet simulation. Very small mesh files. Burger variant is especially minimal. Active development (v2.3.6 Dec 2025). |

### 2.2 Clearpath Husky / Jackal (Clearpath Robotics)

| Field | Value |
|---|---|
| **Manufacturer** | Clearpath Robotics |
| **GitHub (ROS2)** | https://github.com/clearpathrobotics/clearpath_common (52 ⭐, `clearpath_platform_description`) |
| **GitHub (Legacy)** | `husky_description`, `jackal_description` in separate repos |
| **PyBullet-ready** | **YES** — `pybullet_data` ships `husky/` with ready-to-use URDF |
| **License** | BSD-3-Clause ✅ |
| **Type** | Husky: 4-wheel skid-steer; Jackal: 4-wheel differential drive |
| **Mesh format** | STL, DAE |
| **Notes** | Husky is the most popular outdoor UGV platform in research. PyBullet includes it natively. Clearpath now has unified `clearpath_common` repo supporting Husky, Jackal, Dingo, Boxer, and more. Includes support for mounting manipulators (Kinova Gen3, UR arms). Moderate mesh complexity. |

### 2.3 Racecar / MIT Racecar (PyBullet built-in)

| Field | Value |
|---|---|
| **Source** | PyBullet built-in (`pybullet_data`) |
| **GitHub** | https://github.com/bulletphysics/bullet3 (`data/racecar/`) |
| **PyBullet-ready** | **YES** — native PyBullet model |
| **License** | Zlib (bullet3 license) ✅ |
| **Type** | Ackermann steering 4-wheeled vehicle |
| **Mesh format** | OBJ, STL |
| **Notes** | Used in many PyBullet RL tutorials (pybullet_envs). Very lightweight. Good for testing vehicle dynamics. Includes textured meshes. |

---

## 3. Mobile Manipulators

### 3.1 Fetch Robot (Fetch Robotics)

| Field | Value |
|---|---|
| **Manufacturer** | Fetch Robotics (now Zebra Technologies) |
| **GitHub** | https://github.com/fetchrobotics/fetch_ros (`fetch_description` subpackage) — repo may be archived |
| **PyBullet-ready** | Community URDFs exist; not in pybullet_data |
| **License** | MIT ✅ (via `fetch_description`) |
| **Type** | Differential-drive base + 7-DOF arm + gripper |
| **Mesh format** | STL, DAE |
| **robot_descriptions.py** | `fetch_description` (MIT) ✅ |
| **Notes** | Classic mobile manipulator for pick-and-place research. Available in `robot_descriptions.py`. ROS1 only (melodic/noetic). No longer actively maintained but models are stable and widely used. |

### 3.2 PAL TIAGo (PAL Robotics)

| Field | Value |
|---|---|
| **Manufacturer** | PAL Robotics |
| **PyBullet-ready** | Not directly, but URDF available via robot_descriptions.py |
| **License** | Apache-2.0 ✅ |
| **Type** | Differential-drive base + 7-DOF arm + parallel gripper |
| **robot_descriptions.py** | `tiago_description` (Apache-2.0) |
| **MuJoCo Menagerie** | `pal_tiago/` and `pal_tiago_dual/` (Apache-2.0, 12 DOF / 21 DOF) |
| **Notes** | Active commercial/research platform. Available in both single-arm (TIAGo) and dual-arm (TIAGo++) variants. Good mesh quality. Moderate complexity. |

### 3.3 TurtleBot3 + OpenMANIPULATOR (ROBOTIS)

| Field | Value |
|---|---|
| **Manufacturer** | ROBOTIS |
| **GitHub** | https://github.com/ROBOTIS-GIT/turtlebot3_manipulation |
| **License** | Apache-2.0 ✅ |
| **Type** | Differential-drive base (TB3 Waffle) + 4-DOF arm |
| **Mesh format** | STL |
| **Notes** | Lightest-weight mobile manipulator option. Combine TurtleBot3 base with OpenMANIPULATOR-X arm. Very low mesh complexity — ideal for 100+ robot fleet scenarios. Actively maintained. |

---

## 4. Aggregator Repositories

### 4.1 `robot_descriptions.py` ⭐⭐⭐ (BEST AGGREGATOR)

| Field | Value |
|---|---|
| **GitHub** | https://github.com/robot-descriptions/robot_descriptions.py (726 ⭐) |
| **License** | Apache-2.0 ✅ |
| **Models** | **175+ robot descriptions** across all categories |
| **PyBullet Loader** | **YES** — `robot_descriptions.loaders.pybullet` built-in |
| **Install** | `pip install robot_descriptions` |
| **Features** | Auto-downloads and caches models. Supports URDF, MJCF, Xacro. Organized by category (Arms, Bipeds, Dual Arms, Drones, Educational, End Effectors, Humanoids, Mobile Manipulators, Quadrupeds, Wheeled). |

**Key robot descriptions available via this package:**

| Category | Available Models |
|---|---|
| **Arms** | Panda, FR3, UR3/5/10(e), KUKA iiwa7/14, Kinova Gen2/Gen3, xArm6/7, Z1, OpenMANIPULATOR-X, Sawyer, Lite6, YAM, SO-ARM100 |
| **Mobile Manipulators** | Fetch, TIAGo, PR2, Pepper, Stretch RE1/SE3/3, Eve R3, Reachy, BamBot |
| **Quadrupeds** | A1, Aliengo, ANYmal B/C/D, B1, B2, Go1, Go2, HyQ, Laikago, Mini Cheetah, Minitaur, Solo, Spot |
| **Humanoids** | Atlas, G1, H1, TALOS, Valkyrie, and many more |
| **End Effectors** | Robotiq 2F-85, Allegro Hand, Shadow Hand, LEAP Hand |

### 4.2 MuJoCo Menagerie (Google DeepMind)

| Field | Value |
|---|---|
| **GitHub** | https://github.com/google-deepmind/mujoco_menagerie (3.2k ⭐) |
| **License** | Apache-2.0 (repo), per-model licenses (mostly BSD-3/Apache-2.0) ✅ |
| **Models** | 60+ high-quality MJCF models |
| **Format** | MJCF (MuJoCo XML), OBJ/STL meshes |
| **PyBullet-ready** | No (MJCF format, would need conversion to URDF) |
| **Notes** | Highest quality simulation models available. OBJ meshes could potentially be reused with custom URDFs. Excellent reference for mass/inertia parameters. Includes arms, mobile manipulators, humanoids, quadrupeds. Active development (last commit: last week). |

### 4.3 PyBullet Built-in (`pybullet_data`)

| Field | Value |
|---|---|
| **GitHub** | https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet/gym/pybullet_data |
| **License** | Zlib (bullet3), per-model licenses vary |
| **Install** | Comes with `pip install pybullet` |
| **Format** | URDF, SDF, OBJ, STL, DAE |
| **PyBullet-ready** | **YES** — all models are natively PyBullet-ready |

**Key models in pybullet_data:**

| Model | Type | Notes |
|---|---|---|
| `franka_panda/` | 7-DOF arm | With friction anchor gripper |
| `kuka_iiwa/` | 7-DOF arm | Multiple variants, SDF also available |
| `kuka_lwr/` | 7-DOF arm | Older KUKA model |
| `husky/` | Mobile robot | Clearpath Husky UGV |
| `racecar/` | Vehicle | Ackermann steering |
| `a1/` | Quadruped | Unitree A1 |
| `aliengo/` | Quadruped | Unitree Aliengo |
| `laikago/` | Quadruped | Unitree Laikago (simplified variant available) |
| `mini_cheetah/` | Quadruped | MIT Mini Cheetah |
| `differential/` | Mobile robot | Simple diff-drive |
| `xarm/` | Arm | UFACTORY xArm |
| `widowx/` | Arm | Trossen WidowX |
| `r2d2.urdf` | Mobile | Toy model, very lightweight |
| `plane.urdf` | Ground | Standard ground plane |
| `table/` | Object | Table for manipulation tasks |
| `tray/` | Object | Tray for grasping |
| `random_urdfs/` | Various | Procedurally generated shapes |

### 4.4 Unitree Robotics

| Field | Value |
|---|---|
| **GitHub** | https://github.com/unitreerobotics/unitree_ros (1.3k ⭐) |
| **License** | BSD-3-Clause ✅ |
| **Models** | A1, A2, Aliengo, B1, B2, B2W, G1, Go1, Go2, Go2W, H1, H1_2, H2, Laikago, R1, R1 Air, Z1, AS2, Dexterous Hand |
| **Format** | URDF + xacro, STL/DAE meshes |
| **Notes** | Most comprehensive single-manufacturer collection. Actively maintained (last commit: 19 hours ago!). Includes quadrupeds, humanoids, and dexterous hands. URDF files work in PyBullet with minor adjustments. |

### 4.5 ROS-Industrial

| Field | Value |
|---|---|
| **GitHub Org** | https://github.com/ros-industrial |
| **Key Repos** | `universal_robot` (1.4k ⭐), `kuka_experimental` (331 ⭐), `robotiq`, `abb`, `fanuc`, `motoman` |
| **License** | Apache-2.0 / BSD-3-Clause ✅ |
| **Notes** | Industrial robot focus. URDF descriptions for many manufacturer-specific robots. Primarily ROS1 but URDFs are reusable. |

---

## 5. PyBullet Built-in Models Summary

Models accessible via `import pybullet_data; pybullet_data.getDataPath()`:

```
pybullet_data/
├── franka_panda/       # Franka Panda arm (7-DOF + gripper)
├── kuka_iiwa/          # KUKA iiwa14 (7-DOF)
├── kuka_lwr/           # KUKA LWR
├── husky/              # Clearpath Husky mobile robot
├── racecar/            # MIT Racecar
├── a1/                 # Unitree A1 quadruped
├── aliengo/            # Unitree Aliengo quadruped
├── laikago/            # Unitree Laikago (with simplified variant)
├── mini_cheetah/       # MIT Mini Cheetah
├── differential/       # Simple diff-drive robot
├── xarm/               # UFACTORY xArm
├── widowx/             # Trossen WidowX arm
├── quadruped/          # Generic quadruped
├── gripper/            # Various grippers
├── humanoid/           # MuJoCo-derived humanoid
├── plane.urdf          # Ground plane
├── plane100.urdf       # Large ground plane
├── table/              # Table
├── tray/               # Tray for grasping
├── cube.urdf           # Simple cube
├── sphere*.urdf        # Various spheres
├── r2d2.urdf           # R2D2 (educational)
└── random_urdfs/       # Procedural shapes (zipped)
```

---

## 6. Recommendations for PyBulletFleet

### Tier 1: Immediately Usable (PyBullet-native, permissive license)

These have PyBullet-ready URDFs in `pybullet_data` or simple, proven URDFs:

| Robot | Category | License | Why |
|---|---|---|---|
| **Franka Panda** | Arm | Apache-2.0 | In pybullet_data. Most tested arm in PyBullet. 7-DOF + gripper. |
| **KUKA iiwa14** | Arm | BSD-3-Clause | In pybullet_data. Erwin Coumans' go-to demo arm. |
| **Husky** | Mobile | BSD-3-Clause | In pybullet_data. Standard outdoor UGV. |
| **Racecar** | Mobile | Zlib | In pybullet_data. Lightweight Ackermann vehicle. |
| **UR5 + Robotiq** | Arm + gripper | BSD-2-Clause | `pybullet_ur5_robotiq` repo. Gym-styled, IK-ready. |

### Tier 2: Easy Integration (small effort to convert)

| Robot | Category | License | Why |
|---|---|---|---|
| **TurtleBot3 Burger** | Mobile | Apache-2.0 | Extremely lightweight meshes. Perfect for 100-robot scenarios. |
| **OpenMANIPULATOR-X** | Arm | Apache-2.0 | 4-DOF + gripper. Very low mesh complexity. Education-focused. |
| **Kinova Gen3** | Arm | BSD-3-Clause | 6/7-DOF. Clean URDF. Available via robot_descriptions.py. |
| **Fetch** | Mobile Manip | MIT | Classic research platform. Available via robot_descriptions.py. |
| **TIAGo** | Mobile Manip | Apache-2.0 | Available in MuJoCo Menagerie + robot_descriptions.py. |

### Tier 3: Use via `robot_descriptions.py`

**Recommended approach:** Install `robot_descriptions` and use its PyBullet loader to access 175+ models:

```python
# pip install robot_descriptions
from robot_descriptions.loaders.pybullet import load_robot_description

# Load any supported robot directly into PyBullet
robot_id = load_robot_description("panda_description")
robot_id = load_robot_description("ur5e_description")
robot_id = load_robot_description("fetch_description")
robot_id = load_robot_description("gen3_description")
```

### Fleet Simulation Considerations

For 100+ robot scenarios in PyBulletFleet, prioritize:

1. **Mesh complexity** — Simpler meshes = faster rendering and collision checking
   - Best: TurtleBot3 Burger (minimal STL), simple cube robots
   - Good: OpenMANIPULATOR-X, KUKA iiwa (moderate STL)
   - Heavier: Franka Panda (DAE visual meshes), Fetch (detailed STL)

2. **Collision mesh simplification** — Use collision-specific meshes (most packages provide separate visual/collision meshes where collision is simplified)

3. **Kinematic-only mode** — PyBulletFleet's teleport mode means physics fidelity of inertia/mass matters less; focus on visual representation and URDF joint structure

4. **License compatibility** — All Tier 1 and Tier 2 recommendations are Apache-2.0 compatible

### Quick Integration Path

For adding a new robot to PyBulletFleet:

```python
# Option A: Use pybullet_data built-in
import pybullet_data
urdf_path = f"{pybullet_data.getDataPath()}/franka_panda/panda.urdf"

# Option B: Use robot_descriptions.py
from robot_descriptions import panda_description
urdf_path = panda_description.URDF_PATH

# Option C: Bundle URDF+meshes directly in PyBulletFleet's robots/ directory
urdf_path = "robots/my_robot.urdf"
```

---

## License Compatibility Matrix

| License | Compatible with Apache-2.0? | Notes |
|---|---|---|
| Apache-2.0 | ✅ Yes | Same license |
| BSD-3-Clause | ✅ Yes | More permissive |
| BSD-2-Clause | ✅ Yes | More permissive |
| MIT | ✅ Yes | More permissive |
| Zlib | ✅ Yes | More permissive |
| UR Graphical Doc License | ⚠️ Partial | Allows use/modify/share with restrictions. UR20/30/15/18 meshes only |
| GPL-3.0 | ❌ No | Copyleft, incompatible |
| CC-BY-SA-4.0 | ❌ No | Share-alike, incompatible |
| NASA-1.3 | ⚠️ Check | Government license, special terms |

---

## Summary Table

| # | Robot | Manufacturer | Category | DOF | License | PyBullet-Ready | Mesh Weight | Source |
|---|---|---|---|---|---|---|---|---|
| 1 | Franka Panda | Franka Robotics | Arm | 7+2 | Apache-2.0 | ✅ Native | Medium | pybullet_data |
| 2 | KUKA iiwa 14 | KUKA | Arm | 7 | BSD-3-Clause | ✅ Native | Low | pybullet_data |
| 3 | UR5e | Universal Robots | Arm | 6 | BSD-3-Clause | ✅ Community | Medium | ros-industrial / UR official |
| 4 | Kinova Gen3 | Kinova | Arm | 6/7 | BSD-3-Clause | Partial | Medium | ros2_kortex |
| 5 | OpenMANIPULATOR-X | ROBOTIS | Arm | 4+1 | Apache-2.0 | Easy convert | **Very Low** | ROBOTIS-GIT |
| 6 | TurtleBot3 | ROBOTIS | Mobile | 2 | Apache-2.0 | Easy convert | **Very Low** | ROBOTIS-GIT |
| 7 | Husky | Clearpath | Mobile | 4 | BSD-3-Clause | ✅ Native | Medium | pybullet_data |
| 8 | Racecar | MIT/PyBullet | Mobile | 4 | Zlib | ✅ Native | Low | pybullet_data |
| 9 | Fetch | Fetch Robotics | Mobile Manip | 7+2+2 | MIT | robot_descriptions | Medium-High | robot_descriptions.py |
| 10 | TIAGo | PAL Robotics | Mobile Manip | 12 | Apache-2.0 | robot_descriptions | Medium | robot_descriptions.py |
| 11 | TB3 + OpenManip | ROBOTIS | Mobile Manip | 2+4+1 | Apache-2.0 | Easy convert | **Very Low** | ROBOTIS-GIT |
