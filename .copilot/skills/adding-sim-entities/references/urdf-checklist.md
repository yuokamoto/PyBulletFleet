# URDF Checklist

Requirements and patterns for URDF files used in PyBulletFleet.

## Table of Contents

1. [Minimum Requirements](#minimum-requirements)
2. [Structure Templates](#structure-templates)
3. [Collision Geometry Guidelines](#collision-geometry-guidelines)
4. [Joint Configuration](#joint-configuration)
5. [Common Issues](#common-issues)

## Minimum Requirements

Every URDF loaded by PyBulletFleet must have:

- [ ] At least one `<link>` with `<visual>` geometry
- [ ] At least one `<link>` with `<collision>` geometry (for collision detection)
- [ ] All links referenced by joints must exist
- [ ] Reasonable dimensions in meters (SI units)

## Structure Templates

### Simple box robot (like simple_cube.urdf)

```xml
<?xml version="1.0"?>
<robot name="simple_box">
  <link name="base_link">
    <visual>
      <geometry><box size="0.5 0.5 0.3"/></geometry>
      <material name="blue"><color rgba="0.2 0.4 0.8 1.0"/></material>
    </visual>
    <collision>
      <geometry><box size="0.5 0.5 0.3"/></geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>
</robot>
```

### Articulated arm (like arm_robot.urdf)

```xml
<?xml version="1.0"?>
<robot name="arm">
  <link name="base_link">
    <!-- base with visual + collision + inertial -->
  </link>

  <link name="link_1">
    <!-- visual + collision + inertial -->
  </link>

  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100" velocity="1.0"/>
  </joint>

  <!-- More links and joints... -->
</robot>
```

### Mobile base with wheels

```xml
<joint name="left_wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="left_wheel"/>
  <origin xyz="0 0.2 0"/>
  <axis xyz="0 1 0"/>
</joint>
```

## Collision Geometry Guidelines

- **Keep collision geometry simpler than visual** — use boxes/cylinders instead of mesh
- **Collision geometry should be slightly larger** than visual for margin
- **Matching collision = visual is fine** for simple shapes
- **Mesh collision is expensive** — use only when box/cylinder approximation is insufficient

```xml
<!-- Good: simple collision with detailed visual -->
<visual>
  <geometry><mesh filename="detailed_model.obj" scale="1 1 1"/></geometry>
</visual>
<collision>
  <geometry><box size="0.5 0.5 0.3"/></geometry>  <!-- bounding box -->
</collision>
```

## Joint Configuration

| Joint Type | Use For |
|-----------|---------|
| `revolute` | Arm joints with limits |
| `continuous` | Wheels (no limits) |
| `prismatic` | Linear slides, elevators |
| `fixed` | Rigidly attached parts |

**Always set `<limit>`** for revolute and prismatic joints:
```xml
<limit lower="-1.57" upper="1.57" effort="100" velocity="2.0"/>
```

**`use_fixed_base`** in AgentSpawnParams:
- `True` → base link fixed to world (arms, stationary machines)
- `False` → base link can move (mobile robots)

## Common Issues

| Issue | Symptom | Fix |
|-------|---------|-----|
| No `<collision>` | Object exists but no collision detected | Add `<collision>` geometry |
| Scale wrong | Robot tiny or giant | URDF uses meters — check dimensions |
| Missing `<inertial>` | Warning with physics=true; behaves strangely | Add mass and inertia |
| Joint axis wrong | Joint rotates unexpected direction | Check `<axis xyz="">` — 0 0 1 = Z-axis |
| Mesh path not found | URDF load fails | Use path relative to URDF file or absolute |
