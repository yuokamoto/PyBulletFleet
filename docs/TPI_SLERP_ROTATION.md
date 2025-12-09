# TPI + Slerp Rotation Implementation

## Overview

This document explains how we combine Two-Point Interpolation (TPI) with Spherical Linear Interpolation (Slerp) to achieve smooth rotation with angular acceleration and deceleration constraints for differential drive robots.

## Problem

The original implementation used Slerp with a simple time-based interpolation:
```python
rotation_duration = rotation_angle / max_angular_vel
key_times = [start_time, end_time]
rotation_slerp = Slerp(key_times, key_rots)
```

**Limitations:**
- No acceleration/deceleration phase
- Instant jump to max angular velocity
- No respect for `max_angular_accel` parameter
- Unrealistic motion profile

## Solution

We use TPI to control the **angle progression** (0 to total_angle) with acceleration constraints, then use Slerp to perform the **quaternion interpolation** based on the angle ratio.

### Architecture

```
TPI: Controls angle progression
  Input: 0 → total_rotation_angle
  Output: current_angle, angular_velocity, angular_acceleration
  Constraints: max_angular_vel, max_angular_accel

Slerp: Performs quaternion interpolation
  Input: angle_ratio (0.0 to 1.0)
  Output: interpolated quaternion
  Provides: Shortest-path quaternion interpolation
```

### Key Components

#### 1. Initialize TPI for Rotation Angle

```python
# Calculate total rotation angle
r_current = R.from_quat(start_quat)
r_target = R.from_quat(target_quat)
r_delta = r_target * r_current.inv()
rotation_angle = r_delta.magnitude()  # Total angle in radians

# Initialize TPI for angle progression
self._tpi_rotation_angle = TwoPointInterpolation()
self._tpi_rotation_angle.init(
    p0=0.0,                      # Start at 0 angle
    pe=rotation_angle,           # End at total rotation angle
    acc_max=angular_accel,       # Angular acceleration constraint
    vmax=angular_vel,            # Angular velocity constraint
    t0=start_time,
    v0=0.0,                      # Start from rest
    ve=0.0,                      # Stop at target
    dec_max=angular_accel,
)
self._tpi_rotation_angle.calc_trajectory()
```

#### 2. Initialize Slerp with Normalized Range

```python
# Create Slerp interpolator with normalized time [0, 1]
key_rots = R.from_quat([start_quat, target_quat])
self._rotation_slerp = Slerp([0.0, 1.0], key_rots)
```

#### 3. Update Loop

```python
# Get current angle from TPI (with acceleration/deceleration)
current_angle, angular_vel, angular_accel = self._tpi_rotation_angle.get_point(current_time)

# Calculate angle ratio (0.0 to 1.0)
angle_ratio = current_angle / total_rotation_angle
angle_ratio = np.clip(angle_ratio, 0.0, 1.0)

# Query Slerp with angle ratio
r_interpolated = self._rotation_slerp(angle_ratio)
new_orientation = r_interpolated.as_quat()

# Update robot orientation
p.resetBasePositionAndOrientation(body_id, position, new_orientation)

# Store angular velocity from TPI
self._current_angular_velocity = angular_vel
```

## Benefits

### 1. **Smooth Acceleration/Deceleration**
- TPI automatically calculates acceleration and deceleration phases
- Respects `max_angular_accel` constraint
- Smooth velocity profile: 0 → max_vel → 0

### 2. **Velocity Constraints**
- Respects `max_angular_vel` constraint
- Never exceeds maximum angular velocity
- Realistic motion profile

### 3. **Accurate Angular Velocity**
- TPI provides instantaneous angular velocity at each time step
- Can be used for physics simulation or visualization
- No need for approximation from time derivatives

### 4. **Shortest Path Rotation**
- Slerp ensures shortest path quaternion interpolation
- No gimbal lock issues
- Smooth quaternion transitions

### 5. **Flexible Parameters**
- Independent control of `max_angular_vel` and `max_angular_accel`
- Can adjust acceleration for different robot types
- Easy to tune for different motion characteristics

## Motion Profile Example

For a 90° rotation with:
- `max_angular_vel = 1.0 rad/s`
- `max_angular_accel = 10.0 rad/s²`

```
Angle (rad)
1.57 |                    ________
     |                  /          \
     |                /              \
     |              /                  \
0.0  |____________/                      \___________
     0          0.1                1.47  1.57  (time)

Angular Velocity (rad/s)
1.0  |            __________________
     |          /                    \
     |        /                        \
0.0  |______/                            \______
     0    0.1                      1.47  1.57

     Accel    Cruise      Decel
     Phase    Phase       Phase
```

## Comparison with Previous Implementation

### Before (Simple Slerp)
```python
rotation_duration = angle / max_vel
slerp = Slerp([t0, t0 + duration], rotations)
# Instant acceleration, constant velocity, instant stop
```

**Issues:**
- No acceleration phase
- Unrealistic motion
- Ignores max_angular_accel

### After (TPI + Slerp)
```python
tpi.init(0, angle, max_accel, max_vel, ...)
angle_ratio = tpi.get_point(t) / total_angle
quat = slerp(angle_ratio)
# Smooth acceleration, cruise, deceleration
```

**Advantages:**
- Smooth acceleration/deceleration
- Respects both velocity and acceleration constraints
- Realistic motion profile
- Accurate velocity tracking

## Code Structure

### Agent Class Members

```python
# Rotation state
self._tpi_rotation_angle: Optional[TwoPointInterpolation] = None
self._rotation_slerp: Optional[Slerp] = None
self._rotation_total_angle: float = 0.0
self._rotation_start_quat: Optional[np.ndarray] = None
self._rotation_target_quat: Optional[np.ndarray] = None
```

### Key Methods

1. **`_init_differential_rotation_trajectory(goal: Pose)`**
   - Calculates target orientation
   - Computes rotation angle
   - Initializes TPI and Slerp

2. **`_update_differential(dt: float)`**
   - Queries TPI for current angle
   - Calculates angle ratio
   - Queries Slerp for quaternion
   - Updates robot orientation

## Performance Considerations

### Computational Cost
- **TPI calculation**: O(1) per frame - simple polynomial evaluation
- **Slerp interpolation**: O(1) per frame - quaternion operations
- **Total overhead**: Minimal (microseconds per frame)

### Memory Overhead
- TPI instance: ~200 bytes
- Slerp instance: ~100 bytes
- State variables: ~64 bytes
- **Total**: < 500 bytes per robot

### Accuracy
- TPI: Numerical precision limited by floating point (double)
- Slerp: Mathematically exact quaternion interpolation
- Combined accuracy: Excellent for all practical applications

## Usage Example

```python
# Create robot with angular constraints
robot = Agent.from_urdf(
    urdf_path="mobile_robot.urdf",
    max_angular_vel=1.0,      # 1 rad/s max velocity
    max_angular_accel=10.0,   # 10 rad/s² max acceleration
    motion_mode=MotionMode.DIFFERENTIAL
)

# Set path - rotation will use TPI + Slerp automatically
path = Path.create_square(center=[0, 0, 0], side_length=2.0)
robot.set_path(path.waypoints)

# In simulation loop
while True:
    robot.update(dt=0.01)

    # Angular velocity is available
    angular_vel = robot.angular_velocity  # From TPI
```

## Future Enhancements

### Potential Improvements
1. **Per-axis angular acceleration**: Use different constraints for yaw/pitch/roll
2. **Jerk constraints**: Add maximum angular jerk for even smoother motion
3. **Time-optimal trajectories**: Calculate minimum-time trajectory
4. **Velocity continuation**: Support non-zero initial/final angular velocities

### Advanced Applications
1. **Aerial robots**: Use full 3-axis rotation control
2. **Robotic arms**: Apply to joint rotations
3. **Camera control**: Smooth gimbal motion
4. **Animation**: Natural character rotation

## References

1. Two-Point Interpolation Library: `external/two_points_interpolation_py/`
2. Scipy Slerp Documentation: https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Slerp.html
3. Quaternion Interpolation: Shoemake, K. (1985). "Animating rotation with quaternion curves"

## Summary

The combination of TPI and Slerp provides:
- ✅ Smooth acceleration/deceleration
- ✅ Respects angular velocity constraints
- ✅ Respects angular acceleration constraints
- ✅ Shortest-path quaternion interpolation
- ✅ Accurate angular velocity tracking
- ✅ Minimal computational overhead
- ✅ Easy to tune and configure

This approach gives realistic, smooth robot motion while maintaining full control over velocity and acceleration constraints.
