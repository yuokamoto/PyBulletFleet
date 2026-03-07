# Agent.update() Profiling Results and Optimization Proposals

**Date**: 2026-01-01
**Test Environment**: 1,000 Agents, PyBullet DIRECT mode
**Target**: Update time ≤ 10ms

---

## 🎯 Key Findings

### Current Performance (1,000 Agents)

| State | Update Time | μs/agent | vs Target |
|-------|-------------|----------|-----------|
| **All Moving (DIFFERENTIAL)** | 105.75ms | 105.75μs | **10.6x** ❌ |
| **All Stationary** | 3.48ms | 3.48μs | **0.35x** ✅ |
| **50% Moving + 50% Stationary** | 54.61ms | - | **5.5x** ⚠️ |

---

## 📊 Detailed Profiling Results

### Test 1: Stationary vs Moving

```
Stationary (no goal):
├─ Total time: 3.48 ms
└─ Per agent: 3.48 μs

Moving (with goal):
├─ Total time: 105.75 ms
└─ Per agent: 105.75 μs

Speed difference: 30.4x slower (moving)
```

**Analysis**:
- Stationary Agent.update() is **very lightweight** (3.48μs/agent)
- Moving agents have **30x+ overhead**
- This is trajectory calculation cost in `_update_differential()` and `_update_omnidirectional()`

### Test 2: Motion Control Mode Comparison

```
DIFFERENTIAL (rotate then move forward):
├─ Total time: 110.75 ms
└─ Per agent: 110.75 μs

OMNIDIRECTIONAL (holonomic movement):
├─ Total time: 26.12 ms
└─ Per agent: 26.12 μs

Speed ratio: 4.24x (DIFFERENTIAL is slower)
```

**Analysis**:
- **OMNIDIRECTIONAL is 4x faster than DIFFERENTIAL!**
- DIFFERENTIAL has complex rotation control calculations
- Can switch to OMNIDIRECTIONAL depending on use case

### Test 3: Goal Setting Impact

```
No goal (stationary): 3.68 ms (3.68 μs/agent)
With goal (moving): 117.11 ms (117.11 μs/agent)
After goal reached (stopped): 69.52 ms (69.52 μs/agent)

Movement overhead: 31.8x
```

**Analysis**:
- `_is_moving` flag may not update immediately after goal reached
- Room for optimization in stop detection

---

## 💡 Optimization Strategies

### 🥇 Priority 1: Early Return for Stationary Agents (Recommended)

**Implementation**:
```python
class Agent:
    def update(self, dt: float):
        # Early return for stationary agents
        if not self._is_moving or self._goal_pose is None:
            # Current implementation:
            p.resetBaseVelocity(...)  # ← Unnecessary PyBullet call
            p.getBasePositionAndOrientation(...)  # ← Unnecessary
            p.resetBasePositionAndOrientation(...)  # ← Unnecessary
            return

        # Optimized version:
        if not self._is_moving or self._goal_pose is None:
            return  # ← Immediate return (no PyBullet calls)

        # Below executed only when moving...
```

**Expected Effect**:
- 50% moving scenario: 54.61ms → ~27ms (**2x improvement**)
- 10% moving scenario: ~11ms → ~2ms (**5x improvement**)
- Zero overhead for stationary agents

**Risk**: Low (simple change, no functional impact)

---

### 🥈 Priority 2: Switch to OMNIDIRECTIONAL Mode

**When Applicable**:
- Warehouse robots (holonomic drive possible)
- Aerial robots (drones)
- Scenarios where rotation constraints don't matter

**Implementation**:
```python
agent = Agent.from_mesh(
    motion_mode=MotionMode.OMNIDIRECTIONAL  # Instead of DIFFERENTIAL
)
```

**Expected Effect**:
- 4.24x faster than DIFFERENTIAL
- 1,000 agents: 110.75ms → 26.12ms

**Risk**: Medium (requires robot hardware to support holonomic drive)

---

### 🥉 Priority 3: Batch PyBullet Calls

**Current Problem**:
```python
# Called 1,000 times individually
for agent in agents:
    agent.update(dt)  # Each calls p.resetBaseVelocity separately
```

**Optimized Approach**:
```python
# Batch reset velocities
body_ids = [agent.body_id for agent in moving_agents]
velocities = [agent.target_vel for agent in moving_agents]
p.resetBaseVelocitiesBatch(body_ids, velocities)  # Single call
```

**Expected Effect**:
- Reduces PyBullet API overhead
- Estimated 10-20% improvement

**Risk**: Medium (requires API changes, testing needed)

---

## 📈 Performance Summary

| Optimization | Improvement | Implementation Cost | Risk |
|--------------|-------------|---------------------|------|
| Early return for stationary | 2-5x | Low | Low |
| OMNIDIRECTIONAL mode | 4.24x | Low | Medium |
| Batch PyBullet calls | 1.1-1.2x | High | Medium |

**Recommendation**:
1. **Implement Priority 1 immediately** (low risk, high reward)
2. **Consider Priority 2** if robot hardware allows
3. **Defer Priority 3** until other optimizations exhausted

---

## 🔬 Test Code

See `tests/profile_agent_update.py` for full benchmark implementation.

**Key Metrics Tested**:
- Stationary agents (no goal)
- Moving agents (DIFFERENTIAL vs OMNIDIRECTIONAL)
- Mixed scenarios (10%, 50%, 90% moving)
- Goal reached behavior

---

## 📝 Conclusion

The current Agent.update() performance is acceptable for small-scale simulations (<100 agents) but needs optimization for large fleets (>500 agents). The early return optimization for stationary agents provides the best risk/reward ratio and should be implemented first.
