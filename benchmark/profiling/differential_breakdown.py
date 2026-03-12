"""Micro-benchmark: differential drive sub-operation cost breakdown.

Measures the per-call cost of each sub-operation inside Agent._update_differential()
to identify where time is spent and which operations benefit from optimisation.

Key finding (before optimisation):
    scipy.spatial.transform.Slerp dominated the ROTATE phase at ~74% of per-step cost.
    The main reason is a design mismatch: Slerp is a *generic array-oriented API* that
    performs input normalisation, dtype promotion, validation, and Rotation object
    construction on every call — overhead that is negligible when interpolating thousands
    of points in one batch, but dominates when called once per tick with a single scalar.
    Similarly, np.clip is an array operation; for a single scalar, Python's built-in
    max/min avoids that overhead entirely.

    After replacing Slerp with a precomputed quaternion slerp using the math module, and
    np.clip with max/min, the ROTATE phase dropped from ~37 us to ~7 us per agent (5x).

Note on absolute numbers:
    Timings are environment-specific (CPU, Python version, NumPy/SciPy version).
    The *ratios* between old and new are the useful metric; absolute us values
    should be taken as one data point, not universal constants.
"""

import time

import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
from two_point_interpolation import TwoPointInterpolation

# Import the optimised slerp from agent.py for before/after comparison
from pybullet_fleet.geometry import quat_slerp as _quat_slerp, quat_slerp_precompute as _quat_slerp_precompute

N = 10000

# ===== Setup =====
tpi_rot = TwoPointInterpolation()
tpi_rot.init(p0=0.0, pe=1.5, acc_max=5.0, vmax=3.0, t0=0.0, v0=0.0, ve=0.0, dec_max=5.0)
tpi_rot.calc_trajectory()

tpi_fwd = TwoPointInterpolation()
tpi_fwd.init(p0=0.0, pe=5.0, acc_max=2.0, vmax=1.5, t0=0.0, v0=0.0, ve=0.0, dec_max=2.0)
tpi_fwd.calc_trajectory()

q_start = np.array([0.0, 0.0, 0.0, 1.0])
q_end = np.array([0.0, 0.0, 0.3827, 0.9239])
q_end = q_end / np.linalg.norm(q_end)  # normalise
key_rots = R.from_quat([q_start, q_end])
slerp_fn = Slerp([0.0, 1.0], key_rots)

# Fast slerp precomputed constants
slerp_precomp = _quat_slerp_precompute(q_start, q_end)

r_current = R.from_quat(q_start)
r_target = R.from_quat(q_end)

start_pos = np.array([0.0, 0.0, 0.0])
goal_pos = np.array([3.0, 4.0, 0.0])
direction_3d = goal_pos - start_pos
total_distance = float(np.linalg.norm(direction_3d))
direction_unit = direction_3d / total_distance

results = {}

# ===== Per-Step operations =====

# 1. TPI get_point (rotation)
times_mid = [t * tpi_rot.get_end_time() / N for t in range(N)]
start = time.perf_counter()
for t in times_mid:
    tpi_rot.get_point(t)
elapsed = time.perf_counter() - start
results["TPI.get_point (rotation)"] = elapsed / N * 1e6

# 2. TPI get_point (forward)
times_fwd = [t * tpi_fwd.get_end_time() / N for t in range(N)]
start = time.perf_counter()
for t in times_fwd:
    tpi_fwd.get_point(t)
elapsed = time.perf_counter() - start
results["TPI.get_point (forward)"] = elapsed / N * 1e6

# 3. TPI get_end_time
start = time.perf_counter()
for _ in range(N):
    tpi_rot.get_end_time()
elapsed = time.perf_counter() - start
results["TPI.get_end_time"] = elapsed / N * 1e6

# 4. Slerp: scipy (old) - single scalar per call
ratios = np.linspace(0.01, 0.99, N).tolist()
start = time.perf_counter()
for r in ratios:
    result = slerp_fn(r)
    result.as_quat().tolist()
elapsed = time.perf_counter() - start
results["scipy Slerp + as_quat + tolist"] = elapsed / N * 1e6

# 5. Slerp: fast precomputed (new) - single scalar per call
start = time.perf_counter()
for r in ratios:
    result = _quat_slerp(q_start, q_end, r, slerp_precomp)
    result.tolist()
elapsed = time.perf_counter() - start
results["fast _quat_slerp + tolist"] = elapsed / N * 1e6

# 6a. np.clip on scalar (old)
vals = np.random.random(N).tolist()
start = time.perf_counter()
for v in vals:
    np.clip(v, 0.0, 1.0)
elapsed = time.perf_counter() - start
results["np.clip (scalar, old)"] = elapsed / N * 1e6

# 6b. max/min on scalar (new)
start = time.perf_counter()
for v in vals:
    max(0.0, min(1.0, v))
elapsed = time.perf_counter() - start
results["max/min (scalar, new)"] = elapsed / N * 1e6

# 7. np.array from list
pos_list = [1.0, 2.0, 3.0]
start = time.perf_counter()
for _ in range(N):
    np.array(pos_list)
elapsed = time.perf_counter() - start
results["np.array([x,y,z])"] = elapsed / N * 1e6

# 8. Forward phase vector math
start = time.perf_counter()
for i in range(N):
    dist = i / N * total_distance
    ratio = dist / total_distance
    new_pos = start_pos + direction_3d * ratio
    vel = direction_unit * 1.0
    new_pos.tolist()
elapsed = time.perf_counter() - start
results["Forward math (ratio+interp+tolist)"] = elapsed / N * 1e6

# ===== Per-Goal-Init operations =====

# 9. Rotation delta + magnitude
start = time.perf_counter()
for _ in range(N):
    r_delta = r_target * r_current.inv()
    r_delta.magnitude()
elapsed = time.perf_counter() - start
results["R delta + magnitude (init)"] = elapsed / N * 1e6

# 10. Slerp constructor (no longer used, kept for comparison)
start = time.perf_counter()
for _ in range(N):
    Slerp([0.0, 1.0], key_rots)
elapsed = time.perf_counter() - start
results["Slerp constructor (init, old)"] = elapsed / N * 1e6

# 11. _quat_slerp_precompute (new init replacement)
start = time.perf_counter()
for _ in range(N):
    _quat_slerp_precompute(q_start, q_end)
elapsed = time.perf_counter() - start
results["_quat_slerp_precompute (init, new)"] = elapsed / N * 1e6

# 12. TPI constructor + init + calc_trajectory
start = time.perf_counter()
for _ in range(N):
    t = TwoPointInterpolation()
    t.init(p0=0.0, pe=1.5, acc_max=5.0, vmax=3.0, t0=0.0, v0=0.0, ve=0.0, dec_max=5.0)
    t.calc_trajectory()
elapsed = time.perf_counter() - start
results["TPI new+init+calc (init)"] = elapsed / N * 1e6

# 13. np.column_stack + R.from_matrix + as_quat (orientation build in init)
x_ax = np.array([0.6, 0.8, 0.0])
y_ax = np.array([-0.8, 0.6, 0.0])
z_ax = np.array([0.0, 0.0, 1.0])
start = time.perf_counter()
for _ in range(N):
    mat = np.column_stack([x_ax, y_ax, z_ax])
    r = R.from_matrix(mat)
    r.as_quat()
elapsed = time.perf_counter() - start
results["column_stack+from_matrix+as_quat (init)"] = elapsed / N * 1e6

# ===== Print results =====
print()
print("=" * 72)
print("  Differential Drive Sub-operation Cost Breakdown")
print("=" * 72)

# --- Per-Step: old vs new ---
print()
print("--- Per-Step: Before vs After Optimisation ---")
print()

slerp_old = results["scipy Slerp + as_quat + tolist"]
slerp_new = results["fast _quat_slerp + tolist"]
clip_old = results["np.clip (scalar, old)"]
clip_new = results["max/min (scalar, new)"]
tpi_rot_cost = results["TPI.get_point (rotation)"]
tpi_fwd_cost = results["TPI.get_point (forward)"]
tpi_end = results["TPI.get_end_time"]
np_array = results["np.array([x,y,z])"]
fwd_math = results["Forward math (ratio+interp+tolist)"]

hdr = f"  {'Operation':<50} {'us/call':>8}"
print(hdr)
print(f"  {'-' * 50} {'-' * 8}")
print(f"  {'TPI.get_point (rotation)':<50} {tpi_rot_cost:>8.2f}")
print(f"  {'TPI.get_point (forward)':<50} {tpi_fwd_cost:>8.2f}")
print(f"  {'TPI.get_end_time':<50} {tpi_end:>8.2f}")
print(f"  {'np.array([x,y,z])':<50} {np_array:>8.2f}")
print(f"  {'Forward math (ratio+interp+tolist)':<50} {fwd_math:>8.2f}")
print()
print(f"  {'scipy Slerp + as_quat + tolist (old)':<50} {slerp_old:>8.2f}")
print(f"  {'fast _quat_slerp + tolist        (new)':<50} {slerp_new:>8.2f}  ({slerp_old / slerp_new:.1f}x)")
print()
print(f"  {'np.clip scalar                   (old)':<50} {clip_old:>8.2f}")
print(f"  {'max/min scalar                   (new)':<50} {clip_new:>8.2f}  ({clip_old / clip_new:.1f}x)")

# --- ROTATE phase totals ---
print()
print("--- ROTATE Phase Per-Step Total ---")
rot_old = tpi_rot_cost + tpi_end + clip_old + slerp_old + np_array
rot_new = tpi_rot_cost + tpi_end + clip_new + slerp_new + np_array
print(f"  {'Before (scipy Slerp + np.clip):':<50} {rot_old:>8.2f} us")
print(f"  {'After  (fast slerp + max/min):':<50} {rot_new:>8.2f} us  ({rot_old / rot_new:.1f}x)")

# --- FORWARD phase ---
fwd_cost = tpi_fwd_cost + tpi_end + fwd_math + np_array
print()
print(f"  {'FORWARD phase (unchanged):':<50} {fwd_cost:>8.2f} us")

# --- Per-Goal-Init ---
print()
print("--- Per-Goal-Init (when new waypoint is set) ---")
print()
init_old_keys = [
    "R delta + magnitude (init)",
    "Slerp constructor (init, old)",
    "TPI new+init+calc (init)",
    "column_stack+from_matrix+as_quat (init)",
]
init_new_keys = [
    "R delta + magnitude (init)",
    "_quat_slerp_precompute (init, new)",
    "TPI new+init+calc (init)",
    "column_stack+from_matrix+as_quat (init)",
]
for name in init_old_keys:
    v = results[name]
    print(f"  {name:<50} {v:>8.2f}")
precomp = results["_quat_slerp_precompute (init, new)"]
slerp_ctor = results["Slerp constructor (init, old)"]
print()
print(f"  {'_quat_slerp_precompute (replaces Slerp ctor):':<50} {precomp:>8.2f}  ({slerp_ctor / precomp:.1f}x)")

init_old = sum(results[k] for k in init_old_keys)
init_new = sum(results[k] for k in init_new_keys)
print(f"  {'Init total (old):':<50} {init_old:>8.2f}")
print(f"  {'Init total (new):':<50} {init_new:>8.2f}  ({init_old / init_new:.1f}x)")

# --- 500-agent frame estimate ---
print()
print("--- 500-Agent Frame Estimate ---")
print(f"  ROTATE  (old): 500 x {rot_old:.1f}us = {500 * rot_old / 1000:.2f} ms/frame")
print(f"  ROTATE  (new): 500 x {rot_new:.1f}us = {500 * rot_new / 1000:.2f} ms/frame")
print(f"  FORWARD:       500 x {fwd_cost:.1f}us = {500 * fwd_cost / 1000:.2f} ms/frame")
print(f"  Saved:         {500 * (rot_old - rot_new) / 1000:.2f} ms/frame in ROTATE")
