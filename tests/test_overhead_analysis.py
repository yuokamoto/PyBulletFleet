#!/usr/bin/env python3
"""
test_overhead_analysis_v3_cpu.py
Overhead analysis with process isolation + CPU time metrics.

Adds:
- CPU time (user+sys) for spawn/update sections inside CHILD
- CPU utilization ratio (cpu_time / wall_time) as a sanity check against background noise

Notes:
- CPU time is generally more stable against background processes than wall time,
  but not perfectly immune (scheduling, contention, throttling can still affect).
"""
import os
import sys
import time
import gc
import json
import psutil
import tracemalloc
import statistics
import subprocess
import argparse

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pybullet as p
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.sim_object import SimObject, ShapeParams, SimObjectSpawnParams
from pybullet_fleet.agent_manager import SimObjectManager, AgentManager, GridSpawnParams
from pybullet_fleet.agent import Agent, AgentSpawnParams, MotionMode
from pybullet_fleet.geometry import Pose


# ==================== Configuration ====================
# Production scenario
N_MAX_OBJECTS = 10000  # Maximum expected objects in production
ACCEPTABLE_MEMORY_OVERHEAD_MB = 100  # Acceptable memory overhead at N_MAX
ACCEPTABLE_SPAWN_TIME_S = 5.0  # Acceptable TOTAL spawn time at N_MAX
ACCEPTABLE_UPDATE_TIME_MS = 10.0  # Acceptable TOTAL update time at N_MAX

# Test parameters
NUM_REPETITIONS = 5  # Number of repetitions for statistical stability
NUM_TEST_OBJECTS = 10000  # Number of objects per test (will extrapolate to N_MAX)


# ==================== Helpers ====================
def get_memory_info():
    """Get memory info (RSS + tracemalloc current)."""
    process = psutil.Process()
    mem = process.memory_info()

    py_traced_mb = 0.0
    if tracemalloc.is_tracing():
        current, _peak = tracemalloc.get_traced_memory()
        py_traced_mb = current / 1024 / 1024

    rss_mb = mem.rss / 1024 / 1024
    return {
        "rss_mb": rss_mb,
        "vms_mb": mem.vms / 1024 / 1024,
        "py_traced_mb": py_traced_mb,
        # Not "native". Just RSS minus what tracemalloc sees.
        "rss_minus_tracemalloc_mb": rss_mb - py_traced_mb,
    }


def force_cleanup():
    """Best-effort cleanup inside a process."""
    gc.collect()
    gc.collect()
    gc.collect()
    time.sleep(0.05)


def cpu_time_s(process: psutil.Process) -> float:
    """Return user+sys CPU time seconds."""
    t = process.cpu_times()
    return float(t.user + t.system)


# ==================== Tests (CHILD) ====================
def test_direct_pybullet(num_objects: int) -> dict:
    proc = psutil.Process()
    tracemalloc.start()
    force_cleanup()

    if p.isConnected():
        p.disconnect()
    p.connect(p.DIRECT)
    p.resetSimulation()
    p.setGravity(0, 0, -9.81)

    mem_before = get_memory_info()

    vis_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05], rgbaColor=[1, 0, 0, 1])
    col_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05])

    # Spawn timing (wall + CPU)
    cpu0 = cpu_time_s(proc)
    w0 = time.perf_counter()
    bodies = []
    for i in range(num_objects):
        x = (i % 100) * 0.2
        y = (i // 100) * 0.2
        body_id = p.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=col_id,
            baseVisualShapeIndex=vis_id,
            basePosition=[x, y, 0.1],
            baseOrientation=[0, 0, 0, 1],
        )
        bodies.append(body_id)
    spawn_wall_s = time.perf_counter() - w0
    spawn_cpu_s = cpu_time_s(proc) - cpu0

    mem_after = get_memory_info()

    # Update: get_pose (wall + CPU)
    cpu1 = cpu_time_s(proc)
    w1 = time.perf_counter()
    poses = [p.getBasePositionAndOrientation(bid) for bid in bodies]
    get_pose_wall_s = time.perf_counter() - w1
    get_pose_cpu_s = cpu_time_s(proc) - cpu1

    # Update: set_pose (wall + CPU) - move slightly up to match Agent behavior
    cpu2 = cpu_time_s(proc)
    w2 = time.perf_counter()
    for bid, (pos, orn) in zip(bodies, poses):
        new_pos = [pos[0], pos[1], pos[2] + 0.01]
        p.resetBasePositionAndOrientation(bid, new_pos, orn)
    set_pose_wall_s = time.perf_counter() - w2
    set_pose_cpu_s = cpu_time_s(proc) - cpu2

    # Combined update time
    update_wall_s = get_pose_wall_s + set_pose_wall_s
    update_cpu_s = get_pose_cpu_s + set_pose_cpu_s
    update_wall_ms = update_wall_s * 1000.0

    mem_delta = {
        "rss_mb": mem_after["rss_mb"] - mem_before["rss_mb"],
        "py_traced_mb": mem_after["py_traced_mb"] - mem_before["py_traced_mb"],
        "rss_minus_tracemalloc_mb": mem_after["rss_minus_tracemalloc_mb"] - mem_before["rss_minus_tracemalloc_mb"],
    }

    # Cleanup
    p.resetSimulation()
    p.disconnect()
    tracemalloc.stop()
    del bodies, poses
    force_cleanup()

    return {
        "test": "direct",
        "num_objects": num_objects,
        "spawn_time_s": spawn_wall_s,
        "update_time_ms": update_wall_ms,
        "get_pose_time_ms": get_pose_wall_s * 1000.0,
        "set_pose_time_ms": set_pose_wall_s * 1000.0,
        "mem_delta_mb": mem_delta,
        "cpu_spawn_s": spawn_cpu_s,
        "cpu_update_s": update_cpu_s,
        "cpu_get_pose_s": get_pose_cpu_s,
        "cpu_set_pose_s": set_pose_cpu_s,
        "cpu_total_s": spawn_cpu_s + update_cpu_s,
        "cpu_spawn_percent": (spawn_cpu_s / spawn_wall_s * 100.0) if spawn_wall_s > 0 else 0.0,
        "cpu_update_percent": (update_cpu_s / update_wall_s * 100.0) if update_wall_s > 0 else 0.0,
    }


def test_simobject_wrapper(num_objects: int) -> dict:
    proc = psutil.Process()
    tracemalloc.start()
    force_cleanup()

    if p.isConnected():
        p.disconnect()

    params = SimulationParams(gui=False, timestep=0.01, monitor=True, enable_monitor_gui=False)
    sim_core = MultiRobotSimulationCore(params)

    p.resetSimulation()
    p.setGravity(0, 0, -9.81)

    mem_before = get_memory_info()

    shape_params = ShapeParams(
        shape_type="box",
        half_extents=[0.05, 0.05, 0.05],
        rgba_color=[1, 0, 0, 1],
    )

    # Spawn (wall + CPU)
    cpu0 = cpu_time_s(proc)
    w0 = time.perf_counter()
    objects = []
    for i in range(num_objects):
        x = (i % 100) * 0.2
        y = (i // 100) * 0.2
        obj = SimObject.from_mesh(
            visual_shape=shape_params,
            collision_shape=shape_params,
            pose=Pose.from_xyz(x, y, 0.1),
            mass=0.0,
            sim_core=sim_core,
        )
        objects.append(obj)
    spawn_wall_s = time.perf_counter() - w0
    spawn_cpu_s = cpu_time_s(proc) - cpu0

    mem_after = get_memory_info()

    # Update: get_pose (wall + CPU)
    cpu1 = cpu_time_s(proc)
    w1 = time.perf_counter()
    poses = [obj.get_pose() for obj in objects]
    get_pose_wall_s = time.perf_counter() - w1
    get_pose_cpu_s = cpu_time_s(proc) - cpu1

    # Update: set_pose (wall + CPU)
    new_poses = [Pose.from_xyz(p.position[0], p.position[1], p.position[2] + 0.01) for p in poses]
    cpu2 = cpu_time_s(proc)
    w2 = time.perf_counter()
    for obj, new_pose in zip(objects, new_poses):
        obj.set_pose(new_pose)
    set_pose_wall_s = time.perf_counter() - w2
    set_pose_cpu_s = cpu_time_s(proc) - cpu2

    # Combined update time
    update_wall_s = get_pose_wall_s + set_pose_wall_s
    update_cpu_s = get_pose_cpu_s + set_pose_cpu_s
    update_wall_ms = update_wall_s * 1000.0

    mem_delta = {
        "rss_mb": mem_after["rss_mb"] - mem_before["rss_mb"],
        "py_traced_mb": mem_after["py_traced_mb"] - mem_before["py_traced_mb"],
        "rss_minus_tracemalloc_mb": mem_after["rss_minus_tracemalloc_mb"] - mem_before["rss_minus_tracemalloc_mb"],
    }

    cache_size = len(getattr(SimObject, "_shared_shapes", {}))

    # Cleanup
    p.resetSimulation()
    if p.isConnected():
        p.disconnect()
    tracemalloc.stop()
    del objects, poses, new_poses, sim_core
    force_cleanup()

    return {
        "test": "simobject",
        "num_objects": num_objects,
        "spawn_time_s": spawn_wall_s,
        "update_time_ms": update_wall_ms,
        "get_pose_time_ms": get_pose_wall_s * 1000.0,
        "set_pose_time_ms": set_pose_wall_s * 1000.0,
        "mem_delta_mb": mem_delta,
        "cache_size": cache_size,
        "cpu_spawn_s": spawn_cpu_s,
        "cpu_update_s": update_cpu_s,
        "cpu_get_pose_s": get_pose_cpu_s,
        "cpu_set_pose_s": set_pose_cpu_s,
        "cpu_total_s": spawn_cpu_s + update_cpu_s,
        "cpu_spawn_percent": (spawn_cpu_s / spawn_wall_s * 100.0) if spawn_wall_s > 0 else 0.0,
        "cpu_update_percent": (update_cpu_s / update_wall_s * 100.0) if update_wall_s > 0 else 0.0,
    }


def test_simobject_manager(num_objects: int) -> dict:
    proc = psutil.Process()
    tracemalloc.start()
    force_cleanup()

    if p.isConnected():
        p.disconnect()

    params = SimulationParams(gui=False, timestep=0.01, monitor=True, enable_monitor_gui=False)
    sim_core = MultiRobotSimulationCore(params)

    p.resetSimulation()
    p.setGravity(0, 0, -9.81)

    mem_before = get_memory_info()

    manager = SimObjectManager(sim_core=sim_core)

    rows = (num_objects + 99) // 100
    grid_params = GridSpawnParams(
        x_min=0,
        x_max=99,
        y_min=0,
        y_max=rows - 1,
        z_min=0,
        z_max=0,
        spacing=[0.2, 0.2, 0.0],
        offset=[0.0, 0.0, 0.1],
    )

    spawn_params = SimObjectSpawnParams(
        visual_shape=ShapeParams(
            shape_type="box",
            half_extents=[0.05, 0.05, 0.05],
            rgba_color=[1, 0, 0, 1],
        ),
        collision_shape=ShapeParams(
            shape_type="box",
            half_extents=[0.05, 0.05, 0.05],
        ),
        mass=0.0,
    )

    # Spawn (wall + CPU)
    cpu0 = cpu_time_s(proc)
    w0 = time.perf_counter()
    objects = manager.spawn_objects_grid(
        num_objects=num_objects,
        grid_params=grid_params,
        spawn_params=spawn_params,
    )
    spawn_wall_s = time.perf_counter() - w0
    spawn_cpu_s = cpu_time_s(proc) - cpu0

    mem_after = get_memory_info()

    # Update (wall + CPU)
    cpu1 = cpu_time_s(proc)
    w1 = time.perf_counter()
    _poses = manager.get_all_poses()
    update_wall_s = time.perf_counter() - w1
    update_cpu_s = cpu_time_s(proc) - cpu1
    update_wall_ms = update_wall_s * 1000.0

    mem_delta = {
        "rss_mb": mem_after["rss_mb"] - mem_before["rss_mb"],
        "py_traced_mb": mem_after["py_traced_mb"] - mem_before["py_traced_mb"],
        "rss_minus_tracemalloc_mb": mem_after["rss_minus_tracemalloc_mb"] - mem_before["rss_minus_tracemalloc_mb"],
    }

    # Cleanup
    p.resetSimulation()
    if p.isConnected():
        p.disconnect()
    tracemalloc.stop()
    del objects, _poses, manager, sim_core
    force_cleanup()

    return {
        "test": "manager",
        "num_objects": num_objects,
        "spawn_time_s": spawn_wall_s,
        "update_time_ms": update_wall_ms,
        "mem_delta_mb": mem_delta,
        "cpu_spawn_s": spawn_cpu_s,
        "cpu_update_s": update_cpu_s,
        "cpu_total_s": spawn_cpu_s + update_cpu_s,
        "cpu_spawn_percent": (spawn_cpu_s / spawn_wall_s * 100.0) if spawn_wall_s > 0 else 0.0,
        "cpu_update_percent": (update_cpu_s / update_wall_s * 100.0) if update_wall_s > 0 else 0.0,
    }


def test_agent_wrapper(num_objects: int) -> dict:
    """Test Agent wrapper overhead (get_pose + set_pose)."""
    proc = psutil.Process()
    tracemalloc.start()
    force_cleanup()

    if p.isConnected():
        p.disconnect()

    params = SimulationParams(gui=False, timestep=0.01, monitor=False)
    sim_core = MultiRobotSimulationCore(params)

    p.resetSimulation()
    p.setGravity(0, 0, -9.81)

    mem_before = get_memory_info()

    # Prepare URDF path (use simple cube for performance testing)
    robot_urdf = os.path.join(os.path.dirname(__file__), "../robots/simple_cube.urdf")

    # Spawn (wall + CPU)
    cpu0 = cpu_time_s(proc)
    w0 = time.perf_counter()

    agents = []
    for i in range(num_objects):
        x = (i % 100) * 0.5
        y = (i // 100) * 0.5
        spawn_params = AgentSpawnParams(
            urdf_path=robot_urdf,
            initial_pose=Pose.from_xyz(x, y, 0.1),
            motion_mode=MotionMode.DIFFERENTIAL,
            max_linear_vel=1.0,
            max_angular_vel=1.0,
            mass=0.0,
        )
        agent = Agent.from_params(spawn_params, sim_core=sim_core)
        agents.append(agent)

    spawn_wall_s = time.perf_counter() - w0
    spawn_cpu_s = cpu_time_s(proc) - cpu0

    mem_after = get_memory_info()

    # Update: get_pose (wall + CPU)
    cpu1 = cpu_time_s(proc)
    w1 = time.perf_counter()
    poses = [agent.get_pose() for agent in agents]
    get_pose_wall_s = time.perf_counter() - w1
    get_pose_cpu_s = cpu_time_s(proc) - cpu1

    # Update: set_pose (wall + CPU)
    new_poses = [Pose.from_xyz(p.position[0], p.position[1], p.position[2] + 0.01) for p in poses]
    cpu2 = cpu_time_s(proc)
    w2 = time.perf_counter()
    for agent, new_pose in zip(agents, new_poses):
        agent.set_pose(new_pose)
    set_pose_wall_s = time.perf_counter() - w2
    set_pose_cpu_s = cpu_time_s(proc) - cpu2

    # Combined update time
    update_wall_s = get_pose_wall_s + set_pose_wall_s
    update_cpu_s = get_pose_cpu_s + set_pose_cpu_s
    update_wall_ms = update_wall_s * 1000.0

    mem_delta = {
        "rss_mb": mem_after["rss_mb"] - mem_before["rss_mb"],
        "py_traced_mb": mem_after["py_traced_mb"] - mem_before["py_traced_mb"],
        "rss_minus_tracemalloc_mb": mem_after["rss_minus_tracemalloc_mb"] - mem_before["rss_minus_tracemalloc_mb"],
    }

    # Cleanup
    p.resetSimulation()
    if p.isConnected():
        p.disconnect()
    tracemalloc.stop()
    del agents, poses, new_poses, sim_core
    force_cleanup()

    return {
        "test": "agent",
        "num_objects": num_objects,
        "spawn_time_s": spawn_wall_s,
        "update_time_ms": update_wall_ms,
        "get_pose_time_ms": get_pose_wall_s * 1000.0,
        "set_pose_time_ms": set_pose_wall_s * 1000.0,
        "mem_delta_mb": mem_delta,
        "cpu_spawn_s": spawn_cpu_s,
        "cpu_update_s": update_cpu_s,
        "cpu_get_pose_s": get_pose_cpu_s,
        "cpu_set_pose_s": set_pose_cpu_s,
        "cpu_total_s": spawn_cpu_s + update_cpu_s,
        "cpu_spawn_percent": (spawn_cpu_s / spawn_wall_s * 100.0) if spawn_wall_s > 0 else 0.0,
        "cpu_update_percent": (update_cpu_s / update_wall_s * 100.0) if update_wall_s > 0 else 0.0,
    }


def test_agent_manager(num_objects: int) -> dict:
    """Test AgentManager bulk spawn + update (get_pose + set_goal_pose)."""
    proc = psutil.Process()
    tracemalloc.start()
    force_cleanup()

    if p.isConnected():
        p.disconnect()

    params = SimulationParams(gui=False, timestep=0.01, monitor=False)
    sim_core = MultiRobotSimulationCore(params)

    p.resetSimulation()
    p.setGravity(0, 0, -9.81)

    mem_before = get_memory_info()

    manager = AgentManager(sim_core=sim_core)

    rows = (num_objects + 99) // 100
    grid_params = GridSpawnParams(
        x_min=0,
        x_max=99,
        y_min=0,
        y_max=rows - 1,
        z_min=0,
        z_max=0,
        spacing=[0.5, 0.5, 0.0],
        offset=[0.0, 0.0, 0.1],
    )

    # Use simple cube URDF for performance testing
    robot_urdf = os.path.join(os.path.dirname(__file__), "../robots/simple_cube.urdf")
    agent_spawn_params = AgentSpawnParams(
        urdf_path=robot_urdf,
        motion_mode=MotionMode.DIFFERENTIAL,
        max_linear_vel=1.0,
        max_angular_vel=1.0,
        mass=0.0,
    )

    # Spawn (wall + CPU)
    cpu0 = cpu_time_s(proc)
    w0 = time.perf_counter()
    agents = manager.spawn_agents_grid(
        num_agents=num_objects,
        grid_params=grid_params,
        spawn_params=agent_spawn_params,
    )
    spawn_wall_s = time.perf_counter() - w0
    spawn_cpu_s = cpu_time_s(proc) - cpu0

    mem_after = get_memory_info()

    # Update: get_all_poses (wall + CPU)
    cpu1 = cpu_time_s(proc)
    w1 = time.perf_counter()
    poses = manager.get_all_poses()
    get_pose_wall_s = time.perf_counter() - w1
    get_pose_cpu_s = cpu_time_s(proc) - cpu1

    # Update: set_pose (bulk) - wall + CPU
    # Note: Using set_pose instead of set_goal_pose because set_goal_pose is not
    # intended to be called every step (it recalculates trajectories).
    # set_pose is the equivalent of SimObject.set_pose for direct position updates.
    new_poses = [Pose.from_xyz(p.position[0], p.position[1], p.position[2] + 0.01) for p in poses]
    cpu2 = cpu_time_s(proc)
    w2 = time.perf_counter()
    for i, agent in enumerate(agents):
        agent.set_pose(new_poses[i])
    set_pose_wall_s = time.perf_counter() - w2
    set_pose_cpu_s = cpu_time_s(proc) - cpu2

    # Combined update time
    update_wall_s = get_pose_wall_s + set_pose_wall_s
    update_cpu_s = get_pose_cpu_s + set_pose_cpu_s
    update_wall_ms = update_wall_s * 1000.0

    mem_delta = {
        "rss_mb": mem_after["rss_mb"] - mem_before["rss_mb"],
        "py_traced_mb": mem_after["py_traced_mb"] - mem_before["py_traced_mb"],
        "rss_minus_tracemalloc_mb": mem_after["rss_minus_tracemalloc_mb"] - mem_before["rss_minus_tracemalloc_mb"],
    }

    # Cleanup
    p.resetSimulation()
    if p.isConnected():
        p.disconnect()
    tracemalloc.stop()
    del agents, poses, new_poses, manager, sim_core
    force_cleanup()

    return {
        "test": "agent_manager",
        "num_objects": num_objects,
        "spawn_time_s": spawn_wall_s,
        "update_time_ms": update_wall_ms,
        "get_pose_time_ms": get_pose_wall_s * 1000.0,
        "set_pose_time_ms": set_pose_wall_s * 1000.0,
        "mem_delta_mb": mem_delta,
        "cpu_spawn_s": spawn_cpu_s,
        "cpu_update_s": update_cpu_s,
        "cpu_get_pose_s": get_pose_cpu_s,
        "cpu_set_pose_s": set_pose_cpu_s,
        "cpu_total_s": spawn_cpu_s + update_cpu_s,
        "cpu_spawn_percent": (spawn_cpu_s / spawn_wall_s * 100.0) if spawn_wall_s > 0 else 0.0,
        "cpu_update_percent": (update_cpu_s / update_wall_s * 100.0) if update_wall_s > 0 else 0.0,
    }


TEST_FUNCS = {
    "direct": test_direct_pybullet,
    "simobject": test_simobject_wrapper,
    "manager": test_simobject_manager,
    "agent": test_agent_wrapper,
    "agent_manager": test_agent_manager,
}


# ==================== Parent runner ====================
def run_child_once(test_name: str, num_objects: int) -> dict:
    cmd = [
        sys.executable,
        os.path.abspath(__file__),
        "--child",
        "--test",
        test_name,
        "--n",
        str(num_objects),
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(f"Child failed (test={test_name})\n" f"STDOUT:\n{proc.stdout}\n" f"STDERR:\n{proc.stderr}\n")

    # Extract JSON from stdout (filter out PyBullet warnings that may appear on same line)
    # JSON starts with '{' and ends with '}', so we extract the first complete JSON object
    stdout = proc.stdout.strip()
    if not stdout:
        raise RuntimeError(f"Child produced no JSON output (test={test_name}). STDERR:\n{proc.stderr}")

    # Find the first '{' and the last '}' to extract JSON
    json_start = stdout.find("{")
    json_end = stdout.rfind("}")

    if json_start == -1 or json_end == -1 or json_start >= json_end:
        raise RuntimeError(
            f"No valid JSON found in output (test={test_name})\n"
            f"Raw stdout:\n{proc.stdout}\n"
            f"Raw stderr:\n{proc.stderr}\n"
        )

    json_str = stdout[json_start : json_end + 1]

    try:
        return json.loads(json_str)
    except json.JSONDecodeError as e:
        raise RuntimeError(
            f"Failed to parse child JSON (test={test_name}): {e}\n"
            f"Raw stdout:\n{proc.stdout}\n"
            f"Raw stderr:\n{proc.stderr}\n"
        )


def run_multiple_tests_isolated(test_name: str, num_reps: int, num_objects: int) -> dict:
    results = []
    for i in range(num_reps):
        print(f"  Rep {i+1}/{num_reps}...", end="", flush=True)
        r = run_child_once(test_name, num_objects)
        results.append(r)
        print(" done")

    def stats(xs):
        return {
            "median": statistics.median(xs),
            "mean": statistics.mean(xs),
            "stdev": statistics.stdev(xs) if len(xs) > 1 else 0.0,
            "min": min(xs),
            "max": max(xs),
        }

    spawn = [r["spawn_time_s"] for r in results]
    update = [r["update_time_ms"] for r in results]
    rss = [r["mem_delta_mb"]["rss_mb"] for r in results]
    py_traced = [r["mem_delta_mb"]["py_traced_mb"] for r in results]
    rss_minus = [r["mem_delta_mb"]["rss_minus_tracemalloc_mb"] for r in results]

    cpu_spawn = [r["cpu_spawn_s"] for r in results]
    cpu_update = [r["cpu_update_s"] for r in results]
    cpu_total = [r["cpu_total_s"] for r in results]
    cpu_spawn_pct = [r["cpu_spawn_percent"] for r in results]
    cpu_update_pct = [r["cpu_update_percent"] for r in results]

    out = {
        "test": test_name,
        "num_objects": num_objects,
        "num_reps": num_reps,
        "spawn_time_s": stats(spawn),
        "update_time_ms": stats(update),
        "mem_rss_mb": stats(rss),
        "mem_py_traced_mb": stats(py_traced),
        "mem_rss_minus_tracemalloc_mb": stats(rss_minus),
        "cpu_spawn_s": stats(cpu_spawn),
        "cpu_update_s": stats(cpu_update),
        "cpu_total_s": stats(cpu_total),
        "cpu_spawn_percent": stats(cpu_spawn_pct),
        "cpu_update_percent": stats(cpu_update_pct),
    }

    # Optional fields for agent tests
    get_pose_times = [r.get("get_pose_time_ms") for r in results if "get_pose_time_ms" in r]
    set_pose_times = [r.get("set_pose_time_ms") for r in results if "set_pose_time_ms" in r]
    if get_pose_times:
        out["get_pose_time_ms"] = stats(get_pose_times)
    if set_pose_times:
        out["set_pose_time_ms"] = stats(set_pose_times)

    cache_sizes = [r.get("cache_size") for r in results if "cache_size" in r]
    if cache_sizes:
        out["cache_size"] = stats(cache_sizes)

    return out


def print_stats(label: str, st: dict):
    print(f"\n{label}:")
    print(
        f"  Spawn time: {st['spawn_time_s']['median']:.3f}s "
        f"(±{st['spawn_time_s']['stdev']:.3f}s, "
        f"min={st['spawn_time_s']['min']:.3f}, max={st['spawn_time_s']['max']:.3f})"
    )
    print(
        f"  Update time: {st['update_time_ms']['median']:.2f}ms "
        f"(±{st['update_time_ms']['stdev']:.2f}ms, "
        f"min={st['update_time_ms']['min']:.2f}, max={st['update_time_ms']['max']:.2f})"
    )

    # Show get_pose / set_pose breakdown if available
    if "get_pose_time_ms" in st:
        print(f"    ├─ get_pose: {st['get_pose_time_ms']['median']:.2f}ms " f"(±{st['get_pose_time_ms']['stdev']:.2f}ms)")
    if "set_pose_time_ms" in st:
        print(f"    └─ set_pose: {st['set_pose_time_ms']['median']:.2f}ms " f"(±{st['set_pose_time_ms']['stdev']:.2f}ms)")

    print(f"  RSS delta: {st['mem_rss_mb']['median']:.2f}MB " f"(±{st['mem_rss_mb']['stdev']:.2f}MB)")
    print(f"  tracemalloc delta: {st['mem_py_traced_mb']['median']:.2f}MB " f"(±{st['mem_py_traced_mb']['stdev']:.2f}MB)")
    print(
        f"  RSS - tracemalloc: {st['mem_rss_minus_tracemalloc_mb']['median']:.2f}MB "
        f"(±{st['mem_rss_minus_tracemalloc_mb']['stdev']:.2f}MB)"
    )

    print(
        f"  CPU time (spawn): {st['cpu_spawn_s']['median']:.3f}s "
        f"(±{st['cpu_spawn_s']['stdev']:.3f}s)  "
        f"util~{st['cpu_spawn_percent']['median']:.1f}%"
    )
    print(
        f"  CPU time (update): {st['cpu_update_s']['median']:.3f}s "
        f"(±{st['cpu_update_s']['stdev']:.3f}s)  "
        f"util~{st['cpu_update_percent']['median']:.1f}%"
    )
    print(f"  CPU time (total): {st['cpu_total_s']['median']:.3f}s " f"(±{st['cpu_total_s']['stdev']:.3f}s)")

    if "cache_size" in st:
        print(
            f"  Cache size: median={st['cache_size']['median']}, "
            f"min={st['cache_size']['min']}, max={st['cache_size']['max']}"
        )


def print_stats_run_simulation(label: str, st: dict):
    """Print statistics for run_simulation test."""
    print(f"\n{label}:")
    print(
        f"  Spawn time: {st['spawn_time_s']['median']:.3f}s "
        f"(±{st['spawn_time_s']['stdev']:.3f}s, "
        f"min={st['spawn_time_s']['min']:.3f}, max={st['spawn_time_s']['max']:.3f})"
    )

    if "simulation_duration_s" in st:
        print(f"  Simulation duration: {st['simulation_duration_s']['median']:.2f}s")

    if "simulation_wall_s" in st:
        print(
            f"  Wall time (simulation): {st['simulation_wall_s']['median']:.3f}s "
            f"(±{st['simulation_wall_s']['stdev']:.3f}s, "
            f"min={st['simulation_wall_s']['min']:.3f}, "
            f"max={st['simulation_wall_s']['max']:.3f})"
        )

    if "real_time_factor" in st:
        print(
            f"  Real-Time Factor: {st['real_time_factor']['median']:.2f}x "
            f"(±{st['real_time_factor']['stdev']:.2f}x, "
            f"min={st['real_time_factor']['min']:.2f}x, "
            f"max={st['real_time_factor']['max']:.2f}x)"
        )

    if "expected_steps" in st:
        print(f"  Expected steps: {st['expected_steps']['median']:.0f}")

    if "avg_step_time_ms" in st:
        print(f"  Avg step time: {st['avg_step_time_ms']['median']:.2f}ms " f"(±{st['avg_step_time_ms']['stdev']:.2f}ms)")

    print(f"  RSS delta: {st['mem_rss_mb']['median']:.2f}MB (±{st['mem_rss_mb']['stdev']:.2f}MB)")
    print(f"  tracemalloc delta: {st['mem_py_traced_mb']['median']:.2f}MB " f"(±{st['mem_py_traced_mb']['stdev']:.2f}MB)")
    print(
        f"  RSS - tracemalloc: {st['mem_rss_minus_tracemalloc_mb']['median']:.2f}MB "
        f"(±{st['mem_rss_minus_tracemalloc_mb']['stdev']:.2f}MB)"
    )

    print(
        f"  CPU time (spawn): {st['cpu_spawn_s']['median']:.3f}s "
        f"(±{st['cpu_spawn_s']['stdev']:.3f}s)  "
        f"util~{st['cpu_spawn_percent']['median']:.1f}%"
    )

    if "cpu_simulation_s" in st:
        print(
            f"  CPU time (simulation): {st['cpu_simulation_s']['median']:.3f}s "
            f"(±{st['cpu_simulation_s']['stdev']:.3f}s)  "
            f"util~{st['cpu_simulation_percent']['median']:.1f}%"
        )

    print(f"  CPU time (total): {st['cpu_total_s']['median']:.3f}s (±{st['cpu_total_s']['stdev']:.3f}s)")


# ==================== Main (PARENT) ====================
def main_parent(args):
    print("=" * 70)
    print("SimObject Overhead Analysis v3+CPU - Process Isolated")
    print("=" * 70)
    print("\nConfiguration:")
    print(f"  Test objects: {args.n}")
    print(f"  Repetitions: {args.reps}")
    print(f"  Max production objects: {N_MAX_OBJECTS}")
    print(f"  Acceptable memory overhead (@{N_MAX_OBJECTS}): +{ACCEPTABLE_MEMORY_OVERHEAD_MB}MB")
    print(f"  Acceptable spawn time (@{N_MAX_OBJECTS}): <{ACCEPTABLE_SPAWN_TIME_S}s (TOTAL)")
    print(f"  Acceptable update time (@{N_MAX_OBJECTS}): <{ACCEPTABLE_UPDATE_TIME_MS}ms (TOTAL)")

    print("\n" + "=" * 70)
    print("Test A: Direct PyBullet (Baseline)")
    print("=" * 70)
    baseline = run_multiple_tests_isolated("direct", args.reps, args.n)
    print_stats("Direct PyBullet", baseline)

    print("\n" + "=" * 70)
    print("Test B: SimObject Wrapper")
    print("=" * 70)
    simobj = run_multiple_tests_isolated("simobject", args.reps, args.n)
    print_stats("SimObject Wrapper", simobj)

    print("\n" + "=" * 70)
    print("Test C: SimObjectManager (Bulk)")
    print("=" * 70)
    manager = run_multiple_tests_isolated("manager", args.reps, args.n)
    print_stats("SimObjectManager", manager)

    print("\n" + "=" * 70)
    print("Test D: Agent Wrapper")
    print("=" * 70)
    agent = run_multiple_tests_isolated("agent", args.reps, args.n)
    print_stats("Agent Wrapper", agent)

    print("\n" + "=" * 70)
    print("Test E: AgentManager (Bulk)")
    print("=" * 70)
    agent_mgr = run_multiple_tests_isolated("agent_manager", args.reps, args.n)
    print_stats("AgentManager", agent_mgr)

    # Overhead vs baseline (median)
    print("\n" + "=" * 70)
    print("Overhead Analysis - Median Values")
    print("=" * 70)

    spawn_over_s = simobj["spawn_time_s"]["median"] - baseline["spawn_time_s"]["median"]
    update_over_ms = simobj["update_time_ms"]["median"] - baseline["update_time_ms"]["median"]
    mem_over_mb = simobj["mem_rss_mb"]["median"] - baseline["mem_rss_mb"]["median"]

    cpu_spawn_over_s = simobj["cpu_spawn_s"]["median"] - baseline["cpu_spawn_s"]["median"]
    cpu_update_over_s = simobj["cpu_update_s"]["median"] - baseline["cpu_update_s"]["median"]

    agent_spawn_over_s = agent["spawn_time_s"]["median"] - baseline["spawn_time_s"]["median"]
    agent_update_over_ms = agent["update_time_ms"]["median"] - baseline["update_time_ms"]["median"]
    agent_mem_over_mb = agent["mem_rss_mb"]["median"] - baseline["mem_rss_mb"]["median"]

    print("\nSimObject vs Direct PyBullet:")
    print(f"  Spawn overhead (wall): +{spawn_over_s:.3f}s")
    print(f"  Spawn overhead (CPU):  +{cpu_spawn_over_s:.3f}s")
    print(f"  Update overhead (wall): +{update_over_ms:.2f}ms")
    print(f"  Update overhead (CPU):  +{cpu_update_over_s:.3f}s")
    mem_per_obj_kb = mem_over_mb / args.n * 1024
    print(f"  Memory overhead (RSS):  +{mem_over_mb:.2f}MB  ({mem_per_obj_kb:.2f}KB/obj)")

    print("\nAgent vs Direct PyBullet:")
    print(f"  Spawn overhead (wall): +{agent_spawn_over_s:.3f}s")
    print(f"  Update overhead (wall): +{agent_update_over_ms:.2f}ms")
    if "get_pose_time_ms" in agent:
        get_pose_overhead = agent["get_pose_time_ms"]["median"] - baseline["update_time_ms"]["median"]
        print(f"    ├─ get_pose overhead: +{get_pose_overhead:.2f}ms")
    if "set_pose_time_ms" in agent:
        print(f"    └─ set_pose overhead: +{agent['set_pose_time_ms']['median']:.2f}ms (vs 0 baseline)")
    agent_mem_per_obj_kb = agent_mem_over_mb / args.n * 1024
    print(f"  Memory overhead (RSS):  +{agent_mem_over_mb:.2f}MB  ({agent_mem_per_obj_kb:.2f}KB/obj)")

    # Extrapolate to production scale (reference)
    print("\n" + "=" * 70)
    print(f"Extrapolation (REFERENCE) to {N_MAX_OBJECTS} objects")
    print("=" * 70)
    scale = N_MAX_OBJECTS / args.n

    base_spawn_prod = baseline["spawn_time_s"]["median"] * scale
    sim_spawn_prod = simobj["spawn_time_s"]["median"] * scale
    agent_spawn_prod = agent["spawn_time_s"]["median"] * scale

    base_update_prod = baseline["update_time_ms"]["median"] * scale
    sim_update_prod = simobj["update_time_ms"]["median"] * scale
    agent_update_prod = agent["update_time_ms"]["median"] * scale

    mem_over_prod_sim = (simobj["mem_rss_mb"]["median"] - baseline["mem_rss_mb"]["median"]) * scale
    mem_over_prod_agent = (agent["mem_rss_mb"]["median"] - baseline["mem_rss_mb"]["median"]) * scale

    print("\nSpawn time (TOTAL):")
    print(f"  Baseline: {base_spawn_prod:.2f}s")
    print(f"  SimObject: {sim_spawn_prod:.2f}s")
    print(f"  Agent: {agent_spawn_prod:.2f}s")

    print("\nUpdate time (get+set pose, TOTAL per step):")
    print(f"  Baseline (get only): {base_update_prod:.2f}ms")
    print(f"  SimObject (get only): {sim_update_prod:.2f}ms")
    print(f"  Agent (get+set): {agent_update_prod:.2f}ms")
    if "get_pose_time_ms" in agent and "set_pose_time_ms" in agent:
        print(f"    ├─ Agent get_pose: {agent['get_pose_time_ms']['median'] * scale:.2f}ms")
        print(f"    └─ Agent set_pose: {agent['set_pose_time_ms']['median'] * scale:.2f}ms")

    print("\nMemory overhead (RSS delta, TOTAL):")
    print(f"  SimObject: +{mem_over_prod_sim:.2f}MB")
    print(f"  Agent: +{mem_over_prod_agent:.2f}MB")

    # Decision framework
    print("\n" + "=" * 70)
    print("Decision Framework (TOTAL @ production scale)")
    print("=" * 70)

    spawn_ok_sim = sim_spawn_prod <= ACCEPTABLE_SPAWN_TIME_S
    spawn_ok_agent = agent_spawn_prod <= ACCEPTABLE_SPAWN_TIME_S
    mem_ok_sim = mem_over_prod_sim <= ACCEPTABLE_MEMORY_OVERHEAD_MB
    mem_ok_agent = mem_over_prod_agent <= ACCEPTABLE_MEMORY_OVERHEAD_MB
    update_ok_sim = sim_update_prod <= ACCEPTABLE_UPDATE_TIME_MS
    update_ok_agent = agent_update_prod <= ACCEPTABLE_UPDATE_TIME_MS

    print("\nSimObject:")
    pass_fail_sim_spawn = "✅ PASS" if spawn_ok_sim else "❌ FAIL"
    print(f"  1) Spawn time:  {pass_fail_sim_spawn}  ({sim_spawn_prod:.2f}s <= {ACCEPTABLE_SPAWN_TIME_S}s)")
    pass_fail_sim_mem = "✅ PASS" if mem_ok_sim else "❌ FAIL"
    print(f"  2) Memory overhead: {pass_fail_sim_mem}  " f"(+{mem_over_prod_sim:.2f}MB <= +{ACCEPTABLE_MEMORY_OVERHEAD_MB}MB)")
    pass_fail_sim_update = "✅ PASS" if update_ok_sim else "❌ FAIL"
    print(f"  3) Update time: {pass_fail_sim_update}  " f"({sim_update_prod:.2f}ms <= {ACCEPTABLE_UPDATE_TIME_MS}ms)")

    print("\nAgent:")
    pass_fail_agent_spawn = "✅ PASS" if spawn_ok_agent else "❌ FAIL"
    print(f"  1) Spawn time:  {pass_fail_agent_spawn}  " f"({agent_spawn_prod:.2f}s <= {ACCEPTABLE_SPAWN_TIME_S}s)")
    pass_fail_agent_mem = "✅ PASS" if mem_ok_agent else "❌ FAIL"
    print(
        f"  2) Memory overhead: {pass_fail_agent_mem}  "
        f"(+{mem_over_prod_agent:.2f}MB <= +{ACCEPTABLE_MEMORY_OVERHEAD_MB}MB)"
    )
    pass_fail_agent_update = "✅ PASS" if update_ok_agent else "❌ FAIL"
    print(f"  3) Update time: {pass_fail_agent_update}  " f"({agent_update_prod:.2f}ms <= {ACCEPTABLE_UPDATE_TIME_MS}ms)")

    print("\n" + "=" * 70)


def main_child(args):
    fn = TEST_FUNCS[args.test]
    result = fn(args.n)
    sys.stdout.write(json.dumps(result) + "\n")
    sys.stdout.flush()


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--child", action="store_true")
    ap.add_argument("--test", choices=["direct", "simobject", "manager", "agent", "agent_manager"], default="direct")
    ap.add_argument("--n", type=int, default=NUM_TEST_OBJECTS)
    ap.add_argument("--reps", type=int, default=NUM_REPETITIONS)
    return ap.parse_args()


if __name__ == "__main__":
    args = parse_args()
    if args.child:
        main_child(args)
    else:
        main_parent(args)
