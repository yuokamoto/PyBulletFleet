#!/usr/bin/env python3
"""
Test if p.getAABB() is the bottleneck.
"""
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pybullet as p
import pybullet_data
from pybullet_fleet.agent_manager import AgentManager, GridSpawnParams
from pybullet_fleet.agent import AgentSpawnParams, MotionMode
from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
import math

# Setup
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf")

num_agents = 1000
grid_size = int(math.ceil(math.sqrt(num_agents)))

# Spawn agents manually
robot_urdf = os.path.join(os.path.dirname(__file__), "../../robots/simple_cube.urdf")
body_ids = []

print(f"Spawning {num_agents} agents...")
for i in range(num_agents):
    x = (i % grid_size) * 1.0
    y = (i // grid_size) * 1.0
    z = 0.1
    body_id = p.loadURDF(robot_urdf, [x, y, z])
    body_ids.append(body_id)

print(f"Spawned {len(body_ids)} agents")

# Test getAABB performance
iterations = 100

print(f"\nTesting p.getAABB() performance ({iterations} iterations)...")

t0 = time.perf_counter()
for _ in range(iterations):
    aabbs = [p.getAABB(bid) for bid in body_ids]
t1 = time.perf_counter()

avg_ms = (t1 - t0) / iterations * 1000
print(f"  Average time: {avg_ms:.3f}ms")
print(f"  Per-robot: {avg_ms/num_agents*1000:.3f}µs")

p.disconnect()
