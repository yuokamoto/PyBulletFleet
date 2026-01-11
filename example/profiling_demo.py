#!/usr/bin/env python3
"""
profiling_demo.py
Demonstration of the built-in profiling features in core_simulation.py

Shows:
1. How to enable profiling
2. How to configure profiling log frequency
3. Example profiling output with collision details
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.agent import Agent
import logging

# Set log level to DEBUG to see profiling output
logging.basicConfig(level=logging.DEBUG)

def main():
    # Enable profiling in simulation parameters
    params = SimulationParams(
        gui=True,
        speed=1.0,
        timestep=1.0/240.0,
        duration=5.0,  # 5 seconds
        enable_profiling=True,  # Enable profiling
        collision_check_frequency=None,  # Check every step
        log_level="debug"
    )
    
    sim_core = MultiRobotSimulationCore(params)
    
    # Configure profiling log frequency
    # Default is 10 (log every 10 steps)
    # Try different values:
    # - 1: Every step (detailed but high overhead)
    # - 10: Every 10 steps (recommended)
    # - 100: Every 100 steps (minimal overhead)
    sim_core.set_profiling_log_frequency(10)
    
    # Spawn some agents to create interesting profiling data
    print("\n=== Spawning 50 agents ===")
    for i in range(50):
        x = (i % 10) * 2.0
        y = (i // 10) * 2.0
        agent = Agent(
            sim_core=sim_core,
            initial_position=[x, y, 0.5],
            urdf_path="robots/mobile_robot.urdf"
        )
        # Set simple circular movement
        agent.set_goal([x + 1.0, y + 1.0, 0.5])
    
    print("\n=== Starting simulation with profiling ===")
    print("Watch the console for [PROFILE] logs every 10 steps")
    print("\nExample profiling output:")
    print("[PROFILE] step 10: Agent.update=0.52ms, Callbacks=0.01ms, stepSimulation=1.23ms, "
          "Collisions=2.45ms [GetAABBs=0.12ms, SpatialHash=0.34ms, AABBFilter=1.56ms, ContactPts=0.43ms], "
          "Monitor=0.08ms, Total=4.29ms")
    print("\nThis shows:")
    print("- Agent.update: Time to update all agents (movement logic)")
    print("- Collisions: Total collision check time")
    print("  - GetAABBs: Getting bounding boxes from PyBullet")
    print("  - SpatialHash: Building spatial grid")
    print("  - AABBFilter: Broad-phase filtering (often the bottleneck)")
    print("  - ContactPts: Narrow-phase contact point checks")
    print("- Total: End-to-end time for one simulation step")
    print("\n" + "="*80 + "\n")
    
    sim_core.run_simulation()
    
    print("\n=== Simulation complete ===")
    print("Review the [PROFILE] logs above to see performance breakdown")

if __name__ == "__main__":
    main()
