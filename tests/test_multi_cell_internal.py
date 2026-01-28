"""
Direct test of multi-cell registration internals.
"""

import logging
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from pybullet_fleet.core_simulation import MultiRobotSimulationCore, SimulationParams
from pybullet_fleet.agent_manager import AgentManager, GridSpawnParams
from pybullet_fleet.agent import AgentSpawnParams, Pose
from pybullet_fleet.types import CollisionMode, SpatialHashCellSizeMode

logging.basicConfig(level=logging.INFO, format="%(message)s")
logger = logging.getLogger(__name__)


def main():
    print("=" * 80)
    print("Multi-Cell Registration Internal Test")
    print("=" * 80)
    
    # Create simulation
    params = SimulationParams(
        gui=False,
        physics=False,
        monitor=False,
        spatial_hash_cell_size_mode=SpatialHashCellSizeMode.CONSTANT,
        spatial_hash_cell_size=1.0,  # 1m cells
        multi_cell_threshold=1.5,  # Objects > 1.5m use multi-cell
        log_level="warning",
    )
    
    sim_core = MultiRobotSimulationCore(params)
    sim_core.set_collision_spatial_hash_cell_size_mode()
    
    print(f"\nConfiguration:")
    print(f"  Cell size: {sim_core._cached_cell_size}m")
    print(f"  Threshold: {params.multi_cell_threshold}x = {sim_core._cached_cell_size * params.multi_cell_threshold}m")
    
    # Create agent manager and spawn robots
    agent_manager = AgentManager(sim_core=sim_core)
    
    grid_params = GridSpawnParams(
        x_min=0, x_max=1, y_min=0, y_max=1,
        spacing=[3.0, 3.0, 0.0],
        offset=[0.0, 0.0, 0.0],
    )
    
    spawn_params = AgentSpawnParams(
        urdf_path="robots/mobile_robot.urdf",
        collision_mode=CollisionMode.NORMAL_3D,
    )
    
    print(f"\nSpawning 4 robots...")
    agents = agent_manager.spawn_agents_grid(
        spawn_params=spawn_params,
        grid_params=grid_params,
        num_agents=4,
    )
    
    print(f"Spawned {len(agents)} robots")
    
    # Analyze cell registrations
    print(f"\n--- Cell Registration Analysis ---")
    print(f"Total objects in simulation: {len(sim_core._sim_objects_dict)}")
    print(f"Objects with cell mapping: {len(sim_core._cached_object_to_cell)}")
    print(f"Total grid cells used: {len(sim_core._cached_spatial_grid)}")
    
    # Check each robot
    print(f"\nRobot cell registrations:")
    for i, agent in enumerate(agents):
        body_id = agent.body_id
        cells = sim_core._cached_object_to_cell.get(body_id, [])
        aabb = sim_core._cached_aabbs_dict.get(body_id)
        
        if aabb:
            extent_x = aabb[1][0] - aabb[0][0]
            extent_y = aabb[1][1] - aabb[0][1]
            max_extent = max(extent_x, extent_y)
            should_multi = sim_core._should_use_multi_cell_registration(body_id)
        else:
            max_extent = 0
            should_multi = False
        
        print(f"  Robot {i} (ID={body_id}):")
        print(f"    AABB extent: {max_extent:.2f}m")
        print(f"    Should use multi-cell: {should_multi}")
        print(f"    Registered to {len(cells)} cell(s): {cells[:3]}{'...' if len(cells) > 3 else ''}")
    
    # Distribution
    print(f"\n--- Distribution ---")
    cell_count_dist = {}
    for obj_id, cells in sim_core._cached_object_to_cell.items():
        count = len(cells)
        cell_count_dist[count] = cell_count_dist.get(count, 0) + 1
    
    for num_cells in sorted(cell_count_dist.keys()):
        count = cell_count_dist[num_cells]
        print(f"  {count} object(s) registered to {num_cells} cell(s)")
    
    # Test _should_use_multi_cell_registration and _get_overlapping_cells
    print(f"\n--- Method Testing ---")
    if agents:
        test_body_id = agents[0].body_id
        should_use = sim_core._should_use_multi_cell_registration(test_body_id)
        overlapping = sim_core._get_overlapping_cells(test_body_id)
        
        print(f"Test object ID: {test_body_id}")
        print(f"  _should_use_multi_cell_registration(): {should_use}")
        print(f"  _get_overlapping_cells(): {len(overlapping)} cells")
        if overlapping:
            print(f"    First few cells: {overlapping[:5]}")
    
    print(f"\n✓ Test completed")
    print("=" * 80)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        import traceback
        traceback.print_exc()
        sys.exit(1)
