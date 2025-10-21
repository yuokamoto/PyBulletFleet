import numpy as np
import random
import pybullet as p

from core.core_simulation import URDFObject, MeshObject

def world_to_grid(pos, spacing, offset=None):
    """
    Convert world coordinates to grid indices (always 3D), with offset.
    pos: [x, y, z] or [x, y]
    spacing: [spacing_x, spacing_y, spacing_z] or [spacing_x, spacing_y]
    offset: [offset_x, offset_y, offset_z] or [offset_x, offset_y] or None
    Returns: (ix, iy, iz)
    """
    x = pos[0]
    y = pos[1]
    z = pos[2] if len(pos) > 2 else 0.0
    spacing_x = spacing[0]
    spacing_y = spacing[1]
    spacing_z = spacing[2] if len(spacing) > 2 else 1.0
    offset_x = offset[0] if offset and len(offset) > 0 else 0.0
    offset_y = offset[1] if offset and len(offset) > 1 else 0.0
    offset_z = offset[2] if offset and len(offset) > 2 else 0.0
    grid_ix = int(round((x - offset_x) / spacing_x))
    grid_iy = int(round((y - offset_y) / spacing_y))
    grid_iz = int(round((z - offset_z) / spacing_z))
    return [grid_ix, grid_iy, grid_iz]

def grid_to_world(grid, spacing, offset=None):
    """
    Convert grid indices to world coordinates (always 3D), with offset.
    grid: [ix, iy, iz] or [ix, iy]
    spacing: [spacing_x, spacing_y, spacing_z] or [spacing_x, spacing_y]
    offset: [offset_x, offset_y, offset_z] or [offset_x, offset_y] or None
    Returns: [x, y, z]
    """
    ix = grid[0]
    iy = grid[1]
    iz = grid[2] if len(grid) > 2 else 0
    spacing_x = spacing[0]
    spacing_y = spacing[1]
    spacing_z = spacing[2] if len(spacing) > 2 else 1.0
    offset_x = offset[0] if offset and len(offset) > 0 else 0.0
    offset_y = offset[1] if offset and len(offset) > 1 else 0.0
    offset_z = offset[2] if offset and len(offset) > 2 else 0.0
    x = ix * spacing_x + offset_x
    y = iy * spacing_y + offset_y
    z = iz * spacing_z + offset_z
    return [x, y, z]

def world_to_grid_2d(pos, spacing):
    """
    2D wrapper for world_to_grid
    pos: [x, y]
    spacing: [spacing_x, spacing_y]
    Returns: (ix, iy)
    """
    return world_to_grid(pos[:2], spacing[:2])

def grid_to_world_2d(grid, spacing):
    """
    2D wrapper for grid_to_world
    grid: [ix, iy]
    spacing: [spacing_x, spacing_y]
    Returns: [x, y]
    """
    return grid_to_world(grid[:2], spacing[:2])

def grid_execution(x_num=1, y_num=1, z_num=1,
                   spacing_x=1.0, spacing_y=1.0, spacing_z=1.0,
                   offset_x=0.0, offset_y=0.0, offset_z=0.0,
                   func=None, args=None):
    """
    Calls the callback function func(ix, iy, iz, x, y, z) at each grid point with grid coordinates and physical coordinates.
    You can freely describe generation, parameter calculation, conditional branching, etc.
    """
    if func is None:
        raise ValueError("Please specify func (callback function)")
    spacing = [spacing_x, spacing_y, spacing_z]
    offset = [offset_x, offset_y, offset_z]
    for ix in range(x_num):
        for iy in range(y_num):
            for iz in range(z_num):
                grid = [ix, iy, iz]
                x, y, z = grid_to_world(grid, spacing, offset)
                call_args = dict(ix=ix, iy=iy, iz=iz, x=x, y=y, z=z)
                if args:
                    call_args.update(args)
                func(**call_args)

def grid_spawn(model_class, model_paths, sim_core, x_num=1, y_num=1, z_num=1,
               spacing_x=1.0, spacing_y=1.0, spacing_z=1.0,
               offset_x=0.0, offset_y=0.0, offset_z=0.0,
               spawn_probability=1.0, orientation_euler=[0,0,0], extra_args=None, max_spawn=None):
    """
    model_class: Class such as URDFObject, MeshObject
    model_paths: List of model file paths (can be just one)
    sim_core: MultiRobotSimulationCore instance
    x_num, y_num, z_num: Number of grids
    spacing_x, spacing_y, spacing_z: Spacing for each axis
    offset_x, offset_y, offset_z: Offset for each axis
    spawn_probability: Probability of spawning (0.0~1.0, default=1.0)
    orientation_euler: [roll, pitch, yaw] (default [0,0,0])
    extra_args: dict for additional parameters (specific to URDFObject/MeshObject)
    max_spawn: Maximum number of objects to spawn (None for unlimited)
    """
    if extra_args is None:
        extra_args = {}
    spawned_objects = []
    total_grids = x_num * y_num * z_num
    spawn_count = [0]  # mutable for closure

    def spawn_func(ix, iy, iz, x, y, z, **args):
        # 1) If max_spawn is set and reached, do not spawn further
        if max_spawn is not None and spawn_count[0] >= max_spawn:
            return
        # 2) If remaining grids == remaining spawn, always spawn
        remaining_grids = total_grids - (ix * y_num * z_num + iy * z_num + iz)
        remaining_spawn = max_spawn - spawn_count[0] if max_spawn is not None else None
        force_spawn = False
        if max_spawn is not None:
            if remaining_spawn is not None and remaining_grids <= remaining_spawn:
                force_spawn = True
        # 3) If max_spawn > total_grids, warning and spawn in all grids
        if max_spawn is not None and max_spawn > total_grids:
            import warnings
            warnings.warn(f"max_spawn ({max_spawn}) is greater than grid count ({total_grids}). Spawning in all grids.")
            force_spawn = True
        # random spawn or forced spawn
        if force_spawn or random.random() <= spawn_probability:
            model_path = random.choice(model_paths)
            orientation = None
            if orientation_euler is not None:
                orientation = p.getQuaternionFromEuler(orientation_euler)
            obj_args = dict(
                position=[x, y, z],
                orientation=orientation
            )
            obj_args.update(args)
            obj_args['sim_core'] = sim_core
            # クラスか関数かで分岐
            if hasattr(model_class, '__name__') and model_class.__name__ == 'MeshObject':
                obj_args['mesh_path'] = model_path
                obj = model_class.from_mesh(**obj_args)
                sim_core.mesh_objects.append(obj)
            elif hasattr(model_class, '__name__') and model_class.__name__ == 'URDFObject':
                obj_args['urdf_path'] = model_path
                obj = model_class.from_urdf(**obj_args)
                sim_core.robots.append(obj)
            else:
                # factory関数（高速化用）
                obj = model_class(position=[x, y, z], orientation=orientation, sim_core=sim_core)
                sim_core.mesh_objects.append(obj)
            spawned_objects.append(obj)
            spawn_count[0] += 1
    grid_execution(x_num=x_num, y_num=y_num, z_num=z_num,
                  spacing_x=spacing_x, spacing_y=spacing_y, spacing_z=spacing_z,
                  offset_x=offset_x, offset_y=offset_y, offset_z=offset_z,
                  func=spawn_func, args=extra_args)
    return spawned_objects


def grid_spawn_urdf(model_paths, sim_core, x_num=1, y_num=1, z_num=1,
                    spacing_x=1.0, spacing_y=1.0, spacing_z=1.0,
                    offset_x=0.0, offset_y=0.0, offset_z=0.0,
                    spawn_probability=1.0, orientation_euler=[0,0,0],
                    useFixedBase=False, set_mass_zero=False, meta_data=None, max_spawn=None):
    extra_args = {
        "useFixedBase": useFixedBase,
        "set_mass_zero": set_mass_zero
    }
    if meta_data is not None:
        extra_args["meta_data"] = meta_data
    return grid_spawn(
        model_class=URDFObject,
        model_paths=model_paths,
        sim_core=sim_core,
        x_num=x_num,
        y_num=y_num,
        z_num=z_num,
        spacing_x=spacing_x,
        spacing_y=spacing_y,
        spacing_z=spacing_z,
        offset_x=offset_x,
        offset_y=offset_y,
        offset_z=offset_z,
        spawn_probability=spawn_probability,
        orientation_euler=orientation_euler,
        extra_args=extra_args,
        max_spawn=max_spawn
    )

def grid_spawn_mesh(model_paths, sim_core, x_num=1, y_num=1, z_num=1,
                    spacing_x=1.0, spacing_y=1.0, spacing_z=1.0,
                    offset_x=0.0, offset_y=0.0, offset_z=0.0,
                    spawn_probability=1.0, orientation_euler=[0,0,0],
                    base_mass=0.0, mesh_scale=[1,1,1], rgbaColor=[1,1,1,1], max_spawn=None):
    # 1回だけvisual/collision shapeを作成
    mesh_path = model_paths[0] if isinstance(model_paths, list) else model_paths
    vis_id = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        fileName=mesh_path,
        meshScale=mesh_scale,
        rgbaColor=rgbaColor
    )
    col_id = p.createCollisionShape(
        shapeType=p.GEOM_MESH,
        fileName=mesh_path,
        meshScale=mesh_scale
    )
    def mesh_object_factory(position, orientation, sim_core=None, **kwargs):
        body_id = p.createMultiBody(
            baseMass=base_mass,
            baseCollisionShapeIndex=col_id,
            baseVisualShapeIndex=vis_id,
            basePosition=position,
            baseOrientation=orientation
        )
        return MeshObject(body_id=body_id, mesh_path=mesh_path, visual_id=vis_id, collision_id=col_id, sim_core=sim_core)
    # grid_spawnのmodel_class引数にfactory関数を渡す
    return grid_spawn(
        model_class=mesh_object_factory,
        model_paths=[mesh_path],
        sim_core=sim_core,
        x_num=x_num,
        y_num=y_num,
        z_num=z_num,
        spacing_x=spacing_x,
        spacing_y=spacing_y,
        spacing_z=spacing_z,
        offset_x=offset_x,
        offset_y=offset_y,
        offset_z=offset_z,
        spawn_probability=spawn_probability,
        orientation_euler=orientation_euler,
        extra_args={},
        max_spawn=max_spawn
    )
