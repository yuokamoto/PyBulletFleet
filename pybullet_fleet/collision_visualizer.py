"""
collision_visualizer.py
Utilities for visualizing collision shapes in PyBullet.
"""
import pybullet as p
import math
import logging

logger = logging.getLogger(__name__)


class CollisionVisualizer:
    """
    Handles visualization of collision shapes as debug wireframes.
    Independent from visual shape rendering.
    Optimized for fast toggle by caching debug items.
    """
    
    def __init__(self):
        self.debug_items = []
        self.visible = False
        self._cached = False  # Track if debug items are cached
    
    def set_visible(self, visible):
        """
        Show or hide collision shapes (optimized toggle).
        
        Args:
            visible: True to show collision shapes, False to hide them
        """
        if visible == self.visible:
            return  # No change needed
        
        if not visible:
            # Hide: Remove all debug items
            self.clear()
            self.visible = False
            logger.info("Collision shapes disabled")
            return
        
        # Show: Draw collision shapes (only if not cached)
        if not self._cached:
            self.draw_all_collision_shapes()
            self._cached = True
        else:
            # Items already exist, just recreate them (fast)
            self._redraw_cached_items()
        
        self.visible = True
        logger.info(f"Collision shapes enabled ({len(self.debug_items)} debug items)")
    
    def clear(self):
        """Remove all collision shape debug items."""
        for item_id in self.debug_items:
            try:
                p.removeUserDebugItem(item_id)
            except:
                pass
        self.debug_items = []
        self._cached = False
    
    def _redraw_cached_items(self):
        """
        Redraw collision shapes using cached data.
        NOTE: This is a simplified placeholder - in practice, we need to store
        line endpoints and redraw them. For now, we just redraw everything.
        """
        # For fast toggle, we simply redraw (this is still faster than creating geometries)
        self.draw_all_collision_shapes()
    
    def draw_all_collision_shapes(self, simplified=True, max_objects=1000):
        """
        Draw collision shapes for all bodies as debug wireframes.
        
        Args:
            simplified: If True, only draw AABB boxes (much faster).
                       If False, draw detailed wireframes for each shape type.
            max_objects: Maximum number of objects to visualize (for performance).
                        If num_bodies > max_objects, sample every N-th object.
        """
        self.clear()
        
        num_bodies = p.getNumBodies()
        
        # Calculate sampling rate
        if num_bodies > max_objects:
            sample_rate = num_bodies // max_objects
            logger.info(f"Sampling collision shapes: showing 1 of every {sample_rate} objects")
        else:
            sample_rate = 1
        
        count = 0
        for body_id in range(0, num_bodies, sample_rate):
            try:
                if simplified:
                    self.draw_body_aabb(body_id)
                else:
                    self.draw_body_collision_shapes(body_id)
                count += 1
            except Exception as e:
                # Skip bodies without collision shapes or with errors
                pass
        
        logger.info(f"Drew collision shapes for {count} objects (simplified={simplified})")
    
    def draw_body_aabb(self, body_id, color=[0, 1, 0], base_only=True):
        """
        Draw only AABB for a body (fast, simplified visualization).
        
        Args:
            body_id: PyBullet body ID
            color: RGB color for wireframes (default: green)
            base_only: If True, only draw AABB for base link (much faster)
        """
        if base_only:
            # Only draw base link AABB (fastest)
            try:
                aabb_min, aabb_max = p.getAABB(body_id, -1)
                self._draw_aabb(aabb_min, aabb_max, color, simplified=True)
            except:
                pass
        else:
            # Draw AABB for all links
            num_joints = p.getNumJoints(body_id)
            links_to_check = [-1] + list(range(num_joints))
            
            for link_index in links_to_check:
                try:
                    aabb_min, aabb_max = p.getAABB(body_id, link_index)
                    self._draw_aabb(aabb_min, aabb_max, color, simplified=True)
                except:
                    pass  # Skip links without AABB
    
    def draw_body_collision_shapes(self, body_id, color=[0, 1, 0]):
        """
        Draw collision shapes for a specific body.
        
        Args:
            body_id: PyBullet body ID
            color: RGB color for wireframes (default: green)
        """
        # Check each link (including base link at index -1)
        num_joints = p.getNumJoints(body_id)
        links_to_check = [-1] + list(range(num_joints))
        
        for link_index in links_to_check:
            # Get collision shape info
            collision_data = p.getCollisionShapeData(body_id, link_index)
            
            for shape_info in collision_data:
                geom_type = shape_info[2]  # Geometry type
                dimensions = shape_info[3]  # Dimensions
                local_frame_pos = shape_info[5]  # Local frame position
                local_frame_orn = shape_info[6]  # Local frame orientation
                
                # Get link world transform
                if link_index == -1:
                    link_pos, link_orn = p.getBasePositionAndOrientation(body_id)
                else:
                    link_state = p.getLinkState(body_id, link_index)
                    link_pos = link_state[0]
                    link_orn = link_state[1]
                
                # Transform local collision shape to world coordinates
                world_pos, world_orn = p.multiplyTransforms(
                    link_pos, link_orn,
                    local_frame_pos, local_frame_orn
                )
                
                # Draw collision shape based on type
                if geom_type == p.GEOM_BOX:
                    self._draw_box(world_pos, world_orn, dimensions, color)
                elif geom_type == p.GEOM_SPHERE:
                    self._draw_sphere(world_pos, dimensions[0], color)
                elif geom_type == p.GEOM_CYLINDER:
                    self._draw_cylinder(world_pos, world_orn, dimensions[1], dimensions[0], color)
                elif geom_type == p.GEOM_MESH:
                    # For mesh, draw AABB as approximation
                    aabb_min, aabb_max = p.getAABB(body_id, link_index)
                    self._draw_aabb(aabb_min, aabb_max, color)
    
    def _draw_box(self, pos, orn, half_extents, color):
        """Draw a box wireframe."""
        # Get rotation matrix from quaternion
        rot_mat = p.getMatrixFromQuaternion(orn)
        
        # Box corners in local frame
        corners = []
        for i in [-1, 1]:
            for j in [-1, 1]:
                for k in [-1, 1]:
                    corner = [i * half_extents[0], j * half_extents[1], k * half_extents[2]]
                    # Transform to world frame
                    world_corner = [
                        pos[0] + rot_mat[0] * corner[0] + rot_mat[1] * corner[1] + rot_mat[2] * corner[2],
                        pos[1] + rot_mat[3] * corner[0] + rot_mat[4] * corner[1] + rot_mat[5] * corner[2],
                        pos[2] + rot_mat[6] * corner[0] + rot_mat[7] * corner[1] + rot_mat[8] * corner[2]
                    ]
                    corners.append(world_corner)
        
        # Draw 12 edges of the box
        edges = [
            (0, 1), (2, 3), (4, 5), (6, 7),  # edges parallel to x
            (0, 2), (1, 3), (4, 6), (5, 7),  # edges parallel to y
            (0, 4), (1, 5), (2, 6), (3, 7)   # edges parallel to z
        ]
        
        for edge in edges:
            line_id = p.addUserDebugLine(corners[edge[0]], corners[edge[1]], color, lineWidth=2, lifeTime=0)
            self.debug_items.append(line_id)
    
    def _draw_sphere(self, pos, radius, color):
        """Draw a sphere wireframe."""
        # Draw 3 circles (XY, YZ, XZ planes)
        segments = 16
        
        for axis in range(3):
            for i in range(segments):
                angle1 = 2 * math.pi * i / segments
                angle2 = 2 * math.pi * (i + 1) / segments
                
                if axis == 0:  # XY plane
                    p1 = [pos[0] + radius * math.cos(angle1), pos[1] + radius * math.sin(angle1), pos[2]]
                    p2 = [pos[0] + radius * math.cos(angle2), pos[1] + radius * math.sin(angle2), pos[2]]
                elif axis == 1:  # YZ plane
                    p1 = [pos[0], pos[1] + radius * math.cos(angle1), pos[2] + radius * math.sin(angle1)]
                    p2 = [pos[0], pos[1] + radius * math.cos(angle2), pos[2] + radius * math.sin(angle2)]
                else:  # XZ plane
                    p1 = [pos[0] + radius * math.cos(angle1), pos[1], pos[2] + radius * math.sin(angle1)]
                    p2 = [pos[0] + radius * math.cos(angle2), pos[1], pos[2] + radius * math.sin(angle2)]
                
                line_id = p.addUserDebugLine(p1, p2, color, lineWidth=2, lifeTime=0)
                self.debug_items.append(line_id)
    
    def _draw_cylinder(self, pos, orn, radius, height, color):
        """Draw a cylinder wireframe."""
        segments = 16
        
        # Draw top and bottom circles, and vertical lines
        for i in range(segments):
            angle1 = 2 * math.pi * i / segments
            angle2 = 2 * math.pi * (i + 1) / segments
            
            # Top circle
            p1_top = [radius * math.cos(angle1), radius * math.sin(angle1), height / 2]
            p2_top = [radius * math.cos(angle2), radius * math.sin(angle2), height / 2]
            
            # Bottom circle
            p1_bot = [radius * math.cos(angle1), radius * math.sin(angle1), -height / 2]
            p2_bot = [radius * math.cos(angle2), radius * math.sin(angle2), -height / 2]
            
            # Transform to world frame
            p1_top_world, _ = p.multiplyTransforms(pos, orn, p1_top, [0, 0, 0, 1])
            p2_top_world, _ = p.multiplyTransforms(pos, orn, p2_top, [0, 0, 0, 1])
            p1_bot_world, _ = p.multiplyTransforms(pos, orn, p1_bot, [0, 0, 0, 1])
            p2_bot_world, _ = p.multiplyTransforms(pos, orn, p2_bot, [0, 0, 0, 1])
            
            # Draw circles
            line_id = p.addUserDebugLine(p1_top_world, p2_top_world, color, lineWidth=2, lifeTime=0)
            self.debug_items.append(line_id)
            line_id = p.addUserDebugLine(p1_bot_world, p2_bot_world, color, lineWidth=2, lifeTime=0)
            self.debug_items.append(line_id)
            
            # Draw vertical lines (every 4th segment)
            if i % 4 == 0:
                line_id = p.addUserDebugLine(p1_top_world, p1_bot_world, color, lineWidth=2, lifeTime=0)
                self.debug_items.append(line_id)
    
    def _draw_aabb(self, aabb_min, aabb_max, color, simplified=True):
        """
        Draw an AABB (axis-aligned bounding box) wireframe.
        
        Args:
            simplified: If True, draw only 4 edges (outline). If False, draw all 12 edges.
        """
        # 8 corners of AABB
        corners = [
            [aabb_min[0], aabb_min[1], aabb_min[2]],
            [aabb_max[0], aabb_min[1], aabb_min[2]],
            [aabb_max[0], aabb_max[1], aabb_min[2]],
            [aabb_min[0], aabb_max[1], aabb_min[2]],
            [aabb_min[0], aabb_min[1], aabb_max[2]],
            [aabb_max[0], aabb_min[1], aabb_max[2]],
            [aabb_max[0], aabb_max[1], aabb_max[2]],
            [aabb_min[0], aabb_max[1], aabb_max[2]]
        ]
        
        if simplified:
            # Draw complete box: bottom + top + vertical edges (12 edges total)
            edges = [
                (0, 1), (1, 2), (2, 3), (3, 0),  # bottom face (xy plane)
                (4, 5), (5, 6), (6, 7), (7, 4),  # top face (xy plane)
                (0, 4), (1, 5), (2, 6), (3, 7)   # vertical edges (z direction)
            ]
        else:
            # Draw all 12 edges (same as simplified now)
            edges = [
                (0, 1), (1, 2), (2, 3), (3, 0),  # bottom face
                (4, 5), (5, 6), (6, 7), (7, 4),  # top face
                (0, 4), (1, 5), (2, 6), (3, 7)   # vertical edges
            ]
        
        for edge in edges:
            line_id = p.addUserDebugLine(corners[edge[0]], corners[edge[1]], color, lineWidth=1, lifeTime=0)
            self.debug_items.append(line_id)
