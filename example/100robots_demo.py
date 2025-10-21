"""
run_simulation_from_yaml.py
Read simulation parameters from YAML and run MultiRobotSimulationCore.
"""
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
import pybullet as p
import numpy as np
import time
import random
from core.core_simulation import MultiRobotSimulationCore, URDFObject
from core.tools import grid_execution
config_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../config', 'config.yaml'))
sim_core = MultiRobotSimulationCore.from_yaml(config_path)

# --- Original individual callback ---
def robot_movement_callback(robot, sim_core):
    if robot.meta_data['robot_type'] == "mobile_robot":
        pos, orn = robot.get_pose()
        euler = p.getEulerFromQuaternion(orn)
        yaw = euler[2]
        max_linear_speed = 2.0  # m/s (same as configurable_demo.py)
        max_angular_speed = 1.5  # rad/s
        forward_vel = np.random.uniform(-max_linear_speed, max_linear_speed)
        yaw_vel = np.random.uniform(-max_angular_speed, max_angular_speed)
        forward_x = forward_vel * np.cos(yaw)
        forward_y = forward_vel * np.sin(yaw)
        linear_vel = [forward_x, forward_y, 0]
        angular_vel = [0, 0, yaw_vel]
        corrected_orn = p.getQuaternionFromEuler([0, 0, yaw])
        robot.kinematic_teleport_base([pos[0], pos[1], 0.3], corrected_orn, linear_vel, angular_vel)
    elif robot.meta_data['robot_type'] == "arm_robot":
        num_joints = p.getNumJoints(robot.body_id)
        for j in range(num_joints):
            info = p.getJointInfo(robot.body_id, j)
            joint_type = info[2]
            if joint_type == p.JOINT_REVOLUTE:
                lower, upper = info[8:10]
                if lower < upper:
                    target_pos = np.random.uniform(lower, upper)
                else:
                    target_pos = np.random.uniform(-1.0, 1.0)
                robot.set_joint_target(j, target_pos)

# --- Batch callback for all robots (for reference) ---
def batch_robot_movement_callback(robots, sim_core, dt):
    for robot in robots:
        robot_movement_callback(robot, sim_core)

num_robots = 100
robot_types = ["arm_robot", "mobile_robot"]
grid_size = int(np.ceil(np.sqrt(num_robots)))
spacing = 2.0
selected_types = [random.choice(robot_types) for _ in range(num_robots)]

# URDFObject generation with grid_execution

def spawn_robot(ix, iy, iz, x, y, z, **kwargs):
    i = ix * grid_size + iy
    if i >= num_robots:
        return
    robot_type = selected_types[i]
    urdf_path = os.path.abspath(os.path.join(os.path.dirname(__file__), f"../robots/{robot_type}.urdf"))
    useFixedBase = True if robot_type == "arm_robot" else False
    urdf_obj = URDFObject.from_urdf(
        urdf_path=urdf_path,
        position=[x, y, z if robot_type == "mobile_robot" else 0.0],
        orientation=[0, 0, 0, 1],
        useFixedBase=useFixedBase,
        set_mass_zero=True,
        meta_data={'robot_type': robot_type}
    )
    sim_core.robots.append(urdf_obj)

grid_execution(
    x_num=grid_size,
    y_num=grid_size,
    z_num=1,
    spacing_x=spacing,
    spacing_y=spacing,
    spacing_z=0.0,
    offset_x=0.0,
    offset_y=0.0,
    offset_z=0.0,
    func=spawn_robot,
    args=None
)

# Register batch callback for all robots (for reference)
sim_core.register_callback(batch_robot_movement_callback, 4)

sim_core.run_simulation()