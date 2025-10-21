
"""
core_simulation.py
Reusable core simulation logic for multi-robot PyBullet environments.
Integrates generation, management, transport, attach/detach, collision detection, coordinate conversion, occupied judgment, transport path generation, debugging, monitoring, and log control for various robots, pallets, and meshes.
"""

# --- All imports at the top for PEP8 compliance ---
import pybullet as p
import numpy as np
import time
import json
import yaml
import logging
import os
from core.data_monitor import DataMonitor

# --- Unified log level management ---
def get_log_level(level_str):
    level_map = {
        'debug': logging.DEBUG,
        'info': logging.INFO,
        'warn': logging.WARNING,
        'warning': logging.WARNING,
        'error': logging.ERROR,
        'critical': logging.CRITICAL
    }
    return level_map.get(str(level_str).lower(), logging.INFO)

# グローバルlog_level（初期値: 'info'）
GLOBAL_LOG_LEVEL = 'info'
if 'PYBULLET_LOG_LEVEL' in os.environ:
    GLOBAL_LOG_LEVEL = os.environ['PYBULLET_LOG_LEVEL']
logging.basicConfig(level=get_log_level(GLOBAL_LOG_LEVEL), format='%(asctime)s %(levelname)s %(message)s')


# ログレベル管理クラス
class LogLevelManager:
    @staticmethod
    def set_global_log_level(level_str):
        global GLOBAL_LOG_LEVEL
        GLOBAL_LOG_LEVEL = level_str
        logging.getLogger().setLevel(get_log_level(level_str))

    @staticmethod
    def set_log_level_from_params(params):
        level_str = getattr(params, 'log_level', GLOBAL_LOG_LEVEL)
        LogLevelManager.set_global_log_level(level_str)


class SimObject:
    def __init__(self, body_id, sim_core=None):
        self.body_id = body_id
        self.callbacks = []
        self.sim_core = sim_core
        self.attached_objects = []

    def get_pose(self):
        """現在の位置・姿勢（pos, orn）を返す"""
        return p.getBasePositionAndOrientation(self.body_id)

    def kinematic_teleport_base(self, position, orientation, linear_vel=None, angular_vel=None):
        p.resetBasePositionAndOrientation(self.body_id, position, orientation)
        if linear_vel is not None and angular_vel is not None:
            p.resetBaseVelocity(self.body_id, linear_vel, angular_vel)
        # 再帰的にattached_objectsにも同じ座標・速度を適用
        for obj in getattr(self, 'attached_objects', []):
            # attach時の相対位置・姿勢を使って追従
            if hasattr(obj, '_attach_offset'):
                offset_pos, offset_orn = obj._attach_offset
                new_pos, new_orn = p.multiplyTransforms(position, orientation, offset_pos, offset_orn)
                obj.kinematic_teleport_base(new_pos, new_orn, linear_vel, angular_vel)
            else:
                obj.kinematic_teleport_base(position, orientation, linear_vel, angular_vel)
    def attach_object(self, obj, parentFramePosition=[0,0,0], childFramePosition=[0,0,0], jointAxis=[0,0,0], jointType=p.JOINT_FIXED, parentLinkIndex=-1, childLinkIndex=-1):
        if obj not in self.attached_objects:
            self.attached_objects.append(obj)
            # 初期相対位置・姿勢（carrier座標系でのpalletの位置・姿勢）を保存
            parent_pos, parent_orn = self.get_pose()
            child_pos, child_orn = obj.get_pose()
            rel_pos, rel_orn = p.invertTransform(parent_pos, parent_orn)
            offset_pos, offset_orn = p.multiplyTransforms(rel_pos, rel_orn, child_pos, child_orn)
            obj._attach_offset = (offset_pos, offset_orn)
            mass = p.getDynamicsInfo(obj.body_id, -1)[0]
            if mass != 0:
                obj._constraint_id = p.createConstraint(
                    parentBodyUniqueId=self.body_id,
                    parentLinkIndex=parentLinkIndex,
                    childBodyUniqueId=obj.body_id,
                    childLinkIndex=childLinkIndex,
                    jointType=jointType,
                    jointAxis=jointAxis,
                    parentFramePosition=parentFramePosition,
                    childFramePosition=childFramePosition
                )
    def detach_object(self, obj):
        if obj in self.attached_objects:
            self.attached_objects.remove(obj)
            mass = p.getDynamicsInfo(obj.body_id, -1)[0]
            if mass != 0 and hasattr(obj, '_constraint_id'):
                p.removeConstraint(obj._constraint_id)
                obj._constraint_id = None

    def register_callback(self, callback, frequency=0.25):
        self.callbacks.append({'func': callback, 'frequency': frequency, 'last_exec': 0.0})

    def execute_callbacks(self, current_time):
        for cbinfo in self.callbacks:
            freq = cbinfo.get('frequency', 0.25)
            last_exec = cbinfo.get('last_exec', 0.0)
            if current_time - last_exec >= freq:
                cbinfo['func'](self)
                cbinfo['last_exec'] = current_time

class Pose:
    def __init__(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
    def as_position(self):
        return [self.x, self.y, self.z]
    def as_orientation(self):
        return p.getQuaternionFromEuler([self.roll, self.pitch, self.yaw])
    def as_position_orientation(self):
        return self.as_position(), self.as_orientation()

class MeshObject(SimObject):
    @classmethod
    def from_mesh(cls, mesh_path, position, orientation, base_mass=0.0, mesh_scale=[1,1,1], rgbaColor=[1,1,1,1], sim_core=None):
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
        body_id = p.createMultiBody(
            baseMass=base_mass,
            baseCollisionShapeIndex=col_id,
            baseVisualShapeIndex=vis_id,
            basePosition=position,
            baseOrientation=orientation
        )
        return cls(body_id=body_id, mesh_path=mesh_path, visual_id=vis_id, collision_id=col_id, sim_core=sim_core)
    def __init__(self, body_id, mesh_path, visual_id=None, collision_id=None, sim_core=None):
        super().__init__(body_id, sim_core=sim_core)
        self.mesh_path = mesh_path
        self.visual_id = visual_id
        self.collision_id = collision_id
    def set_color(self, rgbaColor, linkIndex=-1):
        p.changeVisualShape(self.body_id, linkIndex, rgbaColor=rgbaColor)

class URDFObject(SimObject):
    @classmethod
    def from_urdf(cls, urdf_path, position, orientation, useFixedBase=False, set_mass_zero=False, meta_data={}, sim_core=None):
        body_id = p.loadURDF(urdf_path, position, orientation, useFixedBase=useFixedBase)
        return cls(body_id, urdf_path, set_mass_zero=set_mass_zero, meta_data=meta_data, sim_core=sim_core)
    def __init__(self, body_id, urdf_path, set_mass_zero=False, meta_data={}, max_accel=1.0, max_speed=1.0, goal_threshold=0.01, sim_core=None):
        super().__init__(body_id, sim_core=sim_core)
        self.urdf_path = urdf_path
        self.joint_info = [p.getJointInfo(body_id, j) for j in range(p.getNumJoints(body_id))]
        if set_mass_zero:
            self.set_all_masses_to_zero()
        self.meta_data = meta_data
        self.max_accel = max_accel
        self.max_speed = max_speed
        self.goal_threshold = goal_threshold
        self.target_actions = []
        self._current_nav_index = 0
        self._nav_velocity = np.array([0.0, 0.0, 0.0])
        self._nav_last_update = None
        self._motion_completed = True

    def set_navigation_params(self, max_accel, max_speed, goal_threshold=None):
        self.max_accel = max_accel
        self.max_speed = max_speed
        if goal_threshold is not None:
            self.goal_threshold = goal_threshold

    def set_action(self, pose_callback_list):
        """
        pose_callback_list: List[Tuple[Pose, Optional[Callable], Optional[float]]]
        Example: [(Pose, callback, wait_time), (Pose, None, 0.0), ...]
        callback is executed on arrival, wait_time is seconds to wait after reaching Pose
        """
        self.target_actions = pose_callback_list
        self._current_nav_index = 0
        self._nav_last_update = None
        self._wait_until = None
        self._motion_completed = False

    def update_action(self, dt=0.01):
        """
        Move sequentially to each Pose in target_actions, considering max acceleration and max speed.
        dt: simulation timestep
        sim_time: simulation time (秒)。wait機能に必要。
        """
        # Wait feature: if _wait_until is set, stop until sim_time reaches that time
        sim_time = self.sim_core.sim_time if self.sim_core is not None else None
        if self._wait_until is not None:
            if sim_time is not None and sim_time < self._wait_until:
                self._nav_velocity = np.array([0.0, 0.0, 0.0])
                return
            else:
                self._wait_until = None
        
        if not self.target_actions or self._current_nav_index >= len(self.target_actions):
            self._motion_completed = True
            return  # Path finished

        # Get current position
        pos, orn = self.get_pose()
        current_pos = np.array(pos)
        # nav_path extension: (Pose, callback, wait_time)
        target_tuple = self.target_actions[self._current_nav_index]
        if isinstance(target_tuple, Pose):
            target_pose = target_tuple
            callback = None
            wait_time = 0.0
        elif len(target_tuple) == 2:
            target_pose, callback = target_tuple
            wait_time = 0.0
        else:
            target_pose, callback, wait_time = target_tuple
        target_pos, target_orn = target_pose.as_position_orientation()
        target_pos = np.array(target_pos)
        # Calculate velocity and acceleration
        direction = target_pos - current_pos
        distance = np.linalg.norm(direction)
        if distance < self.goal_threshold:
            # Goal reached: teleport if within threshold
            self.kinematic_teleport_base(target_pos.tolist(), target_orn)
            # デバッグ用: 次のindexに進む前に一時停止
            # print(f"Reached nav_path index {self._current_nav_index}, pose: {target_pos.tolist()}")
            # input("Press Enter to proceed to next nav_path index...")
            self._current_nav_index += 1
            self._nav_velocity = np.array([0.0, 0.0, 0.0])
            # 到達時callback実行
            if callback:
                callback()
            # wait_time指定があればsim_timeベースで待機
            if wait_time and sim_time is not None:
                self._wait_until = sim_time + wait_time
            return
        direction = direction / (distance + 1e-6)
        # Target speed: do not exceed the distance to the goal
        desired_speed = min(self.max_speed, distance / dt, distance)
        desired_velocity = direction * desired_speed
        # Acceleration limit
        accel = (desired_velocity - self._nav_velocity) / dt
        accel_norm = np.linalg.norm(accel)
        if accel_norm > self.max_accel:
            accel = accel / (accel_norm + 1e-6) * self.max_accel
        self._nav_velocity += accel * dt
        speed = np.linalg.norm(self._nav_velocity)
        if speed > self.max_speed:
            self._nav_velocity = self._nav_velocity / (speed + 1e-6) * self.max_speed
        # Update position
        new_pos = current_pos + self._nav_velocity * dt
        # Orientation matches target Pose
        _, target_orn = target_pose.as_position_orientation()
        self.kinematic_teleport_base(new_pos.tolist(), target_orn)
    def set_all_masses_to_zero(self):
        p.changeDynamics(self.body_id, -1, mass=0)
        for j in range(p.getNumJoints(self.body_id)):
            p.changeDynamics(self.body_id, j, mass=0)
    def kinematic_teleport_joint(self, joint_index, target_pos):
        p.resetJointState(self.body_id, joint_index, target_pos)
    def set_joint_target(self, joint_index, target_pos):
        mass = p.getDynamicsInfo(self.body_id, joint_index)[0]
        if mass == 0:
            self.kinematic_teleport_joint(joint_index, target_pos)
        else:
            p.setJointMotorControl2(self.body_id, joint_index, p.POSITION_CONTROL, targetPosition=target_pos)


class SimulationParams:
    @classmethod
    def from_config(cls, config_path="config.yaml"):
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
        return cls(
            speed=config.get("speed", 1.0),
            timestep=config.get("timestep", 1./10.),
            duration=config.get("duration", 1000),
            gui=config.get("gui", True),
            physics=config.get("physics", False),
            monitor=config.get("monitor", True),
            collision_check_frequency=config.get("collision_check_frequency", None),
            log_level=config.get("log_level", "warn")
        )

    def __init__(self, num_robots=10, speed=1.0, timestep=1./240., duration=10, gui=True, physics=False, monitor=True, collision_check_frequency=None, log_level="warn"):
        self.speed = speed if speed > 0 else 1.0  # If speed <= 0, set to 2.0
        self.timestep = timestep
        self.duration = duration
        self.gui = gui
        self.physics = physics
        self.monitor = monitor
        self.collision_check_frequency = collision_check_frequency
        self.log_level = log_level


class MultiRobotSimulationCore:
    @classmethod
    def from_yaml(cls, yaml_path="config.yaml"):
        params = SimulationParams.from_config(yaml_path)
        return cls(params)
    def __init__(self, params: SimulationParams, collision_color=[0,0,1,1]):
        # ログレベル初期化
        LogLevelManager.set_log_level_from_params(params)
        self.client = None
        self.robots = []  # List of URDFObject instances
        self.mesh_objects = []  # List of MeshObject instances
        self.robot_bodies = []  # For code exchange (AABB, collision, etc.)
        self._last_collided = set()
        self._robot_original_colors = {}  # body_id: rgbaColor
        self.collision_count = 0
        self.step_count = 0
        self.sim_time = 0.0  # Simulation time
        self.start_time = None
        self.monitor_enabled = params.monitor
        self.last_monitor_update = 0
        self.callbacks = []  # List of callback functions
        self.data_monitor = None
        self.collision_check_frequency = params.collision_check_frequency  # If None, check every step
        self.last_collision_check = 0.0
        self.log_level = params.log_level
        self.params = params
        self.collision_color = collision_color
        self.setup_pybullet()
        self.setup_monitor()

    def setup_monitor(self):
        # If monitor: true and console_monitor: false, start DataMonitor
        from core.data_monitor import DataMonitor
        if self.params.monitor:
            self.data_monitor = DataMonitor("PyBullet Simulation Monitor")
            self.data_monitor.start()
        else:
            self.data_monitor = None

    def register_callback(self, callback, frequency=None):
        """
        Register a callback function to be called every step.
        frequency (Hz, number of times per second) can be specified. If None, called every step.
        Default value is 2Hz (every 0.5 seconds).
        For old-style individual callback registration, refer to Robot.register_callback.
        """
        if frequency is None:
            frequency = 2  # Default 2Hz
        self.callbacks.append({'func': callback, 'frequency': frequency, 'last_exec': 0.0})

    def set_collision_check_frequency(self, frequency=None):
        """
        Set the frequency (Hz, number of times per second) for collision detection. If None, check every step.
        Default value is 2Hz (every 0.5 seconds).
        """
        if frequency is None:
            frequency = 2
        self.collision_check_frequency = frequency

    def setup_pybullet(self):
        self.client = p.connect(p.GUI if self.params.gui else p.DIRECT)
        import pybullet_data
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81 if self.params.physics else 0)
        p.setTimeStep(self.params.timestep)
        p.setRealTimeSimulation(0)
        # High-speed parameter settings
        p.setPhysicsEngineParameter(enableFileCaching=True)
        p.setPhysicsEngineParameter(deterministicOverlappingPairs=True)
        p.setPhysicsEngineParameter(allowedCcdPenetration=0.01)
        if not self.params.physics:
            p.setPhysicsEngineParameter(numSubSteps=1)
            p.setPhysicsEngineParameter(numSolverIterations=1)
            p.setPhysicsEngineParameter(enableConeFriction=False)
        self.plane_id = p.loadURDF("plane.urdf")
        # GUI visualization speed-up (disable drawing, shadows, wireframe, GUI panel as needed)
        # *COV_ENABLE_RENDERING should not be disabled at initialization, so the screen is updated every step
        if self.params.gui:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    def get_aabbs(self):
        return [p.getAABB(body_id) for body_id in self.robot_bodies]

    def filter_aabb_pairs(self):
        aabbs = self.get_aabbs()
        pairs = []
        for i in range(len(self.robot_bodies)):
            aabb_i = aabbs[i]
            for j in range(i + 1, len(self.robot_bodies)):
                aabb_j = aabbs[j]
                if (aabb_i[1][0] < aabb_j[0][0] or aabb_i[0][0] > aabb_j[1][0] or
                    aabb_i[1][1] < aabb_j[0][1] or aabb_i[0][1] > aabb_j[1][1] or
                    aabb_i[1][2] < aabb_j[0][2] or aabb_i[0][2] > aabb_j[1][2]):
                    continue
                pairs.append((i, j))
        return pairs

    def check_collisions(self, collision_color=None):
        if collision_color is None:
            collision_color = self.collision_color
        collision_pairs = []
        collided = set()
        # 記録: 初期色保存
        for idx, body_id in enumerate(self.robot_bodies):
            if body_id not in self._robot_original_colors:
                # 現在の色を取得（PyBullet APIで取得不可のため、初期値で仮定）
                self._robot_original_colors[body_id] = [0.0, 0.0, 0.0, 1]
        for i, j in self.filter_aabb_pairs():
            contact_points = p.getContactPoints(self.robot_bodies[i], self.robot_bodies[j])
            if contact_points:
                collision_pairs.append((i, j))
                collided.add(i)
                collided.add(j)
        # Color update (collision: blue, normal: original)
        for idx, body_id in enumerate(self.robot_bodies):
            was_collided = idx in self._last_collided
            is_collided = idx in collided
            if was_collided != is_collided:
                if is_collided:
                    p.changeVisualShape(body_id, -1, rgbaColor=collision_color)
                else:
                    orig_color = self._robot_original_colors.get(body_id, [0,0,0,1])
                    p.changeVisualShape(body_id, -1, rgbaColor=orig_color)
        self._last_collided = collided
        self.collision_count += len(collision_pairs)
        return collision_pairs

    def update_monitor(self, suppress_console=False):
        now = time.time()
        sim_time = self.step_count * self.params.timestep
        # --- 速度履歴バッファ ---
        if not hasattr(self, '_speed_history'):
            self._speed_history = []  # [(real_time, sim_time)]
        self._speed_history.append((now, sim_time))
        # 10秒以内の履歴のみ残す
        self._speed_history = [(rt, st) for rt, st in self._speed_history if now - rt <= 10.0]
        # 直近10秒間の速度計算
        if len(self._speed_history) >= 2:
            rt0, st0 = self._speed_history[0]
            rt1, st1 = self._speed_history[-1]
            elapsed = rt1 - rt0
            sim_elapsed = st1 - st0
            actual_speed = sim_elapsed / elapsed if elapsed > 0 else 0
        else:
            actual_speed = 0
        elapsed_time = now - self.start_time if self.start_time else 0
        monitor_data = {
            'sim_time': sim_time,
            'real_time': elapsed_time,
            'target_speed': self.params.speed,
            'actual_speed': actual_speed,
            'time_step': self.params.timestep,
            'frequency': 1/self.params.timestep,
            'physics': 'enabled' if self.params.physics else 'disabled',
            'robots': len(self.robot_bodies),
            'collisions': self.collision_count,
            'steps': self.step_count
        }
        # Log level control (minimum: warn, info for details, debug for all output)
        log_level = self.log_level if not suppress_console else "warn"
        logging.debug("MONITOR:")
        logging.debug(json.dumps(monitor_data, indent=2, ensure_ascii=False))
        logging.info(f"sim_time={sim_time:.2f}, real_time={elapsed_time:.2f}, speed={actual_speed:.2f}, collisions={self.collision_count}, steps={self.step_count}")
        if self.collision_count > 0:
            logging.info(f"collisions={self.collision_count} at sim_time={sim_time:.2f}")
        # Also output to DataMonitor window
        if self.data_monitor:
            self.data_monitor.write_data(monitor_data)

    def run_simulation(self, duration=None):
        if duration is None:
            duration = self.params.duration
        self.start_time = time.time()
        self.step_count = 0
        self.collision_count = 0
        # After robot generation, re-enable drawing (only when GUI is enabled)
        if self.params.gui:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
        try:
            start_time = time.time()
            while True:
                current_time = time.time()
                elapsed_time = current_time - start_time
                if duration != 0 and elapsed_time >= duration:
                    break
                # Determine stepSimulation count by difference between simulation time and real time
                target_sim_time = elapsed_time * self.params.speed
                current_sim_time = self.step_count * self.params.timestep
                steps_needed = int((target_sim_time - current_sim_time) / self.params.timestep)
                steps_needed = max(1, min(steps_needed, 10))  # 1 to 10 times
                for _ in range(steps_needed):
                    self.step_once()
                # If caught up, sleep at 60FPS (when GUI), otherwise minimum sleep
                if current_sim_time >= target_sim_time:
                    if self.params.gui:
                        time.sleep(1.0 / 60.0)
                    else:
                        time.sleep(0.001)
        except KeyboardInterrupt:
            logging.warning("Simulation interrupted by user")
        self.update_monitor()
        p.disconnect()

    def step_once(self):
        # Synchronize robot_bodies from robots every step
        self.robot_bodies = [robot.body_id for robot in self.robots]
        self.sim_time = self.step_count * self.params.timestep
        # Global callbacks (frequency control)
        for cbinfo in self.callbacks:
            freq = cbinfo.get('frequency', None)
            last_exec = cbinfo.get('last_exec', 0.0)
            interval = 1.0 / freq if freq else 0.0
            # self.sim_timeベースで判定
            if freq is None or self.sim_time - last_exec >= interval:
                dt = self.sim_time - last_exec if last_exec > 0 else self.params.timestep
                cbinfo['func'](self.robots, self, dt)
                cbinfo['last_exec'] = self.sim_time
        # Old style: individual robot callbacks executed according to frequency
        for robot in self.robots:
            robot.execute_callbacks(self.sim_time)
        p.stepSimulation()
        # Collision check frequency control
        freq = self.collision_check_frequency
        interval = 1.0 / freq if freq else 0.0
        # collision checkもself.sim_timeベースで判定
        if freq is None or self.sim_time - self.last_collision_check >= interval:
            self.check_collisions()
            self.last_collision_check = self.sim_time
        self.step_count += 1
        # Monitor: every step if GUI enabled, otherwise every second
        if self.monitor_enabled:
            interval = self.params.timestep if self.params.gui else 1.0
            if self.sim_time - self.last_monitor_update > interval:
                self.update_monitor()
                self.last_monitor_update = self.sim_time
