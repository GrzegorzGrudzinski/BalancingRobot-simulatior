'''
    robot.py
'''

import pybullet as p
import numpy as np

from sim_components.config import RobotConfig, SimConfig

WHEEL_RADIUS = 0.055  # [m] same as firmware config.h, used for position/velocity

class Robot:
    def __init__(self, urdf_path: str, config: RobotConfig, 
                 scale: float = 1, rng=None, sim_cfg: SimConfig | None = None) -> None:
        # config
        self._config = config
        self._sim_cfg = sim_cfg if sim_cfg is not None else SimConfig()
        self._scale = scale
        self._start_pos = [0, 0, 0.14 * scale]  
        self._rng = rng if rng is not None else np.random.default_rng()
        
        # start_pitch = self._rng.uniform(-5.0, 5.0)
        self._start_orientation = self._sample_start_orientation()
        self._pitch_axis: int = 0  # euler index of rotation about the wheel axis (X)
        self._id = p.loadURDF(urdf_path, self._start_pos, self._start_orientation, globalScaling = scale)
        self._orientation = self._start_orientation
        self._position = self._start_pos

        # Joints info
        self._joints_num = p.getNumJoints(self._id)
        self._joint_indices = [
            i for i in range(self._joints_num) 
            if p.getJointInfo(self._id, i)[2] != p.JOINT_FIXED
        ]

        self._max_angle = 70

        self._setup_dynamics()
        start_angle = p.getEulerFromQuaternion(self._start_orientation)[self._pitch_axis]
        self._config.sensor_imu.reset(start_angle)
        self._config.controller.reset(np.rad2deg(start_angle))
        self._free_motors()

        self._dt = 1.0 / 240.0
        # Debug
        self._show_com = False
        self.reset_flag = 0

        self.measured_pitch = start_angle   # [rad] what the controller saw last
        self.last_out = (0.0, 0.0)          # [A] firmware out_l, out_r


    def _setup_dynamics(self) -> None:
        """ Some physical parameters of the robot """
        # mass_variance = self._rng.uniform(0.9, 1.1)
        cfg = self._sim_cfg
        friction_variance = self._rng.uniform(1 - cfg.friction_variance, 1 + cfg.friction_variance)

        for i in self._joint_indices:
            p.changeDynamics(
                bodyUniqueId=self._id, linkIndex=i,
                lateralFriction=cfg.lateral_friction * friction_variance,
                rollingFriction=cfg.rolling_friction,
                spinningFriction=cfg.spinning_friction,
                jointDamping=cfg.joint_damping, linearDamping=0.01, angularDamping=0.01
            )

    def _sample_start_orientation(self) -> list[float]:
        cfg = self._sim_cfg
        pitch_deg = cfg.start_pitch_deg
        if pitch_deg is None:
            pitch_deg = self._rng.uniform(-cfg.start_pitch_range_deg, cfg.start_pitch_range_deg)
        return p.getQuaternionFromEuler([np.deg2rad(pitch_deg), 0, 0])

    @property
    def position(self) -> list[float]:
        return list(p.getBasePositionAndOrientation(self._id)[0])

    def enable_com_display(self, enable: bool) -> None:
        self._show_com = enable
    
    def set_dt(self, dt: float) -> None:
        self._dt = dt
    
    def apply_disturbance(self, force: list[float]) -> None:
        # todo - losowanie roznych czesci robota ??
        pos = self.position
        p.applyExternalForce(self._id, -1, forceObj=force, posObj=pos, flags=p.WORLD_FRAME)

                
    def _reset_position(self, pos = None, orn = None) -> None:
        ''' Reset the robot to the starting state '''
        p.resetBaseVelocity(self._id, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0])

        # Stop the motors
        if self._joint_indices:
            for i in self._joint_indices:
                p.resetJointState(self._id, i, targetValue=0.0, targetVelocity=0.0)

        self._start_orientation = self._sample_start_orientation()
        p.resetBasePositionAndOrientation(self._id, self._start_pos, self._start_orientation)
        self._orientation = self._start_orientation
        self._position = self._start_pos

        start_angle = p.getEulerFromQuaternion(self._start_orientation)[self._pitch_axis]
        self._config.sensor_imu.reset(start_angle)
        self._config.controller.reset(np.rad2deg(start_angle))
        self._config.motors.reset()

        self.reset_flag = 1

    '''
    Motor and Control functions
    '''
    def _free_motors(self):
        if not self._joint_indices: return  
        p.setJointMotorControlArray(
            bodyUniqueId=self._id,
            jointIndices=self._joint_indices,
            controlMode=p.VELOCITY_CONTROL,
            forces=[0.0] * len(self._joint_indices)
        )

    def _control_motors(self, target_val: list[float], mode: int = p.TORQUE_CONTROL) -> None:
        self._config.motors.command(target_val)        

    def apply_motors(self) -> None:
        """Call before every stepSimulation(): holds the last command and applies the delay."""
        self._config.motors.apply(self._id, self._joint_indices)

    def read_state(self) -> dict:
        """True state from PyBullet in firmware units (deg, deg/s, m, m/s, rad, rad/s)."""
        pos, orn = p.getBasePositionAndOrientation(self._id)
        _, ang_vel = p.getBaseVelocity(self._id)
        wheel_axis = np.array(p.getMatrixFromQuaternion(orn))[[0, 3, 6]]  # body X in world
        body_z = np.array(p.getMatrixFromQuaternion(orn))[[2, 5, 8]]
        l_pos, r_pos, l_vel, r_vel = self._config.sensor_encoder.read_all(self._id, self._joint_indices)
        l_pos, l_vel = -l_pos, -l_vel  # wheel axes are mirrored in the URDF
        return {
            "pitch": np.rad2deg(p.getEulerFromQuaternion(orn)[self._pitch_axis]),
            "pitch_rate": np.rad2deg(float(np.dot(ang_vel, wheel_axis))),
            "yaw_rate": np.rad2deg(np.dot(ang_vel, body_z)),
            "position": 0.5 * (l_pos + r_pos) * WHEEL_RADIUS,
            "velocity": 0.5 * (l_vel + r_vel) * WHEEL_RADIUS,
            "left_wheel_pos": l_pos, "right_wheel_pos": r_pos,
            "mot1_vel": l_vel, "mot2_vel": r_vel,
            "base_x": pos[0], "base_y": pos[1],
        }

    def update(self, command: tuple[float, float] | None = None) -> None:
        """ Main robot function. command = (out_l, out_r) [A] from an external controller
        (SiL, HiL, RL), None = use the built-in controller. """
        
        self.reset_flag = 0
        angle = self._config.sensor_imu.read(self._id, self._pitch_axis)  # [rad]
        self.measured_pitch = angle

        if command is None:
            # controller works in firmware units: deg in, A out
            setpoint = 0.0
            u = self._config.controller.compute(setpoint, np.rad2deg(angle), self._dt)
            command = (u, u)
        self.last_out = tuple(command)
        
        # firmware sends the same sign to both motors, URDF wheel axes are mirrored
        self._control_motors([-command[0], command[1]])

        # Reset when fallen
        if abs(angle) > np.deg2rad(self._max_angle):
            self._reset_position()
        else:
            self.reset_flag = 0

    def test_motors(self):
        pass

    '''
    Debug functions
    '''
    def _get_total_com(self):
        total_mass = 0
        com_x, com_y, com_z = 0.0, 0.0, 0.0
        
        # Obliczenie dla bazy (link_index = -1)
        pos, _ = p.getBasePositionAndOrientation(self._id)
        mass = p.getDynamicsInfo(self._id, -1)[0]
        total_mass += mass
        com_x += pos[0] * mass
        com_y += pos[1] * mass
        com_z += pos[2] * mass
        
        # Obliczenie dla pozostałych linków
        for i in range(self._joints_num):
            link_state = p.getLinkState(self._id, i)
            link_pos = link_state[0] # worldLinkFramePosition
            link_mass = p.getDynamicsInfo(self._id, i)[0]
            
            total_mass += link_mass
            com_x += link_pos[0] * link_mass
            com_y += link_pos[1] * link_mass
            com_z += link_pos[2] * link_mass
            
        if total_mass == 0:
            return pos # Zabezpieczenie przed dzieleniem przez 0
            
        return [com_x / total_mass, com_y / total_mass, com_z / total_mass]

    def draw_frame_com(self):
        self._update_sensor()
        global_com = self._get_total_com()
        # global_com = self.position

        # p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 1)

        p.addUserDebugPoints(
            pointPositions=[self.position, global_com, [global_com[0], global_com[1], 0]], 
            pointColorsRGB=[[1, 0, 0], [0, 1, 0], [0, 0, 1]], 
            pointSize=1*self._scale, 
            lifeTime=0.1
        )
        
    def draw_debug_data(self) -> None:
        if not self._show_com:
            return
            
        global_com = self._get_total_com()
        # global_com = self.position
        p.addUserDebugPoints(
            # punkty: środek układu robota, wyliczony com, com rzutowany na plane
            pointPositions=[self.position, global_com, [global_com[0], global_com[1], 0]], 
            pointColorsRGB=[[1, 0, 0], [0, 1, 0], [0, 0, 1]], 
            pointSize=8 * self._scale, lifeTime=0.1
        )
