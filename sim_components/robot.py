'''
    robot.py
'''

import pybullet as p
import numpy as np

from sim_components.config import RobotConfig

rng = np.random.default_rng()

class Robot:
    def __init__(self, urdf_path: str, config: RobotConfig, scale: float = 1) -> None:
        # config
        self._config = config
        self._scale = scale
        self._start_pos = [0,0,1*scale/5]
        
        start_pitch = rng.uniform(-5.0, 5.0)
        self._start_orientation = p.getQuaternionFromEuler([0, np.deg2rad(start_pitch), 0])
        self._id = p.loadURDF(urdf_path, self._start_pos, self._start_orientation, globalScaling = scale)
        
        self._y_axis_num:int = 0
        self._orientation = self._start_orientation
        self._position = self._start_pos

        # Joints info
        self._joints_num = p.getNumJoints(self._id)
        self._joint_indices = [
            i for i in range(self._joints_num) 
            if p.getJointInfo(self._id, i)[2] != p.JOINT_FIXED
        ]

        self._setup_dynamics()
        self._config.sensor_imu.reset(np.deg2rad(start_pitch))
        self._free_motors()

        # Debug
        self._show_com = False
        #
        self._dt = 1.0 / 240.0

    def _setup_dynamics(self) -> None:
        """ Some physical parameters of the robot """
        # mass_variance = rng.uniform(0.9, 1.1)
        friction_variance = rng.uniform(0.8, 1.2)

        for i in self._joint_indices:
            p.changeDynamics(
                bodyUniqueId=self._id, linkIndex=i, 
                lateralFriction=1.2 * friction_variance, 
                rollingFriction=0.01, 
                spinningFriction=0.01,
                jointDamping=0.005, linearDamping=0.01, angularDamping=0.01
            )

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
        
        # Draw the force vector
        scale = 0.05 
        end_pos = [pos[0] + force[0] * scale, pos[1] + force[1] * scale, pos[2] + force[2] * scale]
        p.addUserDebugLine(pos, end_pos, [1, 0.5, 0], 4, 0.5)

                
    def _reset_position(self, pos = None, orn = None) -> None:
        ''' Reset the robot to the starting state '''
        p.resetBaseVelocity(self._id, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0])

        # Stop the motors
        if self._joint_indices:
            self._control_motors([0.0] * len(self._joint_indices))
            for i in self._joint_indices:
                p.resetJointState(self._id, i, targetValue=0.0, targetVelocity=0.0)

        p.resetBasePositionAndOrientation(self._id, self._start_pos, self._start_orientation)
        self._orientation = self._start_orientation
        self._position = self._start_pos

        start_angle = p.getEulerFromQuaternion(self._start_orientation)[self._y_axis_num]
        self._config.sensor_imu.reset(start_angle)
        self._config.controller.reset()

        for motor in self._config.motors:
            motor.reset()

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
        torque = [
            motor.move(val) 
            for motor, val in zip(self._config.motors, target_val)
        ]
        
        p.setJointMotorControlArray(
            self._id, 
            self._joint_indices, 
            mode, 
            # targetVelocities = target_val,
            forces = torque
        )
        
    def update(self) -> None:
        """ Main robot function """
        
        angle = self._config.sensor_imu.read(self._id, self._y_axis_num)
        setpoint = 0.0
        
        motor_val = self._config.controller.compute(setpoint, angle, self._dt)
        self._control_motors([-motor_val, motor_val])

        # Reset when fallen
        if abs(angle) > np.deg2rad(70):
            self._reset_position()

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
