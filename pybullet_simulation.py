import pybullet as p
import pybullet_data
import time
import numpy as np
from dataclasses import dataclass
rng = np.random.default_rng()

@dataclass(frozen=True)
class MotorConfig:
    MAX_TORQUE: float = 0.08
    MAX_VEL: float = 40.0
    DEADBAND_RATIO: float = 0.3

class PID:
    def __init__(self, max_output: float, Kp: float = 1.0, Ki: float = 0.0, Kd: float = 0.0):
        self.dt = 1.0 / 240.0

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        self.output_min = -max_output
        self.output_max = max_output
        
        self.integral = 0.0
        self.last_input = 0.0
    
    def set_parameters(self, dt: float, kp: float = None, ki: float = None, kd: float = None) -> None:
        self._dt = dt
        if kp is not None: self._kp = kp
        if ki is not None: self._ki = ki
        if kd is not None: self._kd = kd

    def reset(self):
        self.integral = 0.0
        self.last_input = 0.0

    def compute(self, setpoint: float, current_value: float) -> float:
        sp = np.deg2rad(setpoint)
        error = sp - current_value
        
        P = self.Kp * error
        
        if self.Ki != 0:
            self.integral += error * self.dt
            # Anti-Windup
            self.integral = max(min(self.integral, self.output_max / self.Ki), 
                                self.output_min / self.Ki)
        I = self.Ki * self.integral
        
        # Derivative on Measurement
        D = -self.Kd * (current_value - self.last_input) / self.dt
        
        output = P + I + D
        output = max(min(output, self.output_max), self.output_min)
        
        self.last_input = current_value
        return output


'''
TODO:
    - add a messaging system to interact with physical robot
    (start, stop, reset, setPID etc)

updatePos:
    - sim / IMU

resetPos:
    - add a STOP signal for robot
setPID:
    - sim / MCU

    
'''
class Robot:
    def __init__(self, urdf_path: str, scale: float = 1, Kp:float=1, Ki:float=0, Kd:float=0) -> None:
        # Position and URDF config
        self._scale = scale
        self._start_pos = [0,0,1*scale/5]
        
        start_pitch = rng.uniform(-5.0, 5.0)
        self._start_orientation = p.getQuaternionFromEuler([0, np.deg2rad(start_pitch), 0])
        self._id = p.loadURDF(urdf_path, self._start_pos, self._start_orientation, globalScaling = scale)
        
        self._y_axis_num:int = 0
        self._orientation = self._start_orientation
        self._y_angle = p.getEulerFromQuaternion(self._orientation)[self._y_axis_num] # obrów w osi Y
        self._position = self._start_pos
        self._sensor_bias = 0.0 # 

        # Joints info
        self._joints_num = p.getNumJoints(self._id)
        self._joint_indices = [
            i for i in range(self._joints_num) 
            if p.getJointInfo(self._id, i)[2] != p.JOINT_FIXED
        ]

        self._setup_dynamics()

        # PID controller
        self._controller = PID(max_output=MotorConfig.MAX_VEL, Kp=Kp, Ki=Ki, Kd=Kd)
        self._free_motors()

        # Debug
        self._show_com = False

    def _setup_dynamics(self) -> None:
        """ Some physical parameters of the robot """
        for i in self._joint_indices:
            p.changeDynamics(
                bodyUniqueId=self._id, linkIndex=i, 
                lateralFriction=1.2, rollingFriction=0.01, spinningFriction=0.01,
                jointDamping=0.005, linearDamping=0.01, angularDamping=0.01
            )

    @property
    def position(self) -> list[float]:
        return list(p.getBasePositionAndOrientation(self._id)[0])

    def configure_pid(self, dt: float = None, kp: float = None, ki: float = None, kd: float = None) -> None:
        self._controller.set_parameters(dt, kp, ki, kd)

    def enable_com_display(self, enable: bool) -> None:
        self._show_com = enable

    def apply_disturbance(self, force: list[float]) -> None:
        # todo - losowanie roznych czesci robota ??
        pos = self.position
        p.applyExternalForce(self._id, -1, forceObj=force, posObj=pos, flags=p.WORLD_FRAME)
        
        # Draw the force vector
        scale = 0.05 
        end_pos = [pos[0] + force[0] * scale, pos[1] + force[1] * scale, pos[2] + force[2] * scale]
        p.addUserDebugLine(pos, end_pos, [1, 0.5, 0], 4, 0.5)

    # (Virtual) Simulate IMU
    def _update_sensor(self) -> None:
        self._position, self._orientation = p.getBasePositionAndOrientation(self._id)
        sim_angle =  p.getEulerFromQuaternion(self._orientation)[self._y_axis_num] # obrów w osi Y

        self._sensor_bias += rng.normal(0.0, 0.0001)
        noise = rng.normal(0.0, np.deg2rad(0.5))
        real_angle = sim_angle + noise + self._sensor_bias
        
        alpha = 0.85
        self._y_angle = alpha * self._y_angle + (1-alpha) * real_angle
            
    # (Virtual) Reset the position and PID
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
        self._y_angle = p.getEulerFromQuaternion(self._orientation)[self._y_axis_num]
        self._position = self._start_pos

    '''
    Motor and Control functions
    '''
    # (Virtual) STOP the motors
    def _free_motors(self):
        if not self._joint_indices: return  
        p.setJointMotorControlArray(
            bodyUniqueId=self._id,
            jointIndices=self._joint_indices,
            controlMode=p.VELOCITY_CONTROL,
            forces=[0.0] * len(self._joint_indices)
        )

    # (Virtual) 
    def _control_motors(self, target_val: list[float], mode: int = p.TORQUE_CONTROL) -> None:
        
        deadband = MotorConfig.DEADBAND_RATIO * MotorConfig.MAX_TORQUE
        real_torque = []
              
        # torque = [max_torque] * len(self._joint_indices)
        real_torque = []
        for val in target_val:
            torque = max(min(val, MotorConfig.MAX_TORQUE), -MotorConfig.MAX_TORQUE)

            # Add deadband
            # real_torque.append(0.0 if abs(torque) < deadband else torque)
            if abs(torque) < deadband:
                real_torque.append(0.0)
            else:
                real_torque.append(torque)

        p.setJointMotorControlArray(
            self._id, 
            self._joint_indices, 
            mode, 
            # targetVelocities = target_val,
            forces = real_torque
        )
        
    def update(self) -> None:
        """ Main robot function """
        self._update_sensor()
        
        setpoint = 0.0
        noise = rng.uniform(-np.deg2rad(1), np.deg2rad(1))
        angle = self._y_angle + noise
        
        motor_val = self._controller.compute(setpoint, angle)
        self._control_motors([-motor_val, motor_val])

        # Reset when fallen
        if abs(self._y_angle) > np.deg2rad(70):
            self._controller.reset()
            self._reset_position()

    '''
    Debug functions
    '''
    def showCOM(self, val: bool)-> None:
        self._show_com = val

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

class Simulation:
    def __init__(self, timestep:float = 1./240. ) -> None:
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-9.81)

        self._show_wireframe = False 
        p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0) # domyślnie wyłączone
        
        self._timestep = timestep
        p.setTimeStep(timestep)

        # 
        self._robot: Robot | None = None
        self._is_running = False

        self._disturb_force = 0.0
        self._disturb_interval = 200


    def load_plane(self, urdf_path: str) -> int:
        self._id = p.loadURDF(urdf_path)
        p.changeDynamics(self._id, -1, lateralFriction=1.2)
        # return self._id

    def attach_robot(self, robot: Robot) -> None:
        self._robot = robot

    def set_disturbances(self, force: float = 0, interval_steps: int = 200) -> None:
        """  """
        self._disturb_force = force
        self._disturb_interval = interval_steps

    def start(self) -> None:
        self._is_running = True

    def stop(self) -> None:
        self._is_running = False

    def _trigger_disturbance(self) -> None:
        if self._robot and self._disturb_force > 0:
            force = np.random.uniform(-self._disturb_force, self._disturb_force, size=3).tolist()
            self._robot.apply_disturbance(force)

                
    def update_camera_tracking(self):
        """Aktualizuje pozycję kamery, aby śledziła robota."""
        if self.robot is None:
            return

        # Pobieramy aktualną pozycję bazy robota [x, y, z]
        base_pos, _ = p.getBasePositionAndOrientation(self.robot._id)
        
        # --- Konfiguracja kamery ---
        # Możesz dobrać te wartości eksperymentalnie:
        distance = 1.0  # Dystans kamery od robota (w metrach)
        yaw = 50         # Obrót poziomy (0 = patrzy wzdłuż osi X, 90 = wzdłuż Y)
        pitch = -35      # Kąt nachylenia pionowego (-90 = pionowo z góry, 0 = poziom)

        # Ustawiamy kamerę tak, aby patrzyła na środek robota
        p.resetDebugVisualizerCamera(
            cameraDistance=distance,
            cameraYaw=yaw,
            cameraPitch=pitch,
            cameraTargetPosition=base_pos
        )

    def enable_wireframe(self, enable: bool) -> None:
        """Włącza lub wyłącza tryb wireframe w wizualizatorze PyBullet."""
        self._show_wireframe = enable
        # Rzutujemy bool na int (True -> 1, False -> 0)
        p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 1 if enable else 0)

    def run(self, max_steps: int = 10000, pid_freq: float = 100.0) -> None:
        if not self._robot:
            print("Error: robot not connected")
            return

        pid_dt = 1.0 / pid_freq
        self._robot.configure_pid(dt=pid_dt)
        time_accumulator = 0.0

        for step in range(max_steps):
            time_accumulator += self._timestep

            # PID freq 
            if self._is_running and time_accumulator >= pid_dt:
                self._robot.update()
                time_accumulator -= pid_dt

            if step % 10 == 0:
                self._robot.draw_debug_data()

            # Disturbances
            if self._disturb_force > 0 and step % self._disturb_interval == 0:
                self._trigger_disturbance()

            p.stepSimulation()
            time.sleep(self._timestep)

    def disconnect(self):
        p.disconnect()


if __name__ == "__main__":
    sim = Simulation()
    sim.load_plane("plane.urdf")

    robot = Robot("robot.urdf", scale=1)
    
    robot.configure_pid(kp=15, kd=1)

    robot.enable_com_display(True)
    
    sim.attach_robot(robot)
    sim.set_disturbances(force=30.0, interval_steps=200)
    sim.start()

    sim.enable_wireframe(True)

    sim.run(max_steps=10000, pid_freq=200)

    sim.disconnect()
