import pybullet as p
import pybullet_data
import time
import numpy as np
rng = np.random.default_rng()

class motor:
    def __init__(self):
        self.max_force = 1.5
        self.max_vel = 200

class PID:
    def __init__(self, max_output: float, Kp:float=1, Ki:float=0, Kd:float=0):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.output_min = -max_output
        self.output_max = max_output
        
        self.dt = 1. / 240.
        self.integral = 0
        self.last_input = 0 # wejście zamiast błędu dla członu D

    def reset(self):
        self.integral = 0
        self.last_input = 0

    def update(self, setpoint, current_value):
        sp = np.deg2rad(setpoint)
        error = sp - current_value
        
        # 1. Proportional
        P = self.Kp * error
        
        # 2. Integral + Anti-Windup
        if self.Ki != 0:
            self.integral += error * self.dt
            # Ograniczamy całkę, by nie "odleciała" poza zakres sterowania
            self.integral = max(min(self.integral, self.output_max / self.Ki), 
                                self.output_min / self.Ki)
        I = self.Ki * self.integral
        
        # 3. Derivative on Measurement (unikamy kopnięcia przy zmianie setpointu)
        # d(error)/self.dt zamieniamy na -d(PV)/self.dt
        D = -self.Kd * (current_value - self.last_input) / self.dt
        
        # 4. Suma i nasycenie wyjścia (Output Saturation)
        output = P + I + D
        output = max(min(output, self.output_max), self.output_min)
        
        self.last_input = current_value
        return output

class Robot:
    def __init__(self, urdf_path: str, scale: float = 1, Kp:float=1, Ki:float=0, Kd:float=0) -> None:
        # Position and URDF config
        self.startPos = [0,0,1*scale/5]
        self.startOrientation = p.getQuaternionFromEuler([0,0,0])
        self.scale = scale
        self.ID = p.loadURDF(urdf_path, self.startPos, self.startOrientation, globalScaling = scale)
        
        self.Y_axis_num:int = 0
        self.Orientation = self.startOrientation
        self.Yangle = p.getEulerFromQuaternion(self.Orientation)[self.Y_axis_num] # obrów w osi Y
        self.Position = self.startPos

        # Joints info
        self.JointsNum = p.getNumJoints(self.ID)
        self.jointIndices = []
        self.JointsInfo = []
        for i in range(self.JointsNum):
            info = p.getJointInfo(self.ID, i)
            self.JointsInfo.append(info)
            if info[2] != p.JOINT_FIXED:
                self.jointIndices.append(i)

        # motor
        self.bldc_motor = motor()
        # PID controller
        self.controller = PID(max_output=self.bldc_motor.max_vel, Kp=Kp,Ki=Ki,Kd=Kd)
        self.freeMotors()

        # Debug
        self.display_com_flag: int = 0

    def updatePos(self) -> tuple:
        self.Position, self.Orientation = p.getBasePositionAndOrientation(self.ID)
        self.Yangle = p.getEulerFromQuaternion(self.Orientation)[self.Y_axis_num] # obrów w osi Y
        
        # return self.Position, self.Orientation
    
    def resetPos(self, pos = None, orn = None) -> None:
        p.resetBaseVelocity(self.ID, linearVelocity=[0, 0, 0], angularVelocity=[0, 0, 0])

        if hasattr(self, 'jointIndices') and len(self.jointIndices) > 0:
            zero_velocities = [0.0] * len(self.jointIndices)
            self.controlMotors(zero_velocities)
            
            # Fizycznie zatrzymujemy obrót kół w silniku fizycznym
            for i in self.jointIndices:
                p.resetJointState(self.ID, i, targetValue=0.0, targetVelocity=0.0)

        if pos is None:
            pos = self.startPos
        if orn is None:
            orn = self.startOrientation
        
        p.resetBasePositionAndOrientation(self.ID, pos, orn)
        
        self.Orientation = self.startOrientation
        self.Yangle = p.getEulerFromQuaternion(self.Orientation)[self.Y_axis_num] # obrów w osi Y
        self.Position = self.startPos

        # time.sleep(0.05)

    '''
    Motor and Control functions
    '''
    def setPID(self, timestep: float, Kp: float = None, Ki: float = None, Kd: float = None) -> None:
        self.controller.dt = timestep
        if Kp is not None: self.controller.Kp = Kp
        if Ki is not None: self.controller.Ki = Ki
        if Kd is not None: self.controller.Kd = Kd

    def freeMotors(self):
        if not self.jointIndices: return
        zero_forces = [0.0] * len(self.jointIndices)        
        p.setJointMotorControlArray(
            bodyUniqueId=self.ID,
            jointIndices=self.jointIndices,
            controlMode=p.VELOCITY_CONTROL,
            forces=zero_forces
        )

    def controlMotors(self, targetVal: list[float], mode: int = p.VELOCITY_CONTROL) -> None:
        max_force = self.bldc_motor.max_force
        forces = [max_force] * len(self.jointIndices)

        # motorArr = [motor1 value, motor2 value ...]
        p.setJointMotorControlArray(
            self.ID, 
            self.jointIndices, 
            mode, 
            targetVelocities = targetVal,
            forces = forces
        )
        
    def getMotorState(self, jointIndex: int) -> tuple[float, float, list[float], float]:
        # returns: jointPosition, jointVelocity, jointForces, appliedTorque
        return p.getJointState(self.ID, jointIndex)
    
    def steerRobot(self):
        self.updatePos()
        setpoint = 0

        noise_val = np.deg2rad(0) # 1 deg 
        noise = rng.uniform(-noise_val, noise_val)
        angle= self.Yangle + noise
        
        motorsVal = self.controller.update(setpoint, angle)
        targetVal = [-motorsVal, motorsVal]
        self.controlMotors(targetVal)

    def robotSimulation(self):
        self.steerRobot()
        if abs(self.Yangle) > np.deg2rad(70):
            self.controller.reset()
            self.resetPos()
            
    '''
    Debug functions
    '''
    def printJointInfo(self) -> None:
        info: str = ['jointIndex', 'jointName', 'jointType', 'qIndex', 
                    'uIndex', 'flags', 'jointDamping', 'jointFriction', 
                    'jointLowerLimit', 'jointUpperLimit','jointMaxForce', 
                    'jointMaxVelocity', 'linkName', 'jointAxis', 
                    'parentFramePos', 'parentFrameOrn', 'parentIndex'  ]
        for i in range(self.JointsNum):
            print(f'\nJoint {i} info:')
            joint_data = self.JointsInfo[i]
            for j in range(len(self.JointsInfo[i])):
              print(f'\t{info[j]}: {self.JointsInfo[i][j]}')

    def getLinkInfo(self) -> tuple: # returns 8x list[float]
        # -1 : frame
        info = p.getLinkState(self.ID, -1, computeLinkVelocity = 1)            
        return info
    
    def showCOM(self, val: int)-> None:
        self.display_com_flag = val

    def get_total_com(self):
        total_mass = 0
        com_x, com_y, com_z = 0.0, 0.0, 0.0
        
        # Obliczenie dla bazy (link_index = -1)
        pos, _ = p.getBasePositionAndOrientation(self.ID)
        mass = p.getDynamicsInfo(self.ID, -1)[0]
        total_mass += mass
        com_x += pos[0] * mass
        com_y += pos[1] * mass
        com_z += pos[2] * mass
        
        # Obliczenie dla pozostałych linków
        for i in range(self.JointsNum):
            link_state = p.getLinkState(self.ID, i)
            link_pos = link_state[0] # worldLinkFramePosition
            link_mass = p.getDynamicsInfo(self.ID, i)[0]
            
            total_mass += link_mass
            com_x += link_pos[0] * link_mass
            com_y += link_pos[1] * link_mass
            com_z += link_pos[2] * link_mass
            
        if total_mass == 0:
            return pos # Zabezpieczenie przed dzieleniem przez 0
            
        return [com_x / total_mass, com_y / total_mass, com_z / total_mass]

    def draw_frame_com(self):
        self.updatePos()
        global_com = self.get_total_com()

        # p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 1)
        p.addUserDebugPoints(
            pointPositions=[self.Position, global_com, [global_com[0], global_com[1], 0]], 
            pointColorsRGB=[[1, 0, 0], [0, 1, 0], [0, 0, 1]], 
            pointSize=1*self.scale, 
            lifeTime=0.1
        )

class Simulation:
    def __init__(self, timestep:float = 1./240. ) -> None:
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0,0,-9.81)
        self.timestep = timestep
        p.setTimeStep(timestep)

        # 
        self.robot:Robot = None
        self.apply_force_flag: bool = True
        # self.apply_torque_flag: bool = False
        self.DisturbancesForce: float = 0 
        self.disturbances_timespan: float = 200.0

        self.is_on_flag = False

        
    def load_plane(self, urdf_path: str) -> int:
        self.ID = p.loadURDF(urdf_path)
        return self.ID

    def step_simulation(self):
        p.stepSimulation()
        time.sleep(self.timestep)

    def applyForce(self, force: float, timespan: float=200.0):
        if force == 0:
            self.apply_force_flag = False
        else:
            self.apply_force_flag = True
            self.DisturbancesForce = force
            self.disturbances_timespan = timespan
                
    def applyDisturbances(self):
        link: int = -1 # todo - losowanie roznych czesci robota
        
        # max_torque: float = 1.0 
        # torque = np.random.uniform(-max_torque, max_torque, size=3)
        # p.applyExternalTorque(ID, link, torqueObj=torque, flags=p.WORLD_FRAME)

        base_pos, _ = p.getBasePositionAndOrientation(self.robot.ID)
        position = list(base_pos)
        # position = np.random.uniform(0.0, 100.0, size=3)

        if self.apply_force_flag:
            max_force = self.DisturbancesForce 
            force = np.random.uniform(-max_force, max_force, size=3)
        #   force = [np.random.uniform(-max_force, max_force), 
        #              np.random.uniform(-max_force, max_force), 
        #              0.0]  
            
            p.applyExternalForce(self.robot.ID, link, forceObj=force, posObj=position, flags=p.WORLD_FRAME)
            
            scale = 0.05 
            end_pos = [
                position[0] + force[0] * scale,
                position[1] + force[1] * scale,
                position[2] + force[2] * scale
            ]
            p.addUserDebugLine(
                lineFromXYZ=position, 
                lineToXYZ=end_pos, 
                lineColorRGB=[1, 0.5, 0], 
                lineWidth=4, 
                lifeTime=0.5
            )

    def showDebugData(self):
        if self.robot.display_com_flag:
            self.robot.draw_frame_com()   
    
    def simulate(self, robot:Robot):
        self.robot = robot
        for i in range (10000):
            if self.is_on_flag:
                self.robot.robotSimulation()
            if i % 10 == 0:
                self.showDebugData()
            if self.apply_force_flag and i % int(self.disturbances_timespan) == 0:
                self.applyDisturbances()
            self.step_simulation()

    def disconnect(self):
        p.disconnect()


if __name__ == "__main__":
    sim = Simulation()
    sim.load_plane("plane.urdf")

    robot = Robot("robot.urdf", scale=9)
    robot.setPID(timestep=sim.timestep, 
                 Kp= 200,
                #  Ki= 1,
                #  Kd= 1
                 )
    # robot.printInfo()
    robot.showCOM(1)

    sim.is_on_flag = True
    # sim.applyForce(30)

    sim.simulate(robot)

    sim.disconnect()
