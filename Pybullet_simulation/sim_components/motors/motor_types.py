import pybullet as p
import numpy as np
from collections import deque
from abc import ABC, abstractmethod

class BaseMotor(ABC):
    @abstractmethod
    def move(self, val: list[float], id, joint_indices: list[int]) -> None:
        pass
    def reset(self) -> None:
        pass
        
class IdealMotor(BaseMotor):
    def __init__(self, max_torque: float):
        self.max_torque = max_torque

    def move(self, val: list[float], id, joint_indices: list[int]) -> None:
        l_torque = max(min(val[0], self.max_torque), -self.max_torque)
        r_torque = max(min(val[1], self.max_torque), -self.max_torque)

        p.setJointMotorControlArray(
            id, 
            joint_indices, 
            p.TORQUE_CONTROL, 
            # targetVelocities = target_val,
            forces = [l_torque, r_torque]
        )



class RealMotor(BaseMotor):
    def __init__(self, max_torque: float, deadband_ratio: float,
                 noise: float = 0.0, delay: int = 0, 
                 asymmetry: float = 0.0 ):
        self.rng = np.random.default_rng()
        self.deadband_ratio = deadband_ratio
        self.noise = noise
        self.asymmetry = asymmetry
        self.base_max_torque = max_torque
        
        self.delay = delay
        self.buffer_l = deque( [0.0]*max(1,delay), maxlen=max(1,delay) )
        self.buffer_r = deque( [0.0]*max(1,delay), maxlen=max(1,delay) )
        self.reset()

    def move(self, val: list[float], id, joint_indices: list[int]) -> None:
        self.buffer_l.append(val[0])
        self.buffer_r.append(val[1])
        
        delayed_torque_l = self.buffer_l[0] if self.delay > 0 else val
        delayed_torque_r = self.buffer_r[0] if self.delay > 0 else val

        l_torque = max(min(delayed_torque_l, self.max_torque_l), -self.max_torque_l)
        r_torque = max(min(delayed_torque_r, self.max_torque_r), -self.max_torque_r)
        
        if abs(l_torque) < self.deadband_l:
                l_torque = 0.0
        if abs(r_torque) < self.deadband_r:
                r_torque = 0.0
        
        if self.noise > 0:
            l_torque += self.rng.normal(0, self.noise)
            r_torque += self.rng.normal(0, self.noise)
        
        p.setJointMotorControlArray(
            id, 
            joint_indices, 
            p.TORQUE_CONTROL, 
            # targetVelocities = target_val,
            forces = [l_torque, r_torque]
        )
        
    def reset(self) -> None:
        self.buffer_l.extend( [0.0] * max(1, self.delay) )
        self.buffer_r.extend( [0.0] * max(1, self.delay) )

        bias1 = self.rng.uniform(-self.asymmetry, self.asymmetry)
        bias2 = self.rng.uniform(-self.asymmetry, self.asymmetry)
        
        self.max_torque_l = self.base_max_torque * (1.0 + bias1)
        self.max_torque_r = self.base_max_torque * (1.0 + bias2)
        
        self.deadband_l = self.deadband_ratio * self.max_torque_l
        self.deadband_r = self.deadband_ratio * self.max_torque_r




'''    TODO      '''
class FOCMotor(BaseMotor):
    def __init__(self, torque_constant: float, kv_rating: float, supply_voltage: float, 
                 bandwidth_hz: float, dt: float = 1/240.0, 
                 noise: float = 0.001, delay_steps: int = 1, asymmetry_variance: float = 0.0):
        
        pass

    def reset(self) -> None:
        pass

    # UWAGA: target_val to teraz wartość z PID (np. 1.0 Amper), a nie gotowe Nm!
    def move(self, target_val: float, encoder_val: list[float] = None, **kwargs) -> float:
        pass