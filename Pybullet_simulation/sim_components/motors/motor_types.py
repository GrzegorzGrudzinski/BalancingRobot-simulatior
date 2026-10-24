import pybullet as p
import numpy as np
from collections import deque
from abc import ABC, abstractmethod

class BaseMotor(ABC):
    """command() is called by the controller (200 Hz), apply() before every physics step."""
    def __init__(self):
        self._target = [0.0, 0.0]

    def command(self, val: list[float]) -> None:
        self._target = [float(val[0]), float(val[1])]
    
    @abstractmethod
    def apply(self, id, joint_indices: list[int]) -> None:
        """Send torque to PyBullet. TORQUE_CONTROL lasts only one stepSimulation(),
        so this must run before every step (zero-order hold between controller updates)."""
        pass

    def reset(self) -> None:
        self._target = [0.0, 0.0]

def _clamp(x: float, limit: float) -> float:
    return max(min(x, limit), -limit)     
class IdealMotor(BaseMotor):
    def __init__(self, kt: float, max_current: float):
        super().__init__()
        self.kt = kt
        self.max_current = max_current

    def apply(self, id, joint_indices: list[int]) -> None:
        torque = [self.kt * _clamp( i, self.max_current) for i in self._target]
        p.setJointMotorControlArray(id, joint_indices, p.TORQUE_CONTROL, forces=torque)

class RealMotor(BaseMotor):
    FRICTION_SMOOTH_W = 0.1 # [rad/s] wheel speed band where Coulomb friction ramps up via tanh instead of jumping
    def __init__(self, kt: float, max_current: float, deadband_ratio: float = 0.0,
                 noise: float = 0.0, delay_steps: int = 0, asymmetry: float = 0.0,
                 viscous: float = 0.0, coulomb: float = 0.0, rng=None ):
        super().__init__()
        self.rng = rng if rng is not None else np.random.default_rng()
        self.base_kt = kt
        self.max_current = max_current
        self.deadband = deadband_ratio * max_current
        self.noise = noise
        self.asymmetry = asymmetry
        self.viscous = viscous
        self.coulomb = coulomb

        # buffer[0] is the command from delay_steps physics steps ago
        self.delay_steps = int(delay_steps)
        self.buffer_l = deque(maxlen=self.delay_steps + 1)
        self.buffer_r = deque(maxlen=self.delay_steps + 1)
        self.reset()

    def apply(self, id, joint_indices: list[int]) -> None:
        self.buffer_l.append(self._target[0])
        self.buffer_r.append(self._target[1])
        
        torque = []
        for i_cmd, kt, joint in ((self.buffer_l[0], self.kt_l, joint_indices[0]),
                                 (self.buffer_r[0], self.kt_r, joint_indices[1])):
            i = _clamp(i_cmd, self.max_current)
            if abs(i) < self.deadband:
                i = 0.0
            if self.noise > 0:
                i += self.rng.normal(0, self.noise)
            w = p.getJointState(id, joint)[1]  # wheel speed relative to the body
            # tau = Kt*I - b*w - tau_c*sign(w): motor torque minus viscous and Coulomb friction,
            # sign(w) smoothed with tanh to avoid chatter around w = 0
            torque.append(kt * i - self.viscous * w - self.coulomb * np.tanh(w / self.FRICTION_SMOOTH_W))

        p.setJointMotorControlArray(id, joint_indices, p.TORQUE_CONTROL, forces=torque)
        
    def reset(self) -> None:
        super().reset()# zero the command so the last one before reset is not executed
        # flush the delay queue with zeros (old commands drop out of the deque)
        self.buffer_l.extend([0.0] * (self.delay_steps + 1))
        self.buffer_r.extend([0.0] * (self.delay_steps + 1))

        # new Kt spread every episode (for assymetry): same current gives slightly different torque per wheel,
        # drawn from self.rng so it is reproducible for a given seed
        self.kt_l = self.base_kt * (1.0 + self.rng.uniform(-self.asymmetry, self.asymmetry))
        self.kt_r = self.base_kt * (1.0 + self.rng.uniform(-self.asymmetry, self.asymmetry))