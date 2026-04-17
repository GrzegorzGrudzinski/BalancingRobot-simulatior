'''
    config.py
'''

from dataclasses import dataclass

@dataclass(frozen=True)
class MotorConfig:
    MAX_TORQUE: float = 0.08
    MAX_VEL: float = 40.0
    DEADBAND_RATIO: float = 0.3