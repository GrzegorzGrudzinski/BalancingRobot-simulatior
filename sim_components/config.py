'''
    config.py
'''

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any

class RunMode(Enum):
    SIMULATION = auto()
    PHYSICAL = auto()
class ControllerType(Enum): 
    PID = auto()
    CONTROLLER_PHYSICAL = auto()
class SensorType(Enum): 
    IDEAL = auto()
    IMU_NOISY = auto()
    IMU_PHYSICAL = auto()
class MotorType(Enum):
    IDEAL = auto()
    REAL = auto()
    #
    BLDC_FOC = auto() 


@dataclass
class MotorConfig:
    MAX_TORQUE: float = 0.023
    MAX_VEL: float = 40.0

    KV_RATING: float = 360.0
    VOLTAGE: float = 12.0
    TORQUE_CONSTANT: float = 8.27 / 360.0 
    
    # FOC
    FOC_BANDWIDTH_HZ: float = 200.0 #
    
    #
    DEADBAND_RATIO: float = 0.02
    NOISE: float = 0.1
    ASYMMETRY: float = 0.1
    DELAY: int = 5 # (sim steps)


@dataclass
class ControllerConfig:
    """Global parameters for every controller"""
    pid_kp: float = 15.0
    pid_ki: float = 0.0
    pid_kd: float = 1.0
    max_output: float = 40.0 # MotorConfig.MAX_VEL


@dataclass
class SimConfig:
    """Environment settings"""
    # debug info
    show_wireframe: bool = False
    camera_tracking: bool = True
    show_com: bool = True
    # external disturbances 
    disturb_force: float = 30.0
    disturb_interval: int = 200

@dataclass
class AppConfig:
    mode: RunMode = RunMode.SIMULATION
    controller: ControllerType = ControllerType.PID
    sensor: SensorType = SensorType.IDEAL
    motors: MotorType = MotorType.REAL

    # default values
    ctrl_params: ControllerConfig = field(default_factory=ControllerConfig)
    sim_params: SimConfig = field(default_factory=SimConfig)
    motor_params: MotorConfig = field(default_factory=MotorConfig)

@dataclass
class RobotConfig:
    controller: Any
    sensor: Any
    motors: list[Any]



'''
    self._show_com = False



'''