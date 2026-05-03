'''
    config.py

    Configure app parameters
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
class ImuType(Enum): 
    IDEAL = auto()
    IMU_NOISY = auto()
    IMU_PHYSICAL = auto()
class EncoderType(Enum):
    IDEAL = auto()
    NOISY = auto()
class MotorType(Enum):
    IDEAL = auto()
    REAL = auto()
    #
    BLDC_FOC = auto() 


@dataclass
class MotorConfig:
    # MAX_TORQUE: float = 0.023
    MAX_TORQUE: float = 0.08
    MAX_VEL: float = 40.0

    KV_RATING: float = 360.0
    VOLTAGE: float = 12.0
    TORQUE_CONSTANT: float = field(init=False)
    
    # FOC
    FOC_BANDWIDTH_HZ: float = 200.0 #
    
    #
    DEADBAND_RATIO: float = 0.02
    NOISE: float = 0.1
    ASYMMETRY: float = 0.1
    DELAY: int = 5 # (sim steps)

    def __post_init__(self):
        self.TORQUE_CONSTANT = 8.27 / self.KV_RATING


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
    sensor_imu: ImuType = ImuType.IDEAL
    sensor_encoder: EncoderType = EncoderType.IDEAL
    motors: MotorType = MotorType.REAL

    motor_profile: str = "generic_bldc"

    # default values
    ctrl_params: ControllerConfig = field(default_factory=ControllerConfig)
    sim_params: SimConfig = field(default_factory=SimConfig)
    # motor_params: MotorConfig = field(default_factory=MotorConfig)
    @property
    def motor_params(self):
        from sim_components.motors.motor_profiles import MOTOR_PROFILES
        temp = MOTOR_PROFILES.get(self.motor_profile)
        if not temp: raise ValueError(f"No implementation for {self.motor_profile}")         
        return temp   

@dataclass
class RobotConfig:
    controller: Any
    sensor_imu: Any
    sensor_encoder: Any
    motors: Any



'''
    self._show_com = False



'''