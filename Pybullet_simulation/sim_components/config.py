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
    CASCADE_PID = auto()
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
class DriverType(Enum):
    OPEN_LOOP = auto()
    CLOSED_LOOP = auto()

@dataclass
class ControllerConfig:
    """Global parameters for every controller"""
    pid_kp: float = 0.8    # [A/deg]       CASCADE_PITCH_KP
    pid_ki: float = 0.0    # [A/(deg*s)]
    pid_kd: float = 0.04   # [A/(deg/s)]   CASCADE_PITCH_KD

    pid_ang_Kp: float = 1.0
    pid_ang_Ki: float = 0.0
    pid_ang_Kd: float = 0.0
    
    max_output: float = 2.5  # [A] CASCADE_PITCH_MAX_OUT
    max_pitch_angle: float = 15.0

@dataclass
class MotorConfig:
    """Motor input is current [A]; torque = Kt * I - friction."""
    KV_RATING: float = 360.0
    TORQUE_CONSTANT: float = field(init=False)  # [Nm/A] 8.27 / KV
    MAX_CURRENT: float = 3.0                    # [A] ESP MOT_LIMIT

    # friction inside the motor, from the coast-down test on the robot; 0 = none
    VISCOUS_FRICTION: float = 0.0  # [Nm*s/rad]
    COULOMB_FRICTION: float = 0.0  # [Nm]

    DEADBAND_RATIO: float = 0.0
    NOISE: float = 0.0             # [A] std per physics step
    ASYMMETRY: float = 0.05        # +- relative spread of Kt between motors
    DELAY_STEPS: int = 1  # command delay in physics steps (1 ms); measured STM->ESP rtt ~1.8 ms
    def __post_init__(self):
        self.TORQUE_CONSTANT = 8.27 / self.KV_RATING
        if not isinstance(self.DELAY_STEPS, int) or self.DELAY_STEPS < 0:
            raise ValueError("DELAY_STEPS must be an int >= 0 (physics steps)")

@dataclass
class SimConfig:
    """Environment settings"""
    # debug info
    show_wireframe: bool = False
    camera_tracking: bool = True
    show_com: bool = True
    # external disturbances 
    disturb_force: float = 30.0
    disturb_interval_s: float = 0.83
    disturb_time_s: float = 0
    #
    # wheel losses inside PyBullet. 
    joint_damping: float = 0.0       # [Nm*s/rad] torque = -c * joint velocity
    rolling_friction: float = 0.0    # [m] lever arm, torque = coef * normal force
    spinning_friction: float = 0.001
    lateral_friction: float = 1.2
    friction_variance: float = 0.2   # +- relative spread of lateral friction

    # run
    gui: bool = True
    realtime: bool = True        # sleep to keep wall-clock time; False -> as fast as possible
    seed: int | None = 0         # None -> different random run every time
    duration_s: float = 60.0
    log_csv: str | None = None   # path -> CSV in telemetry_frame.py column format

    # SiL: firmware cascade in C instead of the Python PID
    sil_dir: str | None = None          # path to BalancingRobot-Main-Controller/sil, None = Python PID
    sil_friction_comp: bool = False     # frictionComp() after the cascade, like FRIC_COMP_ENABLE
    sil_full_core: bool = False         # True: whole STM core (robotCoreStep), False: cascade only

    
    # start / reset
    start_pitch_deg: float | None = None  # None -> random in +-start_pitch_range_deg
    start_pitch_range_deg: float = 3.0    # firmware START_ANGLE



@dataclass
class AppConfig:
    mode: RunMode = RunMode.SIMULATION
    controller: ControllerType = ControllerType.PID
    sensor_imu: ImuType = ImuType.IDEAL
    sensor_encoder: EncoderType = EncoderType.IDEAL
    driver: DriverType = DriverType.OPEN_LOOP
    motors: MotorType = MotorType.IDEAL

    motor_profile: str = "generic_bldc"

    # default values
    sim_params: SimConfig = field(default_factory=SimConfig)
    ctrl_params: ControllerConfig = field(default_factory=ControllerConfig)
    # driver_params: MotorConfig = field(default_factory=MotorConfig)
    motor_params: MotorConfig = field(default_factory=MotorConfig)
    # @property
    # def motor_params(self):
    #     from sim_components.motors.motor_profiles import MOTOR_PROFILES
    #     temp = MOTOR_PROFILES.get(self.motor_profile)
    #     if not temp: raise ValueError(f"No implementation for {self.motor_profile}")         
    #     return temp   

@dataclass
class RobotConfig:
    controller: Any
    sensor_imu: Any
    sensor_encoder: Any
    driver: Any
    motors: Any



'''
    self._show_com = False



'''