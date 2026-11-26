
'''
    builder.py

    build project with applied configuration
'''
from sim_components import *    # config
from sim_components.controllers import PID
from sim_components.sensors import IdealIMU, NoisyIMU, IdealEncoder, NoisyEncoder
from sim_components.motors.motor_drivers import SimpleDriver
from sim_components.motors.motor_types import IdealMotor, RealMotor


class RobotBuilder:
    _CONTROLLERS = {
        ControllerType.PID: lambda cfg: PID(cfg.max_output, cfg.pid_kp, cfg.pid_ki, cfg.pid_kd),
    }

    _SENSORS_IMU = {
        ImuType.IDEAL:       lambda rng: IdealIMU(), 
        ImuType.IMU_NOISY:   lambda rng: NoisyIMU(alpha=0.87, rng=rng),
    }
    _SENSORS_ENCODER = {
        EncoderType.IDEAL:   lambda: IdealEncoder(),
        EncoderType.NOISY:   lambda: NoisyEncoder(),
    }

    _DRIVERS = {
        DriverType.OPEN_LOOP:   lambda: SimpleDriver(),
        DriverType.CLOSED_LOOP: lambda: SimpleDriver(),
    }

    _MOTORS = {
        MotorType.IDEAL:lambda cfg, rng:  IdealMotor(kt= cfg.TORQUE_CONSTANT,
                                               max_current= cfg.MAX_CURRENT),
        MotorType.REAL: lambda cfg, rng:  RealMotor(kt= cfg.TORQUE_CONSTANT,
                                               max_current= cfg.MAX_CURRENT,
                                               viscous= cfg.VISCOUS_FRICTION,
                                               coulomb= cfg.COULOMB_FRICTION,
                                               deadband_ratio= cfg.DEADBAND_RATIO,
                                               delay_steps=cfg.DELAY_STEPS,
                                               noise= cfg.NOISE,
                                               asymmetry= cfg.ASYMMETRY,
                                               rng=rng ),
    }

    @classmethod
    def create_controller(cls, config: AppConfig):
        builder = cls._CONTROLLERS.get(config.controller)
        if not builder: raise ValueError(f"No implementation for {config.controller}")
        return builder(config.ctrl_params)
    
    @classmethod
    def create_imu_sensor(cls, config: AppConfig, rng=None):
        builder = cls._SENSORS_IMU.get(config.sensor_imu)
        if not builder: raise ValueError(f"No implementation for {config.sensor_imu}")
        return builder(rng)

    @classmethod
    def create_encoder_sensor(cls, config: AppConfig):
        builder = cls._SENSORS_ENCODER.get(config.sensor_encoder)
        if not builder: raise ValueError(f"No implementation for {config.sensor_encoder}")
        return builder()
    
    @classmethod
    def create_driver(cls, config: AppConfig):
        builder = cls._DRIVERS.get(config.driver)
        if not builder: raise ValueError(f"No implementation for {config.driver}")
        return builder()

    @classmethod
    def create_motor(cls, config: AppConfig, rng=None):
        builder = cls._MOTORS.get(config.motors)
        if not builder: raise ValueError(f"No implementation for {config.motors}")
        
        # motor_left = builder(config.motor_params)
        # motor_right = builder(config.motor_params)
        # return [motor_left, motor_right]
        return builder(config.motor_params, rng)