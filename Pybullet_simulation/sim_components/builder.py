
'''
    builder.py
'''
from sim_components.config import AppConfig, ControllerType, ImuType, EncoderType, MotorType
from sim_components.controllers import PID
from sim_components.sensors import IdealIMU, NoisyIMU, IdealEncoder, NoisyEncoder
from sim_components.motors import IdealMotor, RealMotor, FOCMotor


class RobotBuilder:
    _CONTROLLERS = {
        ControllerType.PID: lambda cfg: PID(cfg.max_output, cfg.pid_kp, cfg.pid_ki, cfg.pid_kd),
    }

    _SENSORS_IMU = {
        ImuType.IDEAL:       lambda: IdealIMU(), 
        ImuType.IMU_NOISY:   lambda: NoisyIMU(alpha=0.87),
    }
    _SENSORS_ENCODER = {
        EncoderType.IDEAL:   lambda: IdealEncoder(),
        EncoderType.NOISY:   lambda: NoisyEncoder(),
    }

    _MOTORS = {
        MotorType.IDEAL:lambda cfg:  IdealMotor(max_torque= cfg.MAX_TORQUE),
        MotorType.REAL: lambda cfg:  RealMotor(max_torque= cfg.MAX_TORQUE,
                                               deadband_ratio= cfg.DEADBAND_RATIO,
                                               delay=cfg.DELAY,
                                               noise= cfg.NOISE ),
        MotorType.BLDC_FOC: lambda cfg: FOCMotor(noise= cfg.NOISE,
                                                 supply_voltage= cfg.VOLTAGE,
                                                 kv_rating= cfg.KV_RATING,
                                                 bandwidth_hz= cfg.FOC_BANDWIDTH_HZ,
                                                 torque_constant= cfg.TORQUE_CONSTANT ),
    }

    @classmethod
    def create_controller(cls, config: AppConfig):
        builder = cls._CONTROLLERS.get(config.controller)
        if not builder: raise ValueError(f"No implementation for {config.controller}")
        return builder(config.ctrl_params)

    @classmethod
    def create_imu_sensor(cls, config: AppConfig):
        builder = cls._SENSORS_IMU.get(config.sensor_imu)
        if not builder: raise ValueError(f"No implementation for {config.sensor_imu}")
        return builder()

    @classmethod
    def create_encoder_sensor(cls, config: AppConfig):
        builder = cls._SENSORS_ENCODER.get(config.sensor_encoder)
        if not builder: raise ValueError(f"No implementation for {config.sensor_encoder}")
        return builder()

    @classmethod
    def create_motor(cls, config: AppConfig):
        builder = cls._MOTORS.get(config.motors)
        if not builder: raise ValueError(f"No implementation for {config.motors}")
        
        motor_left = builder(config.motor_params)
        motor_right = builder(config.motor_params)
        
        return [motor_left, motor_right]