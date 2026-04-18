
'''
    builder.py
'''
from sim_components.config import AppConfig, ControllerType, SensorType, MotorType
from sim_components.controllers import PID
from sim_components.sensors import IdealSensor, NoisySensor
from sim_components.motors import IdealMotor, RealMotor, FOCMotor


class RobotBuilder:
    _CONTROLLERS = {
        ControllerType.PID: lambda cfg: PID(cfg.max_output, cfg.pid_kp, cfg.pid_ki, cfg.pid_kd),
    }

    _SENSORS = {
        SensorType.IDEAL:       lambda: IdealSensor(), 
        SensorType.IMU_NOISY:   lambda: NoisySensor(alpha=0.87),
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
    def create_sensor(cls, config: AppConfig):
        builder = cls._SENSORS.get(config.sensor)
        if not builder: raise ValueError(f"No implementation for {config.sensor}")
        return builder()

    @classmethod
    def create_motor(cls, config: AppConfig):
        builder = cls._MOTORS.get(config.motors)
        if not builder: raise ValueError(f"No implementation for {config.motors}")
        
        motor_left = builder(config.motor_params)
        motor_right = builder(config.motor_params)
        
        return [motor_left, motor_right]