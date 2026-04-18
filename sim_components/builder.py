from sim_components.config import AppConfig, ControllerType, SensorType
from sim_components.controllers import PID
from sim_components.sensors import IdealSensor, NoisySensor


class RobotBuilder:
    _CONTROLLERS = {
        ControllerType.PID: lambda cfg: PID(cfg.max_output, cfg.pid_kp, cfg.pid_ki, cfg.pid_kd),
    }

    _SENSORS = {
        SensorType.IDEAL:     lambda: IdealSensor(), 
        SensorType.IMU_NOISY: lambda: NoisySensor(alpha=0.87),
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