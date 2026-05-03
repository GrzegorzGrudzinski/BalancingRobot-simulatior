from sim_components.config import MotorConfig

MOTOR_PROFILES: dict[str, MotorConfig] = {
    "generic_bldc": MotorConfig(
        KV_RATING=360 ,
        VOLTAGE=12 ,
        MAX_TORQUE=0.08 ,
        DEADBAND_RATIO=0.02 ,
    ),
    "bldc_5010_360kV": MotorConfig(
        KV_RATING=360 ,
        VOLTAGE=12 ,
        MAX_TORQUE=0.08 ,
        DEADBAND_RATIO=0.02 ,
        # MAX_I = 1 # A
    ),
}