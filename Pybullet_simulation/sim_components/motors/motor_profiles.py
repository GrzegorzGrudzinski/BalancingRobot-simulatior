from sim_components.config import MotorConfig

MOTOR_PROFILES: dict[str, MotorConfig] = {
    "generic_bldc": MotorConfig(
        KV_RATING=360 ,
        MAX_CURRENT=3.0 ,
    ),
    "bldc_5010_360kV": MotorConfig(
        KV_RATING=360 ,
        MAX_CURRENT=3.0 ,  # ESP MOT_LIMIT
    ),
}