from sim_components.config import RobotConfig
# import sensors

class control_state: 
    def __init__(self, id , config: RobotConfig ):
        self._config = config
        self._id = id

        self.pitch: float = 0.0
        self.pitch_rate: float = 0.0
        self.yaw: float = 0.0
        self.yaw_rate: float = 0.0
        self.position: float = 0.0
        self.velocity: float = 0.0
        
        self.left_wheel_pos: float = 0.0
        self.right_wheel_pos: float = 0.0
    


    def UpdateState(self):
        self.pitch = self._config.sensor_imu.read(self._id, )

        [left_wheel, right_wheel] = self._config.sensor_encoder.read_all()