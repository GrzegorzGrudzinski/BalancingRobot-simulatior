"""SiL adapter, whole STM32 core: robotCoreStep() from sil/libcontrol_core.so as the robot controller.

Compared with c_core.py (cascade only) the simulator now runs the firmware wheel odometry (with the
pitch-rate correction), the state machine (handshake, FALLEN, ERROR) and the PC command handling.
The ESP is a stub (robot_core.EspStub) until its model comes with step 16.16.

Pitch angle: the core does not estimate it yet (on the robot: Kalman filter in mpu6050.c), so it is an
input. Here it is the pitch the simulator reports (state["pitch"]: true angle with ImuType.IDEAL).
The raw accelerometer sample is only copied into the telemetry frame.
"""
import math
import struct
import sys
from pathlib import Path


class FullCoreController:
    """step(state, dt) -> (out_l, out_r) [A], reset(), debug, step_position(dx), like CCoreController."""

    FB_AGE_US = 3000            # typical feedback age on the robot (fb_age_us, 27-28.10)

    def __init__(self, sil_dir: str, wheel_lpf_tf: float = 0.05, auto_start: bool = True):
        sil = Path(sil_dir).resolve()
        if str(sil) not in sys.path:
            sys.path.insert(0, str(sil))
        import robot_core as rc
        self._rc = rc
        self._sil = sil
        self._tf = wheel_lpf_tf
        self._auto_start = auto_start
        self._core = None
        self._esp = rc.EspStub()
        self._w = [0.0, 0.0]
        self._cmd_seq = 0
        self._t_ms = 0
        self._press = False
        self._pc_on = False         # True after the first PC command: heartbeat every 100 ms, like pc_step.py
        self._cycles = 0
        self.debug = {}
    def _imu(self, state: dict) -> dict:
        a = math.radians(state["pitch"])
        return {"pitch": state["pitch"],                             # [deg] the angle the firmware gets from its filter
                "ax": 0.0, "ay": math.sin(a), "az": -math.cos(a),   # [g] sensor upside down; telemetry only
                "gx": state["pitch_rate"], "gy": 0.0, "gz": state["yaw_rate"], "imu_ok": 1}

    def _input(self, state: dict, dt: float) -> dict:
        a = 1.0 if self._tf <= 0 else dt / (self._tf + dt)
        for k, key in enumerate(("mot1_vel", "mot2_vel")):
            self._w[k] += a * (state[key] - self._w[k])          # ESP velocity low-pass (MOT_VEL_FILTER)
        d = self._imu(state)
        d.update(fb_status=self._esp.feedback_status(), fb_error=0, fb_rx_seq=self._esp.rx_seq,
                 fb_tx_seq=self._esp.tx_seq, left_pos=state["left_wheel_pos"], right_pos=state["right_wheel_pos"],
                 left_vel=self._w[0], right_vel=self._w[1], left_A=self._esp.target[0], right_A=self._esp.target[1],
                 fb_age_us=self.FB_AGE_US, rtt_us=1800, cmd_seq=self._cmd_seq, time_ms=self._t_ms,
                 button=1 if self._press else 0)
        return d

    def _create(self, state: dict) -> None:
        self._params = self._rc.params_default(str(self._sil / "libcontrol_core.so"))
        self._core = self._rc.RobotCore(self._params, str(self._sil / "libcontrol_core.so"))

    def _start_held(self, state: dict, dt: float) -> None:
        """Start as on the robot: someone holds it upright and still, presses the button and lets go in
        BALANCING. The handshake (START_INIT, HELLO, INIT, START) takes ~0.3-0.5 s of firmware time;
        here it runs in zero simulation time on an upright, still copy of the state."""
        held = dict(state, pitch=0.0, pitch_rate=0.0, yaw_rate=0.0, mot1_vel=0.0, mot2_vel=0.0)
        for _ in range(2000):                                   # 10 s of firmware time at most
            out = self._cycle(held, dt)
            if self._rc.STATES[out["state"]] == "BALANCING" and self._esp.state == "RUN":
                return

    def _cycle(self, state: dict, dt: float) -> dict:
        self._t_ms += round(dt * 1000)
        self._cycles += 1
        if self._pc_on and self._cycles % 20 == 0:
            self._core.pc_command(self._rc.PC_CMD_HEARTBEAT)   # without it PC_TIMEOUT resets the target
        out = self._core.step(self._input(state, dt))
        self._press = False
        if out["send_motor_cmd"]:
            self._cmd_seq += 1
            self._esp.command(out["motor_cmd"], out["value1"], out["value2"], self._cmd_seq)
        if self._auto_start and self._rc.STATES[out["state"]] == "STANDBY":
            self._press = True                                  # "button" in the next cycle: start again
        return out

    def command_pos(self, dx: float) -> None:
        """Wanted position step [m], called every cycle (0 = no key). One SET_TARGET per 100 ms at most."""
        if dx != 0.0 and self._cycles % 20 == 0:
            self.step_position(dx)      # also sets _pc_on, so the heartbeat in _cycle() keeps the target alive
    def command_yaw(self, dz: float) -> None:
        """Wanted yaw step [rad], called every cycle (0 = no key). One SET_TARGET per 100 ms at most."""
        if dz != 0.0 and self._cycles % 20 == 0:
            self.step_yaw(dz)
    
    def step(self, state: dict, dt: float) -> tuple[float, float]:
        if self._core is None:
            self._create(state)
        if self._auto_start and self._esp.state != "RUN":
            self._start_held(state, dt)
        out = self._cycle(state, dt)
        frame = self._core.telemetry()
        tgt = struct.unpack_from("<4f", frame, 111)    
        dbg = struct.unpack_from("<8f", frame, 127)             # ControlDebug_t
        self.debug = {"core_state": out["state"], "target_position": tgt[2],
                      "position_output": dbg[0], "velocity_output": dbg[1], "pitch_setpoint": dbg[2],
                      "pitch_output": dbg[3], "target_yaw" : tgt[1], "yaw_output": dbg[4], "i_velocity": dbg[6]}
        return self._esp.target if self._esp.state == "RUN" else (0.0, 0.0)

    def step_position(self, dx: float) -> None:
        """SET_TARGET TARGET_POS (relative step) from the PC, handled by the firmware code."""
        self._core.pc_command(self._rc.PC_CMD_SET_TARGET, self._rc.PC_PARAM_TARGET_POS, dx)
        self._pc_on = True
    
    def step_yaw(self, dz: float) -> None:
        """SET_TARGET TARGET_POS (relative step) from the PC, handled by the firmware code."""
        self._core.pc_command(self._rc.PC_CMD_SET_TARGET, self._rc.PC_PARAM_TARGET_YAW, dz)
        self._pc_on = True

    def reset(self) -> None:
        """PyBullet put the robot back upright: the firmware sees FALLEN -> STOP -> STANDBY and starts again."""
        pass
