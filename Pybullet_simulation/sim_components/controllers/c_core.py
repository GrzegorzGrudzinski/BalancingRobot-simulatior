"""SiL adapter: the firmware cascade in C (sil/libcontrol_core.so) as the robot controller."""
import sys
from pathlib import Path
import math # import radians

class CCoreController:
    """The STM32 control path in BALANCING: controlRun() and, optionally, frictionComp() from the mean wheel speed (14.12).
    State and command in firmware units (deg, deg/s, m, m/s, rad/s, A), as returned by Simulation.step()."""

    def __init__(self, sil_dir: str, params=None, friction=None, wheel_lpf_tf: float = 0.05,
                 wheel_radius: float = 0.055, wheel_base: float = 0.200):
        """params: control_core.CascadeParams, None = defaults (= config.h).
        friction: control_core.FrictionParams or True (= defaults) -> compensation on, None/False -> off
        (FRIC_COMP_ENABLE 0).
        wheel_lpf_tf: [s] low-pass on the wheel speeds like the ESP (MOT_VEL_FILTER), 0 = ideal."""
        sil = Path(sil_dir).resolve()
        if str(sil) not in sys.path:
            sys.path.insert(0, str(sil))  # sil/ is in the main-controller repo, not a package
        from control_core import CascadeController#, FrictionParams
        self._c = CascadeController(params, str(sil / "libcontrol_core.so"))
        # self._friction = FrictionParams() if friction is True else (friction or None)
        self._tf = wheel_lpf_tf
        self._r = wheel_radius
        self._w = [0.0, 0.0]          # filtered wheel speeds [rad/s], what the STM gets from the ESP
        self._armed = False
        self._target = 0.0
        self._r = wheel_radius
        self._b = wheel_base

    def _wheels(self, state: dict, dt: float) -> None:
        for k, key in enumerate(("mot1_vel", "mot2_vel")):
            a = 1.0 if self._tf <= 0 else dt / (self._tf + dt)
            self._w[k] += a * (state[key] - self._w[k])

    def _fw(self, state: dict) -> tuple[float, float, float, float, float, float]:
        # velocity from the (filtered) wheel speeds, yaw from the wheel positions, as robot_core.c on the STM32
        yaw = (state["right_wheel_pos"] - state["left_wheel_pos"]) * self._r / self._b      # [rad]
        yaw_rate = -math.radians(state["yaw_rate"])  # [rad/s], gyro Z is opposite to the wheel yaw (robot_core.c)
        return (state["pitch"], state["pitch_rate"], state["position"],
                0.5 * (self._w[0] + self._w[1]) * self._r, yaw, yaw_rate)

    def step(self, state: dict, dt: float) -> tuple[float, float]:
        self._wheels(state, dt)
        if not self._armed:          # = entering BALANCING: PID reset, target_position = position
            self._c.reset(*self._fw(state))
            self._target = state["position"]
            self._armed = True
        out_l, out_r = self._c.step(*self._fw(state), dt=dt)
        # if self._friction is not None:   # robot_app.c: after controlRun(), from the mean wheel speed (14.12)
        #     w = 0.5 * (self._w[0] + self._w[1])
        #     out_l += self._c.friction_comp(self._friction, out_l, w)
        #     out_r += self._c.friction_comp(self._friction, out_r, w)
        return out_l, out_r

    def step_position(self, dx: float) -> None:
        """Same as POS_STEP from the PC (step 17)."""
        self._target += dx
        self._c.set_target(target_pitch=0.0, target_position=self._target)

    def reset(self) -> None:
        self._armed = False

    @property
    def debug(self) -> dict:
        """Loop outputs and integrals of the last step (telemetry names) + target_position."""
        return {**self._c.debug, "target_position": self._target}
