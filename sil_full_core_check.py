"""Whole STM core in PyBullet: standstill, then a 0.3 m position step from the "PC".
Run from BalancingRobot-simulatior:  .venv/bin/python sil_full_core_check.py ../BalancingRobot-Main-Controller/sil"""
import sys
sys.path.insert(0, "Pybullet_simulation")
import numpy as np
from sim_components import *
from sim_components.sim import Simulation
from sim_components.robot import Robot
from sim_components.runner import AppRunner
from sim_components.controllers.full_core import FullCoreController

sil_dir = sys.argv[1]
config = AppConfig(mode=RunMode.SIMULATION, controller=ControllerType.PID, sensor_imu=ImuType.IDEAL,
                   sensor_encoder=EncoderType.IDEAL, driver=DriverType.OPEN_LOOP, motors=MotorType.REAL,
                   motor_profile="bldc_5010_360kV")
sp = config.sim_params
sp.seed, sp.gui, sp.realtime = 0, False, False
sp.start_pitch_deg = 2.0
config.motor_params.DELAY_STEPS = 1
app = AppRunner(config)                        # builds robot_cfg and the RNGs as main.py does
sim = Simulation(gui=False, realtime=False, rng=app._rng_sim)
sim.load_plane("plane.urdf")
robot = Robot("./robot.urdf", config=app.robot_cfg, scale=1, rng=app._rng_robot, sim_cfg=sp)
sim.attach_robot(robot)
ctrl = FullCoreController(sil_dir)
sim.start()
dt = 0.005
state = robot.read_state()
log = []
for k in range(int(25.0 / dt)):
    if k == int(10.0 / dt):
        ctrl.step_position(0.3)
    cmd = ctrl.step(state, dt)
    state = sim.step(cmd)
    state.update(ctrl.debug)
    log.append((state["time"], state["pitch"], state["position"], state["target_position"], state["core_state"], state["fallen"]))
sim.disconnect()
a = np.array(log)
t, pitch, pos, tgt, st, fallen = a.T
bal = st == 3
first = t[bal][0] if bal.any() else float("nan")
print(f"BALANCING from {first:.2f} s, {np.mean(bal) * 100:.0f}% of the time, falls {int(fallen.sum())}")
w = (t > 5) & (t < 10)
print(f"standstill 5-10 s: |pitch| max {np.max(np.abs(pitch[w])):.2f} deg, position drift {np.ptp(pos[w]) * 100:.1f} cm")
w2 = t > 10.0
print(f"step +0.3 m at 10 s: target {tgt[w2][0] - pos[t <= 10.0][-1]:+.3f} m relative, position at 25 s {pos[-1] - pos[t <= 10.0][-1]:+.3f} m, "
      f"overshoot {(np.max(pos[w2]) - tgt[w2][-1]) * 100:.1f} cm, |pitch| max {np.max(np.abs(pitch[w2])):.2f} deg")
w3 = t > 20
print(f"20-25 s: |position - target| {np.max(np.abs(pos[w3] - tgt[w3])) * 100:.1f} cm")
