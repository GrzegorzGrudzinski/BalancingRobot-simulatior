# Self-Balancing Robot Simulator

A physics-based simulator of a two-wheeled self-balancing robot, built with **PyBullet**
(engineering thesis project). It is used to test controllers, sensor noise and motor models before
running them on the real robot, and to run the **robot's own firmware code in the loop (SiL)**.

Part of the Balancing Robot project:

| Repository | Role |
| --- | --- |
| [BalancingRobot-Main-Controller](https://github.com/GrzegorzGrudzinski/BalancingRobot-Main-Controller) | STM32 firmware; its `sil/` library is used here as the controller |
| [BalancingRobot-SimpleFOC-Motor-Controller](https://github.com/GrzegorzGrudzinski/BalancingRobot-SimpleFOC-Motor-Controller) | ESP32 + SimpleFOC motor driver |
| **BalancingRobot-simulatior** (this repo) | PyBullet simulator |
| [BalancingRobot-DataAnalyser](https://github.com/GrzegorzGrudzinski/BalancingRobot-DataAnalyser) | PC telemetry parser; same CSV format as the simulator log |

## Features

- URDF robot model (`robot.urdf`, meshes in `assets/`) with real-time centre-of-mass visualisation.
- Component-based setup (`RobotBuilder`): controllers, sensors and motors are chosen in `AppConfig`.
  - **Controllers:** Python PID, Python cascade PID, or the firmware C code through SiL (below).
  - **Sensors:** ideal or noisy IMU and encoders.
  - **Motors:** ideal or "real" (current-driven, deadband, noise, delay, friction), with motor profiles
    (`motor_profiles.py`, e.g. `bldc_5010_360kV`).
- External disturbance forces with configurable magnitude, duration and interval.
- Auto-reset when the robot falls.
- Reproducible runs (`seed`), GUI or headless batch mode, `Simulation.step()` API.
- CSV log in the same column format as `telemetry_frame.py --csv` from the DataAnalyser, so robot and
  simulation runs can be compared with the same scripts.

## Software-in-the-loop (SiL)

With `sim_params.sil_dir` set, the controller is the firmware C code built as a shared library in
`BalancingRobot-Main-Controller/sil`:

| Setting | Controller |
| --- | --- |
| `sil_dir = None` | Python controller from `AppConfig.controller` |
| `sil_dir = ".../sil"`, `sil_full_core = False` | firmware cascade only (`controllers/c_core.py`) |
| `sil_dir = ".../sil"`, `sil_full_core = True` | whole STM32 core: odometry, state machine, PC commands, cascade (`controllers/full_core.py`) |

Build the library first (in the main-controller repo): `make -C sil lib`.

> **Note:** the SiL adapters (`controllers/c_core.py`, `controllers/full_core.py`) are a temporary
> prototype written with AI assistance to try out the approach; they will be rewritten.

## Setup

The simulator expects the main-controller repo next to it (only needed for SiL):

```
Balancing_Robot/
  BalancingRobot-simulatior/
  BalancingRobot-Main-Controller/
```

```bash
cd BalancingRobot-simulatior
python3 -m venv .venv
.venv/bin/pip install -r Pybullet_simulation/requirements.txt
```

## Running

Run from the repository root (paths to `robot.urdf`, `logs/` and `sil_dir` are relative to it):

```bash
.venv/bin/python Pybullet_simulation/main.py
```

Edit `AppConfig` and `sim_params` in `Pybullet_simulation/main.py` to change the controller,
sensors, motor model, duration, GUI/headless mode, disturbances and the log file.

Whole-core SiL check (standstill, then a 0.3 m position step from the "PC", headless):

```bash
.venv/bin/python sil_full_core_check.py ../BalancingRobot-Main-Controller/sil
```

## Repository layout

```
Pybullet_simulation/
  main.py                  entry point and configuration
  sim_components/
    config.py              AppConfig, enums, parameters
    builder.py             RobotBuilder (component factory)
    runner.py              AppRunner: builds and runs the simulation
    sim.py, robot.py       PyBullet world and robot
    controllers/           PID, cascade PID, SiL adapters (c_core.py, full_core.py)
    sensors/               IMU and encoder models
    motors/                motor models, profiles, drivers
    logger.py              CSV logger
robot.urdf, assets/        robot model and meshes
resources/                 UML, state machine and call hierarchy diagrams
sil_full_core_check.py     SiL check script
```

## Planned

- Hardware-in-the-loop and a "mirror" mode (robot pose from live telemetry); `RunMode.PHYSICAL`
  is not implemented yet.
- Closing the sim-to-real gap: battery and ESP32 model, masses and friction from measurements.
- Trajectory following (lines, arcs, circles) and comparison of the same mission in simulation
  and on the robot.
- Alternative controllers (LQR, RL).

## License

MIT, see [LICENSE](LICENSE).
