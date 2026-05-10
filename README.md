# Self-Balancing Robot Simulator

## Overview

A modular, physics-based simulation environment for a two-wheeled self-balancing robot. Built with **PyBullet**, this simulator allows for rapid testing of various control algorithms, sensor noise models, and motor characteristics before real-world deployment.

The project features a highly configurable, component-based architecture where controllers, sensors, and actuators can be easily swapped or modified using a central configuration system.

## Architecture & Features

The simulation is built around modular components created via a `RobotBuilder`:

* **Controllers:** Easily interchangeable control logic (currently implemented: PID).
* **Sensors:** - **IMU:** Configurable as Ideal or Noisy (with adjustable alpha filtering).
* **Encoders:** Configurable as Ideal or Noisy.


* **Actuators (Motors & Drivers):** Support for Ideal, Real (with configurable deadband, noise, and delay), and FOC (BLDC) motor profiles.
* **Physics Engine:** PyBullet handles the rigid body dynamics, custom friction/damping parameters, and external disturbance forces.

## Configuration

The environment is initialized via `AppConfig` in `main.py`, allowing fine-tuning of:

* Control loop gains (`pid_kp`, `pid_ki`, `pid_kd`).
* Motor parameters (`MAX_TORQUE`, `NOISE`, `DEADBAND_RATIO`).
* Disturbance parameters (force magnitude and intervals) for stability testing.

---

## Project Status

### Done

* [x] PyBullet environment setup with URDF robot model loading.
* [x] Modular component factory.
* [x] Base classes and implementations for Ideal/Noisy components.
* [x] Real-time Center of Mass (CoM) calculation and debug visualization.
* [x] External disturbances with configurable duration.
* [x] Auto-reset mechanism when the robot falls (exceeds max angle).

### To Do

- [ ] **Advanced motor control:** Implement a virtual motor control algorithms (`driver class`) and add detailed motor parameters (e.g., acceleration profiles, electrical time constants).
- [ ] **Encoder Refinement:** Improve the virtual encoder models (e.g., ABI signal emulation, resolution limits, and realistic noise).
* [ ] **Sim-to-Real Messaging:** Add a communication system to interact with the physical robot (Hardware-in-the-Loop).
* [ ] **Close sim2real gap**
* [ ] **State Synchronization:** Implement `updatePos` (Sim/IMU) and `resetPos` with a STOP signal for the physical robot.
- [ ] **Simulation Analytics & Telemetry:** Add built-in data logging and analysis tools to evaluate controller performance over time.
- [ ] **GUI Dashboard:** Develop a Graphical User Interface for real-time monitoring, state visualization, and live parameter tuning.
- [ ] **Advanced Control & AI:** Implement alternative control strategies (e.g., LQR, MPC) and explore Reinforcement Learning (RL) agents for balancing.
* [ ] **Remote Tuning:** Allow updating PID parameters dynamically between Sim and MCU.