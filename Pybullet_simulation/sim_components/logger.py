'''
    logger.py

    CSV log with the column names of telemetry_frame.py --csv, so one script can plot sim and robot
'''

import csv
import os

# (csv column, state key): firmware columns first, simulation-only columns at the end
_COLUMNS = [
    ("timestamp_ms", None), ("robot_state", None),
    ("pitch", "measured_pitch"), ("pitch_rate", "pitch_rate"),
    ("position", "position"), ("velocity", "velocity"),
    ("left_wheel_pos", "left_wheel_pos"), ("right_wheel_pos", "right_wheel_pos"),
    ("mot1_vel", "mot1_vel"), ("mot2_vel", "mot2_vel"),
    ("out_l", "out_l"), ("out_r", "out_r"),
    ("sim_true_pitch", "pitch"), ("sim_base_x", "base_x"), ("sim_base_y", "base_y"),
    ("sim_disturb_fy", "disturb_fy"),
]
STATE_BALANCING, STATE_FALLEN = 3, 4  # RobotState_t values


class CsvLogger:
    def __init__(self, path: str):
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        self._file = open(path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow([c for c, _ in _COLUMNS])

    def write(self, state: dict) -> None:
        row = [round(state["time"] * 1000), STATE_FALLEN if state["fallen"] else STATE_BALANCING]
        row += [f"{state[k]:.6g}" for c, k in _COLUMNS[2:]]
        self._writer.writerow(row)

    def close(self) -> None:
        self._file.close()
