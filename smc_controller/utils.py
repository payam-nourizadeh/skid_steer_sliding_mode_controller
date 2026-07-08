"""Framework-agnostic helper utilities for the sliding-mode controller."""

import csv
import datetime
import math
import os
from typing import Optional, Tuple

import yaml


def wrap_to_pi(angle: float) -> float:
    """Wrap an angle in radians to the range (-pi, pi]."""
    return ((angle + math.pi) % (2 * math.pi)) - math.pi


def compute_tracking_error(
    x: float,
    y: float,
    theta: float,
    x_d: float,
    y_d: float,
    theta_d: float,
) -> Tuple[float, float, float]:
    """Express the tracking error between the current and desired pose in
    the robot's local frame.

    ``x, y, theta`` is the robot's current pose (e.g. from odometry) and
    ``x_d, y_d, theta_d`` is the desired pose at this instant, both in the
    same global frame. Returns ``(ehat_x, ehat_y, ehat_th)`` for use with
    :meth:`SlidingModeController.compute`.
    """
    e_x = x_d - x
    e_y = y_d - y

    ehat_x = e_x * math.cos(theta) + e_y * math.sin(theta)
    ehat_y = -e_x * math.sin(theta) + e_y * math.cos(theta)
    ehat_th = wrap_to_pi(theta_d - theta)

    return ehat_x, ehat_y, ehat_th


def load_config(path: str) -> dict:
    """Load a YAML controller configuration file into a plain dict."""
    with open(path, "r") as f:
        return yaml.safe_load(f)


class TrajectoryLogger:
    """Minimal CSV logger for tracking data, independent of any middleware.

    Replaces the ROS-specific logging in the original node with a plain
    Python/CSV logger you can use (or ignore) in any application.
    """

    HEADER = [
        "timestamp",
        "x_d", "y_d", "th_d",
        "x", "y", "th",
        "e_x", "e_y", "e_th",
        "vx_r", "omega_r",
        "vx", "omega",
    ]

    def __init__(self, log_dir: str, run_name: Optional[str] = None):
        run_name = run_name or datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        os.makedirs(log_dir, exist_ok=True)
        self.filepath = os.path.join(log_dir, f"trajectory_{run_name}.csv")
        self._file = open(self.filepath, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.HEADER)

    def log(self, x_d, y_d, th_d, x, y, th, e_x, e_y, e_th, vx_r, omega_r, vx, omega):
        self._writer.writerow(
            [
                datetime.datetime.now().isoformat(),
                x_d, y_d, th_d,
                x, y, th,
                e_x, e_y, e_th,
                vx_r, omega_r,
                vx, omega,
            ]
        )
        self._file.flush()

    def close(self):
        self._file.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
