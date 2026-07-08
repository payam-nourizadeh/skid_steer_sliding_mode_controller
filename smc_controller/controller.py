"""
Sliding-mode trajectory-tracking controller for skid-steering mobile robots (SSMR).

This is a framework-agnostic (pure Python, no ROS) implementation of the
sliding-mode control law described in:

    P. Nourizadeh, F. J. Stevens-McFadden, W. N. Browne,
    "Trajectory Tracking Control for Skid-Steering Mobile Robots with
    Slip and Skid Compensation."

It reproduces the baseline sliding-mode controller (SMC) from Section IV of
the paper (Eqs. 35-44). The slip/skid compensation (SMC-SS, Eqs. 53-54)
requires the two deep-learning slip/skid estimators described in the paper
and is not included here -- see the README for how to plug estimator
outputs into this controller.

The controller takes the tracking error expressed in the robot's local
frame plus the desired trajectory (and its first derivative) and returns
the commanded linear and angular velocity. It has no dependency on ROS,
rospy, or any particular robot driver -- wire it up to whatever
localization/odometry and motor-command interface your platform uses.
"""

from dataclasses import dataclass
from typing import Tuple

import numpy as np

from .utils import load_config


def _sat(value: float) -> float:
    """Saturation function, used in place of sign() to reduce chattering."""
    if abs(value) > 1.0:
        return float(np.sign(value))
    return float(value)


@dataclass
class RobotParams:
    """Physical dimensions of the robot."""

    r: float = 0.09
    """Wheel effective radius [m]."""

    robot_width: float = 0.43
    """Distance between the left and right wheel contact lines, i.e. 2c [m]."""

    x0: float = 0.05
    """Nominal x-position of the instantaneous centre of rotation (ICR) in the
    robot's local frame [m]. This is the uncertain parameter that captures
    the robot's skid-steering behaviour; it is used directly in the
    equivalent control term."""

    x0_min: float = -0.12
    """Lower bound of the ICR uncertainty interval [m] (e.g. how far the ICR
    can shift toward the rear of the robot)."""

    x0_max: float = 0.15
    """Upper bound of the ICR uncertainty interval [m] (e.g. how far the ICR
    can shift toward the front of the robot)."""


@dataclass
class DynamicsParams:
    """Identified low-level dynamics parameters (paper Eq. 30).

    These characterise how the robot's low-level (motor/PID) controller
    responds to commanded linear/angular velocity. They are robot-specific
    and should be obtained via system identification on your platform --
    the defaults below were identified for the robots used to develop this
    controller and are very unlikely to be correct for a different robot.
    """

    c1: float = 0.26038
    c2: float = 0.25095
    c3: float = -0.00049969
    c4: float = 0.99646
    c5: float = 0.002629
    c6: float = 1.0768

    uncertainty_ratio: float = 0.0
    """Fractional uncertainty bound applied symmetrically around c1..c6,
    e.g. 0.25 for +/-25%% (as suggested in the paper). 0.0 reproduces the
    behaviour of the original ROS node, which did not use a margin."""


@dataclass
class ControllerGains:
    """Sliding-mode controller tuning constants (paper Eq. 27, 40-44)."""

    landa1: float = 1.2
    """Sliding-surface constant for the longitudinal error channel (s1).
    Must be > 0."""

    landa2: float = 2.6
    """Sliding-surface constant for the lateral/heading error channel (s2).
    Must be > 0."""

    k1: float = 6.5
    """Robustness gain for the linear-velocity channel. Must dominate the
    estimated uncertainty bound on that channel to guarantee convergence."""

    k2: float = 9.5
    """Robustness gain for the angular-velocity channel. Must dominate the
    estimated uncertainty bound on that channel to guarantee convergence."""

    phi1: float = 3.5
    """Boundary-layer thickness for the saturation function on s1. Larger
    values trade tracking accuracy for less chattering."""

    phi2: float = 2.5
    """Boundary-layer thickness for the saturation function on s2. Larger
    values trade tracking accuracy for less chattering."""


@dataclass
class VelocityLimits:
    """Actuator/safety limits applied to the final control output."""

    threshold_vx: float = 0.4
    """Maximum allowed commanded linear velocity [m/s]."""

    threshold_omega: float = 0.3 * np.pi
    """Maximum allowed commanded angular velocity [rad/s]."""


class SlidingModeController:
    """Sliding-mode trajectory-tracking controller for a skid-steering robot.

    Example
    -------
    >>> controller = SlidingModeController.from_yaml("config/default.yaml")
    >>> vx_cmd, omega_cmd = controller.compute(
    ...     error=(ehat_x, ehat_y, ehat_th),
    ...     vx=current_vx, omega=current_omega,
    ...     vx_d=desired_vx, omega_d=desired_omega,
    ...     vx_dot_d=desired_vx_dot, omega_dot_d=desired_omega_dot,
    ... )
    """

    def __init__(
        self,
        robot: RobotParams = None,
        dynamics: DynamicsParams = None,
        gains: ControllerGains = None,
        limits: VelocityLimits = None,
    ):
        self.robot = robot or RobotParams()
        self.dynamics = dynamics or DynamicsParams()
        self.gains = gains or ControllerGains()
        self.limits = limits or VelocityLimits()

        c = [
            self.dynamics.c1,
            self.dynamics.c2,
            self.dynamics.c3,
            self.dynamics.c4,
            self.dynamics.c5,
            self.dynamics.c6,
        ]
        ratio = self.dynamics.uncertainty_ratio
        self._c = c
        self._c_bounds = [(ci * (1 - ratio), ci * (1 + ratio)) for ci in c]

    # ------------------------------------------------------------------
    # Construction helpers
    # ------------------------------------------------------------------
    @classmethod
    def from_dict(cls, config: dict) -> "SlidingModeController":
        """Build a controller from a nested dict, e.g. loaded from YAML.

        Expected top-level keys: ``robot``, ``dynamics``, ``controller``,
        ``limits`` (all optional -- missing keys fall back to defaults).
        """
        robot = RobotParams(**config.get("robot", {}))
        dynamics = DynamicsParams(**config.get("dynamics", {}))
        gains = ControllerGains(**config.get("controller", {}))
        limits = VelocityLimits(**config.get("limits", {}))
        return cls(robot, dynamics, gains, limits)

    @classmethod
    def from_yaml(cls, path: str) -> "SlidingModeController":
        """Build a controller from a YAML config file (see config/default.yaml)."""
        return cls.from_dict(load_config(path))

    # ------------------------------------------------------------------
    # Control law
    # ------------------------------------------------------------------
    def compute(
        self,
        error: Tuple[float, float, float],
        vx: float,
        omega: float,
        vx_d: float,
        omega_d: float,
        vx_dot_d: float = 0.0,
        omega_dot_d: float = 0.0,
    ) -> Tuple[float, float]:
        """Compute the commanded linear/angular velocity for one control step.

        Parameters
        ----------
        error:
            ``(ehat_x, ehat_y, ehat_th)`` tracking error expressed in the
            robot's local frame. Use ``utils.compute_tracking_error`` to
            build this from global poses.
        vx, omega:
            Current measured linear/angular velocity of the robot (e.g.
            from odometry or wheel encoders).
        vx_d, omega_d:
            Desired linear/angular velocity at this instant.
        vx_dot_d, omega_dot_d:
            Desired linear/angular acceleration at this instant (0 if the
            desired trajectory has constant velocity, e.g. straight-line or
            circular manoeuvres).

        Returns
        -------
        (vx_r, omega_r):
            Commanded linear and angular velocity, already saturated to
            ``limits.threshold_vx`` / ``limits.threshold_omega``.
        """
        e1, e2, e3 = error
        x0, x0_min, x0_max = self.robot.x0, self.robot.x0_min, self.robot.x0_max
        landa1, landa2 = self.gains.landa1, self.gains.landa2
        k1, k2 = self.gains.k1, self.gains.k2
        phi1, phi2 = self.gains.phi1, self.gains.phi2

        c1, c2, c3, c4, c5, c6 = self._c
        (
            (C1_min, C1_max),
            (C2_min, C2_max),
            (C3_min, C3_max),
            (C4_min, C4_max),
            (C5_min, C5_max),
            (C6_min, C6_max),
        ) = self._c_bounds

        sin_e3, cos_e3 = np.sin(e3), np.cos(e3)

        h1 = (
            (-c5 / c2 * vx * omega - c6 / c2 * omega) * e2
            + omega * (-omega * e1 + vx_d * sin_e3 - omega_d * x0 * cos_e3 + omega * x0 + landa1 * e2)
            - vx * landa1
            - (c3 / c1 * omega ** 2 - c4 / c1 * vx)
            + vx_d * (-(omega_d - omega) * sin_e3 + landa1 * cos_e3)
            + vx_dot_d * cos_e3
            + omega_dot_d * x0 * sin_e3
            + omega_d * (x0 * (omega_d - omega) * cos_e3 + landa1 * x0 * sin_e3)
        )

        h1_max = (
            -np.abs((C5_max / C2_min * vx * omega + C6_max / C2_min * omega) * e2)
            + np.abs(omega * (-omega * e1 + vx_d * sin_e3 - omega_d * x0_max * cos_e3 + omega * x0_max + landa1 * e2))
            - np.abs(vx * landa1)
            - np.abs(C3_max / C1_min * omega ** 2 - C4_max / C1_min * vx)
            + np.abs(vx_d * (-(omega_d - omega) * sin_e3 + landa1 * cos_e3))
            + np.abs(vx_dot_d * cos_e3)
            + np.abs(omega_dot_d * x0_max * sin_e3)
            + np.abs(omega_d * (x0_max * (omega_d - omega) * cos_e3 + landa1 * x0_max * sin_e3))
        )

        h2 = (
            -(omega * e2 + vx_d * cos_e3 + omega_d * x0 * sin_e3 - vx) * omega
            + (x0 - e1) * (-c5 / c2 * vx * omega - c6 / c2 * omega)
            + vx_dot_d * sin_e3
            + (omega_d - omega) * vx_d * cos_e3
            - omega_dot_d * x0 * cos_e3
            + (omega_d - omega) * omega_d * x0 * sin_e3
            + landa2 * ((x0 - e1) * omega + vx_d * sin_e3 - omega_d * x0 * cos_e3)
        )

        h2_max2 = (
            (omega * np.abs(e2) + vx_d * cos_e3 + omega_d * x0_max * np.abs(sin_e3) - vx) * np.abs(omega)
            + (x0_max - np.abs(e1)) * np.abs(-C5_max / C2_min * vx * omega - C6_max / C2_min * omega)
            + vx_dot_d * np.abs(sin_e3)
            + np.abs(omega_d - omega) * vx_d * cos_e3
            + np.abs(omega_dot_d) * x0_max * cos_e3
            + np.abs((omega_d - omega) * omega_d * x0_max * np.abs(sin_e3))
            + landa2 * ((x0_max + np.abs(e1)) * omega + vx_d * np.abs(sin_e3) + np.abs(omega_d * x0_max * cos_e3))
        )

        s1 = landa1 * e1 + omega * e2 + vx_d * cos_e3 + omega_d * x0 * sin_e3 - vx
        s2 = landa2 * e2 + omega * (-e1 + x0) + vx_d * sin_e3 - omega_d * x0 * cos_e3

        omega_r_eq = -c2 * h2 / (x0_min - np.abs(e1))
        omega_r_bar = -(C2_max / (x0_min - np.abs(e1)) * (-h2_max2 + k2)) * _sat(s2 / phi2)
        omega_r = omega_r_eq + omega_r_bar

        vx_r_eq = c1 * (h1 + e2 * omega_r_eq / c2)
        vx_r_bar = -(
            C1_max * (np.abs(h1_max) + 1 / C2_min * np.abs(e2) * np.abs(omega_r_bar) - k1)
        ) * _sat(s1 / phi1)
        vx_r = vx_r_eq + vx_r_bar

        threshold_omega = self.limits.threshold_omega
        threshold_vx = self.limits.threshold_vx
        omega_r = threshold_omega * _sat(omega_r / threshold_omega)
        vx_r = threshold_vx * _sat(vx_r / threshold_vx)

        return float(vx_r), float(omega_r)
