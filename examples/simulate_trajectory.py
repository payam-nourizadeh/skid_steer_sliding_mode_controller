"""
Simulate the sliding-mode controller tracking a trajectory.

This uses a simple noise-free unicycle model to integrate the robot's
motion -- it is meant to demonstrate the *software interface* (how to call
the controller each control step), not to reproduce the paper's outdoor
grass-terrain experiments (those relied on real slip/skid and are only
meaningful on real hardware or a proper simulator).

Usage:
    python examples/simulate_trajectory.py --trajectory circular
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from smc_controller import SlidingModeController, wrap_to_pi, compute_tracking_error


TRAJECTORIES = {
    "straight": dict(vx_d=lambda t: 0.3, omega_d=lambda t: 0.0,
                     vx_dot_d=lambda t: 0.0, omega_dot_d=lambda t: 0.0, duration=20.0),
    "circular": dict(vx_d=lambda t: 0.2, omega_d=lambda t: 0.2,
                      vx_dot_d=lambda t: 0.0, omega_dot_d=lambda t: 0.0, duration=40.0),
    "bow": dict(vx_d=lambda t: 0.2 * np.sin(0.1 * t), omega_d=lambda t: 0.2 * np.cos(0.1 * t),
                vx_dot_d=lambda t: 0.02 * np.cos(0.1 * t), omega_dot_d=lambda t: -0.02 * np.sin(0.1 * t),
                duration=120.0),
}


def simulate(trajectory_name: str, config_path: str, dt: float = 0.05):
    traj = TRAJECTORIES[trajectory_name]
    controller = SlidingModeController.from_yaml(config_path)

    # Robot state (global frame): starts with the same initial tracking
    # error used in the paper's experiments (0.3 m, 0.1 m, 0 rad).
    x, y, theta = -0.3, -0.1, 0.0
    vx, omega = 0.0, 0.0

    # Desired trajectory state, integrated forward from the origin.
    x_d, y_d, th_d = 0.0, 0.0, 0.0

    steps = int(traj["duration"] / dt)
    log = {"t": [], "x": [], "y": [], "th": [], "x_d": [], "y_d": [], "dist_err": []}

    for i in range(steps):
        t = i * dt
        vx_d, omega_d = traj["vx_d"](t), traj["omega_d"](t)
        vx_dot_d, omega_dot_d = traj["vx_dot_d"](t), traj["omega_dot_d"](t)

        # Integrate the desired (reference) trajectory forward.
        x_d += vx_d * np.cos(th_d) * dt
        y_d += vx_d * np.sin(th_d) * dt
        th_d = wrap_to_pi(th_d + omega_d * dt)

        error = compute_tracking_error(x, y, theta, x_d, y_d, th_d)
        vx_r, omega_r = controller.compute(error, vx, omega, vx_d, omega_d, vx_dot_d, omega_dot_d)

        # Simple actuator model: robot instantaneously achieves the
        # commanded velocity (replace with your own drivetrain model or
        # real hardware feedback for a more realistic simulation).
        vx, omega = vx_r, omega_r
        x += vx * np.cos(theta) * dt
        y += vx * np.sin(theta) * dt
        theta = wrap_to_pi(theta + omega * dt)

        log["t"].append(t)
        log["x"].append(x)
        log["y"].append(y)
        log["th"].append(theta)
        log["x_d"].append(x_d)
        log["y_d"].append(y_d)
        log["dist_err"].append(np.hypot(x_d - x, y_d - y))

    return log


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--trajectory", choices=TRAJECTORIES.keys(), default="circular")
    parser.add_argument("--config", default=os.path.join(os.path.dirname(__file__), "..", "config", "default.yaml"))
    parser.add_argument("--no-plot", action="store_true", help="Skip matplotlib plotting")
    args = parser.parse_args()

    log = simulate(args.trajectory, args.config)
    print(f"Final distance error: {log['dist_err'][-1]:.4f} m")
    print(f"Mean distance error:  {np.mean(log['dist_err']):.4f} m")

    if args.no_plot:
        return

    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 1, figsize=(7, 8))
    axes[0].plot(log["x_d"], log["y_d"], "b-", label="desired")
    axes[0].plot(log["x"], log["y"], "g-", label="robot")
    axes[0].set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    axes[0].set_title(f"{args.trajectory} trajectory")
    axes[0].legend()
    axes[0].axis("equal")

    axes[1].plot(log["t"], log["dist_err"])
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("distance error [m]")

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
