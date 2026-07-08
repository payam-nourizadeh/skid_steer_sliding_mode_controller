import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from smc_controller import SlidingModeController, compute_tracking_error


CONFIG_PATH = os.path.join(os.path.dirname(__file__), "..", "config", "default.yaml")


def test_loads_default_config():
    controller = SlidingModeController.from_yaml(CONFIG_PATH)
    assert controller.robot.r == 0.09
    assert controller.limits.threshold_vx == 0.4


def test_zero_error_zero_velocity_gives_small_command():
    controller = SlidingModeController.from_yaml(CONFIG_PATH)
    vx_r, omega_r = controller.compute(
        error=(0.0, 0.0, 0.0), vx=0.0, omega=0.0, vx_d=0.0, omega_d=0.0
    )
    assert abs(vx_r) < 1e-6
    assert abs(omega_r) < 1e-6


def test_output_respects_saturation_limits():
    controller = SlidingModeController.from_yaml(CONFIG_PATH)
    vx_r, omega_r = controller.compute(
        error=(5.0, 5.0, 3.0), vx=0.0, omega=0.0, vx_d=2.0, omega_d=2.0
    )
    assert abs(vx_r) <= controller.limits.threshold_vx + 1e-9
    assert abs(omega_r) <= controller.limits.threshold_omega + 1e-9


def test_compute_tracking_error_matches_expected_frame_rotation():
    # Robot at origin facing +x, goal directly ahead: error should be
    # purely in the robot's local x-axis.
    ehat_x, ehat_y, ehat_th = compute_tracking_error(0, 0, 0, 1.0, 0.0, 0.0)
    assert abs(ehat_x - 1.0) < 1e-9
    assert abs(ehat_y) < 1e-9
    assert abs(ehat_th) < 1e-9


if __name__ == "__main__":
    test_loads_default_config()
    test_zero_error_zero_velocity_gives_small_command()
    test_output_respects_saturation_limits()
    test_compute_tracking_error_matches_expected_frame_rotation()
    print("All tests passed.")
