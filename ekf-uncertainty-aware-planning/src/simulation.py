"""Simulation helpers for EKF experiments."""

import numpy as np

from .ekf import EKF
from .robot import Robot
from .uncertainty import position_trace


def direction_acceleration(current, target, gain=1.0):
    """Create a simple acceleration command toward the next grid cell."""
    current = np.asarray(current, dtype=float)
    target = np.asarray(target, dtype=float)
    direction = target - current

    norm = np.linalg.norm(direction)
    if norm < 1e-12:
        return np.zeros(2)

    return gain * direction / norm


def simulate_path(
    path,
    dt=1.0,
    acceleration_noise_std=0.08,
    measurement_noise_std=0.5,
    seed=7,
):
    """Simulate a robot and EKF along a grid path.

    Each path segment is one time step. The robot is encouraged toward
    the next waypoint, while the EKF receives the same nominal control
    plus a noisy position measurement.
    """
    rng = np.random.default_rng(seed)

    start = np.array([
        float(path[0][0]),
        float(path[0][1]),
        0.0,
        0.0,
    ])

    robot = Robot(
        state=start,
        acceleration_noise_std=acceleration_noise_std,
        measurement_noise_std=measurement_noise_std,
    )

    initial_P = np.diag([1.5, 1.5, 1.0, 1.0])
    Q = np.diag([0.02, 0.02, 0.08, 0.08])
    R = np.diag([
        measurement_noise_std**2,
        measurement_noise_std**2,
    ])

    ekf = EKF(
        initial_state=start,
        initial_covariance=initial_P,
        process_noise=Q,
        measurement_noise=R,
    )

    true_states = [robot.state.copy()]
    measurements = [robot.measure_position(rng)]
    estimates = [ekf.x.copy()]
    covariances = [ekf.P.copy()]
    uncertainties = [position_trace(ekf.P)]

    for waypoint in path[1:]:
        nominal_accel = direction_acceleration(
            robot.state[:2],
            waypoint,
            gain=2.0,
        )

        true_state = robot.step(nominal_accel, dt, rng)
        measurement = robot.measure_position(rng)

        estimate, covariance = ekf.step(
            nominal_accel,
            measurement,
            dt,
        )

        true_states.append(true_state)
        measurements.append(measurement)
        estimates.append(estimate)
        covariances.append(covariance)
        uncertainties.append(position_trace(covariance))

    return {
        "true_states": np.asarray(true_states),
        "measurements": np.asarray(measurements),
        "estimates": np.asarray(estimates),
        "covariances": np.asarray(covariances),
        "uncertainties": np.asarray(uncertainties),
    }


def route_candidates(world, start, goal):
    """Return a few manually structured routes for the demo world.

    Routes are generated from A* on modified obstacle sets in the
    experiment script. This helper is kept minimal for readability.
    """
    raise NotImplementedError(
        "Use the candidate-route construction in compare_planners.py."
    )
