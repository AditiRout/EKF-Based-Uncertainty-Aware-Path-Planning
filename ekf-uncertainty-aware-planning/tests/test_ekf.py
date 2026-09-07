import numpy as np

from src.ekf import EKF
from src.uncertainty import position_trace


def make_filter():
    return EKF(
        initial_state=np.zeros(4),
        initial_covariance=np.diag([1.0, 1.0, 1.0, 1.0]),
        process_noise=np.diag([0.01, 0.01, 0.02, 0.02]),
        measurement_noise=np.diag([0.25, 0.25]),
    )


def test_prediction_changes_state():
    ekf = make_filter()

    ekf.predict(np.array([1.0, 0.0]), dt=1.0)

    assert ekf.x[0] > 0.0
    assert ekf.x[2] > 0.0


def test_update_reduces_position_uncertainty():
    ekf = make_filter()

    ekf.predict(np.zeros(2), dt=1.0)
    before = position_trace(ekf.P)

    ekf.update(np.array([0.0, 0.0]))
    after = position_trace(ekf.P)

    assert after < before


def test_covariance_is_symmetric():
    ekf = make_filter()
    ekf.step(np.array([0.5, -0.2]), np.array([0.2, -0.1]))

    assert np.allclose(ekf.P, ekf.P.T)
