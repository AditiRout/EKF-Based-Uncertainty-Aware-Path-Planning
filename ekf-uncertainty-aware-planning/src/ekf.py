"""Extended Kalman Filter for a small 2D constant-velocity model.

The dynamics and measurement functions are linear here, so the EKF
mathematics reduces to the ordinary Kalman Filter. We keep the EKF
interface because it makes the later transition to nonlinear robotics
models straightforward.
"""

import numpy as np


class EKF:
    def __init__(
        self,
        initial_state,
        initial_covariance,
        process_noise,
        measurement_noise,
    ):
        self.x = np.asarray(initial_state, dtype=float).reshape(4)
        self.P = np.asarray(initial_covariance, dtype=float).reshape(4, 4)
        self.Q = np.asarray(process_noise, dtype=float).reshape(4, 4)
        self.R = np.asarray(measurement_noise, dtype=float).reshape(2, 2)

        # Position-only observation model z = Hx.
        self.H = np.array([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
        ])

    @staticmethod
    def transition_matrix(dt):
        """Jacobian F for constant-velocity dynamics."""
        return np.array([
            [1.0, 0.0, dt,  0.0],
            [0.0, 1.0, 0.0, dt ],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])

    @staticmethod
    def control_matrix(dt):
        """Matrix G mapping acceleration to state change."""
        return np.array([
            [0.5 * dt**2, 0.0],
            [0.0, 0.5 * dt**2],
            [dt, 0.0],
            [0.0, dt],
        ])

    def predict(self, acceleration, dt=1.0):
        """Predict state and covariance from acceleration input."""
        F = self.transition_matrix(dt)
        G = self.control_matrix(dt)
        acceleration = np.asarray(acceleration, dtype=float).reshape(2)

        self.x = F @ self.x + G @ acceleration
        self.P = F @ self.P @ F.T + self.Q
        self.P = 0.5 * (self.P + self.P.T)

        return self.x.copy(), self.P.copy()

    def update(self, measurement):
        """Correct the predicted state using a position measurement."""
        z = np.asarray(measurement, dtype=float).reshape(2)

        innovation = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R

        # Solve S X = ... rather than explicitly computing inv(S).
        K = np.linalg.solve(S, self.H @ self.P).T

        self.x = self.x + K @ innovation

        # Joseph form is numerically safer and keeps covariance symmetric.
        I = np.eye(4)
        self.P = (
            (I - K @ self.H) @ self.P @ (I - K @ self.H).T
            + K @ self.R @ K.T
        )
        self.P = 0.5 * (self.P + self.P.T)

        return self.x.copy(), self.P.copy()

    def step(self, acceleration, measurement, dt=1.0):
        """Run one complete predict/update cycle."""
        self.predict(acceleration, dt)
        return self.update(measurement)

    def position_covariance(self):
        return self.P[:2, :2].copy()
