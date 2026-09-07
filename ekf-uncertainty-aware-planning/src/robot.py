"""Simple 2D robot motion model."""

from dataclasses import dataclass
import numpy as np


@dataclass
class Robot:
    state: np.ndarray
    acceleration_noise_std: float = 0.08
    measurement_noise_std: float = 0.5

    def __post_init__(self):
        self.state = np.asarray(self.state, dtype=float).reshape(4)

    def step(self, acceleration, dt=1.0, rng=None):
        """Advance the true robot state using noisy acceleration."""
        if rng is None:
            rng = np.random.default_rng()

        acceleration = np.asarray(acceleration, dtype=float).reshape(2)
        noisy_accel = acceleration + rng.normal(
            0.0, self.acceleration_noise_std, size=2
        )

        px, py, vx, vy = self.state
        ax, ay = noisy_accel

        self.state = np.array([
            px + vx * dt + 0.5 * ax * dt**2,
            py + vy * dt + 0.5 * ay * dt**2,
            vx + ax * dt,
            vy + ay * dt,
        ])

        return self.state.copy()

    def measure_position(self, rng=None):
        """Return a noisy position measurement."""
        if rng is None:
            rng = np.random.default_rng()

        noise = rng.normal(
            0.0, self.measurement_noise_std, size=2
        )
        return self.state[:2] + noise
