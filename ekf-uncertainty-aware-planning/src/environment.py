"""2D grid environment and collision checking."""

from dataclasses import dataclass
import numpy as np


@dataclass
class GridWorld:
    width: int = 20
    height: int = 15

    def __post_init__(self):
        self.grid = np.zeros((self.height, self.width), dtype=bool)

    def add_rectangle(self, x_min, y_min, x_max, y_max):
        """Add an inclusive rectangular obstacle."""
        x_min, x_max = sorted((int(x_min), int(x_max)))
        y_min, y_max = sorted((int(y_min), int(y_max)))
        self.grid[y_min:y_max + 1, x_min:x_max + 1] = True

    def is_free(self, cell):
        x, y = cell
        return (
            0 <= x < self.width
            and 0 <= y < self.height
            and not self.grid[y, x]
        )

    def neighbors4(self, cell):
        x, y = cell
        candidates = [
            (x + 1, y), (x - 1, y),
            (x, y + 1), (x, y - 1),
        ]
        return [c for c in candidates if self.is_free(c)]

    def make_demo_world(self):
        """Create obstacles that leave two meaningful routes."""
        self.add_rectangle(6, 2, 8, 11)
        self.add_rectangle(12, 3, 14, 12)
        self.add_rectangle(9, 7, 11, 9)
        return self
