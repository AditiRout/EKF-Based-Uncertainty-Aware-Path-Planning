"""Run the EKF state-estimation experiment."""

import matplotlib.pyplot as plt

from src.environment import GridWorld
from src.planner import astar
from src.simulation import simulate_path
from src.visualization import (
    plot_ekf,
    plot_uncertainty,
    plot_uncertainty_ellipse,
)


def main():
    world = GridWorld().make_demo_world()
    path = astar(world, (1, 1), (18, 13))

    result = simulate_path(
        path,
        measurement_noise_std=0.7,
        seed=7,
    )

    position_error = (
        result["true_states"][:, :2]
        - result["estimates"][:, :2]
    )
    rmse = (
        (position_error**2).mean(axis=0).sum()
    ) ** 0.5

    print(f"Position RMSE: {rmse:.3f}")
    print(
        "Initial uncertainty:",
        f"{result['uncertainties'][0]:.3f}",
    )
    print(
        "Final uncertainty:",
        f"{result['uncertainties'][-1]:.3f}",
    )

    plot_ekf(
        result,
        save_path="results/figures/ekf_estimation.png",
    )
    plot_uncertainty(
        result,
        save_path="results/figures/uncertainty.png",
    )
    plot_uncertainty_ellipse(
        result,
        save_path="results/figures/uncertainty_ellipse.png",
    )

    plt.show()


if __name__ == "__main__":
    main()
