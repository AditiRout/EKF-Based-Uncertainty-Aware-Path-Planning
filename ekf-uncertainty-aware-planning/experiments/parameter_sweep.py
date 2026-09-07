"""Sweep lambda to study the distance/uncertainty tradeoff."""

from pathlib import Path
import pandas as pd

from src.environment import GridWorld
from src.planner import astar
from src.simulation import simulate_path
from src.uncertainty_planner import score_route
from .compare_planners import build_detour


def main():
    world = GridWorld().make_demo_world()
    start = (1, 1)
    goal = (18, 13)

    shortest = astar(world, start, goal)
    detour = build_detour(start, goal)

    routes = [
        shortest,
        detour,
    ]

    valid_routes = [
        route for route in routes
        if all(world.is_free(cell) for cell in route)
    ]

    simulations = [
        simulate_path(route, measurement_noise_std=0.7, seed=7)
        for route in valid_routes
    ]

    lambda_values = [0.0, 0.02, 0.05, 0.08, 0.12, 0.2, 0.5]

    rows = []

    for lam in lambda_values:
        scores = [
            score_route(
                route,
                simulation["uncertainties"],
                lam,
            )
            for route, simulation in zip(
                valid_routes,
                simulations,
            )
        ]

        best = min(
            range(len(scores)),
            key=lambda i: scores[i].objective,
        )

        rows.append({
            "lambda": lam,
            "selected_route": best + 1,
            "path_length": scores[best].path_length,
            "average_uncertainty": scores[best].average_uncertainty,
            "total_uncertainty": scores[best].total_uncertainty,
            "objective": scores[best].objective,
        })

    output = Path("results/data/parameter_sweep.csv")
    output.parent.mkdir(parents=True, exist_ok=True)
    pd.DataFrame(rows).to_csv(output, index=False)

    print(pd.DataFrame(rows).to_string(index=False))
    print(f"\nSaved: {output}")


if __name__ == "__main__":
    main()
