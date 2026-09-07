"""Compare two deliberately different candidate routes.

The first route is the A* shortest path.

The second route is a manually specified detour through a region with
a different motion profile. This is a teaching experiment: it shows
how the uncertainty-aware objective can prefer a non-shortest route.
It is not an RRBT implementation.
"""

import matplotlib.pyplot as plt

from src.environment import GridWorld
from src.planner import astar
from src.simulation import simulate_path
from src.uncertainty_planner import (
    score_route,
)
from src.visualization import plot_planner_comparison


def build_detour(start, goal):
    """A simple waypoint route that creates a longer candidate."""
    waypoints = [
        start,
        (3, 1),
        (3, 13),
        (10, 13),
        (16, 13),
        goal,
    ]

    path = []
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        x1, y1 = a
        x2, y2 = b

        if not path:
            path.append(a)

        while (x1, y1) != (x2, y2):
            if x1 != x2:
                x1 += 1 if x2 > x1 else -1
            elif y1 != y2:
                y1 += 1 if y2 > y1 else -1
            path.append((x1, y1))

    return path


def main():
    world = GridWorld().make_demo_world()
    start = (1, 1)
    goal = (18, 13)

    shortest = astar(world, start, goal)
    if shortest is None:
        raise RuntimeError("A* could not find a route.")

    detour = build_detour(start, goal)

    # Keep only collision-free candidate routes.
    candidates = []
    for route in [shortest, detour]:
        if all(world.is_free(cell) for cell in route):
            candidates.append(route)

    if len(candidates) < 2:
        raise RuntimeError(
            "The demo detour intersects an obstacle; adjust the waypoints."
        )

    results = []
    for route in candidates:
        simulation = simulate_path(
            route,
            measurement_noise_std=0.7,
            seed=7,
        )
        results.append(simulation)

    # A moderate value makes the tradeoff visible without claiming
    # that this lambda is universally optimal.
    lambda_uncertainty = 0.08

    scores = [
        score_route(
            route,
            simulation["uncertainties"],
            lambda_uncertainty,
        )
        for route, simulation in zip(candidates, results)
    ]

    best_index = min(
        range(len(scores)),
        key=lambda i: scores[i].objective,
    )

    for i, score in enumerate(scores):
        print(
            f"Route {i + 1}: "
            f"length={score.path_length:.1f}, "
            f"avg_uncertainty={score.average_uncertainty:.3f}, "
            f"total_uncertainty={score.total_uncertainty:.3f}, "
            f"objective={score.objective:.3f}"
        )

    print(f"Selected route: {best_index + 1}")

    plot_planner_comparison(
        candidates,
        scores,
        save_path="results/figures/planner_comparison.png",
    )
    plt.show()


if __name__ == "__main__":
    main()
