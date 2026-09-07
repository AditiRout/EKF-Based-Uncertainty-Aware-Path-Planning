"""Run the A* baseline experiment."""

from src.environment import GridWorld
from src.planner import astar
from src.visualization import plot_world


def main():
    world = GridWorld().make_demo_world()
    start = (1, 1)
    goal = (18, 13)

    path = astar(world, start, goal)
    if path is None:
        raise RuntimeError("No path found.")

    print(f"A* path length: {len(path) - 1}")

    plot_world(
        world,
        path=path,
        start=start,
        goal=goal,
        title="A* Shortest-Path Baseline",
        save_path="results/figures/shortest_path.png",
    )


if __name__ == "__main__":
    main()
