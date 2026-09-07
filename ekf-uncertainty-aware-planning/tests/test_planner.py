from src.environment import GridWorld
from src.planner import astar


def test_astar_reaches_goal():
    world = GridWorld(width=10, height=10)
    start = (0, 0)
    goal = (5, 5)

    path = astar(world, start, goal)

    assert path is not None
    assert path[0] == start
    assert path[-1] == goal


def test_astar_avoids_obstacle():
    world = GridWorld(width=10, height=10)
    world.add_rectangle(2, 0, 2, 4)

    path = astar(world, (0, 0), (5, 0))

    assert path is not None
    assert all(world.is_free(cell) for cell in path)
