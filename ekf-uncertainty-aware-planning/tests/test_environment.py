from src.environment import GridWorld


def test_obstacle_and_free_cell():
    world = GridWorld(width=10, height=10)
    world.add_rectangle(2, 2, 4, 4)

    assert not world.is_free((3, 3))
    assert world.is_free((0, 0))
