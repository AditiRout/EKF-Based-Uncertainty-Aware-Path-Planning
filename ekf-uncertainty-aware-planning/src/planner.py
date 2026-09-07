"""A* shortest-path planner for a 4-connected grid."""

import heapq
import math


def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])
  #manhattan distance

def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def astar(world, start, goal):
    """Return a shortest collision-free grid path."""
    if not world.is_free(start) or not world.is_free(goal):
        raise ValueError("Start and goal must be free cells.")

    open_heap = []
    heapq.heappush(open_heap, (manhattan(start, goal), 0, start))

    came_from = {}
    g_score = {start: 0}

    while open_heap:
        _, current_g, current = heapq.heappop(open_heap)

        # Ignore stale heap entries.
        if current_g != g_score.get(current, math.inf):
            continue

        if current == goal:
            return reconstruct_path(came_from, current)

        for neighbor in world.neighbors4(current):
            tentative_g = current_g + 1

            if tentative_g < g_score.get(neighbor, math.inf):
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + manhattan(neighbor, goal)
                heapq.heappush(
                    open_heap,
                    (f_score, tentative_g, neighbor),
                )

    return None
