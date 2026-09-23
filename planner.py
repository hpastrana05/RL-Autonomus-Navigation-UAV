import numpy as np

def get_neighbours(grid, pos_x, pos_y):
    neighbours = []

    up =    grid[pos_y - 1][pos_x] if pos_y > 0 else 1
    down =  grid[pos_y + 1][pos_x] if pos_y < len(grid) -1 else 1
    left =  grid[pos_y][pos_x - 1] if pos_x > 0 else 1
    right = grid[pos_y][pos_x + 1] if pos_x < len(grid[pos_y]) - 1 else 1

    if up == 0:
        neighbours.append((pos_x, pos_y - 1))

    if down == 0:
        neighbours.append((pos_x, pos_y + 1))

    if left == 0:
        neighbours.append((pos_x - 1, pos_y))
    
    if right == 0:
        neighbours.append((pos_x + 1, pos_y))
    
    return neighbours


def heuristic(cell, goal):
    """
    Right now using manhattan distance
    """
    return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])

def _check_if_inside(grid, point):
    if 0 <= point[0] < len(grid[0]) and 0 <= point[1] < len(grid):
        return True
    return False

def plan_route(grid, start, goal):
    """
    Start and goal are in (x, y)
    A* algorithm
    """
    if not _check_if_inside(grid, start) or not _check_if_inside(grid, goal):
        raise ValueError("Start/Goal are not inside the grid")

    pending = [start]
    cost_so_far = {start: 0}
    came_from = {start: None}

    while pending:
        current = min(pending, key=lambda cell: cost_so_far[cell] + heuristic(cell, goal))
        pending.remove(current)

        if current == goal:
            route = []
            while current is not None:
                route.append(current)
                current = came_from[current]
            return list(reversed(route))

        for neighbour in get_neighbours(grid, current[0], current[1]):
            new_cost = cost_so_far[current] + 1

            if neighbour not in cost_so_far or new_cost < cost_so_far[neighbour]:
                cost_so_far[neighbour] = new_cost
                came_from[neighbour] = current

                if neighbour not in pending:
                    pending.append(neighbour)

    return []


def route_to_meters(route, cell_size):
    new_route = []
    for point in route:
        x = point[0] * cell_size
        y = point[1] * cell_size
        z = 0.1
        new_route.append((x,y,z))
    return new_route