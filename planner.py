
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




def plan_route(grid, start, goal):
    pass
    