import numpy as np

class Grid:

    def __init__(self, x, y, cell_size):
        self.size_x = x
        self.size_y = y
        self.cell_size = cell_size
        self.grid_map = self.create_empty_grid()
    
    def create_empty_grid(self):
        """
        For accesing a cell in grid_map is [y][x]
        """
        return np.zeros((self.size_y, self.size_x))

    def add_obstacles(self, obst_list):
        """
        obst_list: List[tuple(x, y)]
        """
        for x, y in obst_list:
            self.grid_map[y][x] = 1



