import numpy as np

class Route:

    def __init__(self, route_points):
        self.points = route_points
        self.index = 0
        self.finished = len(self.points) == self.index
    
    def next_objective(self):

        return np.array(self.points[self.index]) if not self.finished else None

    def objective_reached(self):
        if not self.finished:
            self.index += 1
            self.finished = self.index >= len(self.points)

