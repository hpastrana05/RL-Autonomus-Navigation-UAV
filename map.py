import math

class Map:

    def __init__(self, drone_radius, safety_margin, merge_distance = 0.1) -> None:
        self.drone_position = None
        self.goal_position = None
        self.obstacle_points = []
        
        self.safety_distance = drone_radius + safety_margin
        self.merge_distance = merge_distance

    def add_obstacle(self, point):
        for known in self.obstacle_points:
            if math.dist(point, known) < self.merge_distance:
                return

        self.obstacle_points.append(point)

    def update_drone_position(self, new_pos):
        self.drone_position = new_pos

    def is_position_dafe(self, pos):
        for obstacle in self.obstacle_points:
            if math.dist(pos, obstacle) <= self.safety_distance:
                return False

        return True
        