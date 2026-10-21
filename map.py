import math
import matplotlib.pyplot as plt

class Map:

    def __init__(self, drone_radius, safety_margin, merge_distance = 0.1, show_map=False) -> None:
        self.drone_position = None
        self.goal_position = None
        self.obstacle_points = []
        self.drone_path = []
        
        self.safety_distance = drone_radius + safety_margin
        self.merge_distance = merge_distance

    def update_drone_position(self, new_pos):
        self.drone_position = tuple(new_pos)
        self.drone_path.append(self.drone_position)
        
    def update_goal_position(self, new_pos):
        self.goal_position = tuple(new_pos)
    
    def add_obstacle(self, point):
        point = tuple(round(float(value), 3) for value in point)
        
        for known in self.obstacle_points:
            if math.dist(point, known) < self.merge_distance:
                return False

        self.obstacle_points.append(point)
        return True

    def is_position_safe(self, pos):
        for obstacle in self.obstacle_points:
            if math.dist(pos, obstacle) <= self.safety_distance:
                return False

        return True

    def plot(self):
        figure, axis = plt.subplots()

        if self.drone_path:
            path_x = [position[0] for position in self.drone_path]
            path_y = [position[1] for position in self.drone_path]

            axis.plot(path_x, path_y, color="blue", label="Drone path")

            axis.scatter(path_x[0], path_y[0], color="blue", marker="o", label="start")

            axis.scatter(path_x[-1], path_y[-1], color="cyan", marker="o", label="end")

        if self.obstacle_points:
            obstacle_x = [point[0] for point in self.obstacle_points]
            obstacle_y = [point[1] for point in self.obstacle_points]

            axis.scatter(obstacle_x, obstacle_y, color="red", marker="x", label="Detected obstacle")

        if self.goal_position is not None:
            axis.scatter(self.goal_position[0], self.goal_position[1], color="green", marker="*", label="goal", s=150)
            

        axis.set_title("Drone local map")
        axis.set_xlabel("X (m)")
        axis.set_ylabel("Y (m)")
        axis.set_aspect("equal")
        axis.grid()
        axis.legend()

        plt.show()

    def plot_3d(self):
        figure = plt.figure()
        axis = figure.add_subplot(projection="3d")

        if self.drone_path:
            path_x = [position[0] for position in self.drone_path]
            path_y = [position[1] for position in self.drone_path]
            path_z = [position[2] for position in self.drone_path]

            axis.plot(path_x, path_y, path_z, color="blue", label="Drone path")

            axis.scatter(path_x[0], path_y[0], path_z[0], color="blue", marker="o", label="start")

            axis.scatter(path_x[-1], path_y[-1], path_z[-1], color="cyan", marker="o", label="end")

        if self.obstacle_points:
            obstacle_x = [point[0] for point in self.obstacle_points]
            obstacle_y = [point[1] for point in self.obstacle_points]
            obstacle_z = [point[2] for point in self.obstacle_points]
            

            axis.scatter(obstacle_x, obstacle_y, obstacle_z, color="red", marker="x", label="Detected obstacle")

        if self.goal_position is not None:
            axis.scatter(self.goal_position[0], self.goal_position[1], self.goal_position[2], color="green", marker="*", label="goal", s=150)
            

        axis.set_title("Drone local map")
        axis.set_xlabel("X (m)")
        axis.set_ylabel("Y (m)")
        axis.set_aspect("equal")
        axis.grid()
        axis.legend()

        plt.show()