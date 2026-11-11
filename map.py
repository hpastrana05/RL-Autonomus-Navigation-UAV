import math
import matplotlib.pyplot as plt
import numpy as np

class Map:

    def __init__(self, 
        drone_radius, 
        preferred_margin, 
        merge_distance = 0.1,
        hard_margin=0.02,
    ):
        self.drone_position = None
        self.goal_position = None
        self.obstacle_cells = {}
        self.drone_path = []
        
        self.hard_clearance = drone_radius + hard_margin
        self.preferred_clearance = drone_radius + preferred_margin
        self.merge_distance = merge_distance

    @property
    def obstacle_points(self):
        return self.obstacle_cells.values()
    
    def update_drone_position(self, new_pos):
        self.drone_position = tuple(new_pos)
        self.drone_path.append(self.drone_position)
        
    def update_goal_position(self, new_pos):
        self.goal_position = tuple(new_pos)
    
    def add_obstacle(self, point):    
        if point is None:
            return False
            
        cell = self._get_cell(point)
        
        if cell in self.obstacle_cells:
            return False

        self.obstacle_cells[cell] = self._get_cell_center(cell)
        return True

    def is_position_safe(self, pos):
        pos = np.asarray(pos, dtype=float)

        warning = False
        
        for obstacle in self._nearby_obstacles(pos, self.preferred_clearance):
            distance = np.linalg.norm(pos - np.asarray(obstacle))

            if distance <= self.hard_clearance:
                return 0
            elif not warning and distance <= self.preferred_clearance:
                warning = True

        return 1 if warning else 2

    def is_path_safe(self, start_position, end_position):
        start = np.asarray(start_position, dtype=float)
        end = np.asarray(end_position, dtype=float)

        segment = end - start
        segment_length_squared = np.dot(segment, segment) # Longitud del segmento al cuadrado

        warning = False
        
        for obstacle in self._nearby_path_obstacles(start, end, self.preferred_clearance):
            obstacle = np.asarray(obstacle, dtype=float)

            if segment_length_squared == 0:
                nearest_point = start
            else:
                progress = np.dot(obstacle-start, segment) / segment_length_squared
                progress = np.clip(progress, 0.0, 1.0)

                nearest_point = start + progress * segment
            distance = np.linalg.norm(obstacle-nearest_point)

            if distance <= self.hard_clearance:
                return 0
            elif not warning and distance <= self.preferred_clearance:
                warning = True 
        return 1 if warning else 2

    def closest_obstacle_distance(self, position):
        position = np.asarray(position, dtype=float)
        closest_distance = np.inf

        for obstacle in self._nearby_obstacles(position, self.preferred_clearance):
            distance = np.linalg.norm(position - np.asarray(obstacle))

            if distance < closest_distance:
                closest_distance = distance

        return closest_distance

    def _get_cell(self, point):
        point = np.asarray(point, dtype=float)

        return tuple(
            np.floor(point/self.merge_distance).astype(int)
        )

    def _get_cell_center(self, cell):
        return tuple(
            (np.asarray(cell, dtype=float) + 0.5) * self.merge_distance
        )

    def _nearby_obstacles(self, pos, radius):
        center_cell = self._get_cell(pos)
        cell_radius = math.ceil(radius/self.merge_distance)

        for dx in range (-cell_radius, cell_radius + 1):
            for dy in range (-cell_radius, cell_radius + 1):
                for dz in range (-cell_radius, cell_radius + 1):
                    cell = (
                        center_cell[0] + dx,
                        center_cell[1] + dy,
                        center_cell[2] + dz
                    )
                    obstacle = self.obstacle_cells.get(cell)

                    if obstacle is not None:
                        yield obstacle

    def _nearby_path_obstacles(self, start, end, radius):
        start = np.asarray(start, dtype=float)
        end = np.asarray(end, dtype=float)

        lower_bound = np.minimum(start, end) - radius
        upper_bound = np.maximum(start, end) + radius

        min_cell = self._get_cell(lower_bound)
        max_cell = self._get_cell(upper_bound)

        for x in range(min_cell[0], max_cell[0] + 1):
            for y in range(min_cell[1], max_cell[1] + 1):
                for z in range(min_cell[2], max_cell[2] + 1):
                    obstacle = self.obstacle_cells.get((x,y,z))

                    if obstacle is not None:
                        yield obstacle
                    
    def plot(self, modes:list[str]=["2d"]):
        for mode in modes:
            if mode=="2d":
                self.plot_2d()
            elif mode=="3d":
                self.plot_3d()
    
    def plot_2d(self):
        figure, axis = plt.subplots()

        if self.drone_path:
            print(self.drone_path[0])
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
