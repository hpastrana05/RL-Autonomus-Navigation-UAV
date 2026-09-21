import math
import numpy as np
from route import Route


def next_action(obs, route: Route, velocity, tolerance):

    if route.finished:
        return [0, 0, 0, 0]
    
    vel = velocity
    pos = obs[0, 0:3]
    next_pos = route.next_objective()
    
    distance = math.dist(pos, next_pos)
    direction = next_pos - pos

    if distance < tolerance:
        print(f"Point: {next_pos} reached")
        route.objective_reached()

    action = [direction[0], direction[1], direction[2], vel]

    
    return action

