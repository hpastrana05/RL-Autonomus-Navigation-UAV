import math
import numpy as np


def next_action(obs, route, next_idx, velocity, tolerance):
    if next_idx < len(route):
        vel = velocity
        pos = obs[0, 0:3]
        next_pos = np.array(route[next_idx])
        
        distance = math.dist(pos, next_pos)
        direction = next_pos - pos

        if distance < tolerance:
            next_idx =+ 1

        action = [direction[0], direction[1], direction[2], vel]
                
    else:
        action = [0, 0, 0, 0]
    
    return action, next_idx

