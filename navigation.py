import numpy as np

def action_towards_goal(current_pos, goal_pos, speed_fraction, tolerance=0.05):
    direction = np.asarray(goal_pos) - np.asarray(current_pos)

    distance = np.linalg.norm(direction)

    if distance <= tolerance:
        return np.zeros(4), True

    unit_direction = direction/distance

    return np.array([
        unit_direction[0],
        unit_direction[1],
        unit_direction[2],
        speed_fraction
    ]), False

def action_with_local_avoidance(
    current_pos,
    goal_pos,
    local_map,
    speed_fraction,
    lookahead_distance = 0.25,
    tolerance=0.05,
    min_altitude=0.1
):
    current_pos = np.asarray(current_pos, dtype=float)
    goal_pos = np.asarray(goal_pos, dtype=float)

    direction_to_goal = goal_pos - current_pos

    distance_to_goal = np.linalg.norm(direction_to_goal)

    if distance_to_goal <= tolerance:
        return np.zeros(4), True
        
    
    horizontal_direction = np.array([
        direction_to_goal[0],
        direction_to_goal[1],
        0.0
    ])

    horizontal_distance = np.linalg.norm(horizontal_direction)

    if horizontal_distance == 0:
        horizontal_unit = np.array([1.0, 0.0])
    else:
        horizontal_unit = horizontal_direction[:2] / horizontal_distance

    base_elevation = np.arctan2(
        direction_to_goal[2],
        horizontal_distance
    )
    
    
    candidate_angles = [
        (0.0, 0.0),              # directo 
        (np.pi / 4, 0.0),        # izquierda
        (-np.pi / 4, 0.0),       # derecha
        (np.pi / 2, 0.0),        # izquierda fuerte
        (-np.pi / 2, 0.0),       # derecha fuerte

        (0.0, np.pi/2),          # subir vertical
        
        (0.0, np.pi / 6),        # hacia objetivo y arriba
        (np.pi / 4, np.pi / 6),  # izquierda y arriba
        (-np.pi / 4, np.pi / 6), # derecha y arriba
    
        (0.0, -np.pi / 6),       # hacia objetivo y abajo

        (np.pi, 0.0),
        (np.pi, np.pi / 6),
        (np.pi, np.pi / 3),
    ]

    current_clearance = local_map.closest_obstacle_distance(current_pos)

    best_escape_dir = None
    best_escape_clearance = current_clearance

    for yaw, elevation_offset in candidate_angles:
        rotation = np.array([
            [np.cos(yaw), -np.sin(yaw)],
            [np.sin(yaw), np.cos(yaw)]
        ])

        candidate_xy = rotation @ horizontal_unit
        elevation = base_elevation + elevation_offset
        
        candidate_direction = np.array([
            np.cos(elevation) * candidate_xy[0],
            np.cos(elevation) * candidate_xy[1],
            np.sin(elevation),
        ])

        candidate_position = current_pos + candidate_direction * lookahead_distance
        
        if candidate_position[2] < min_altitude:
            continue
        
        path_status = local_map.is_path_safe(current_pos, candidate_position)

        if path_status == 0:
            continue

        if path_status == 2:
            return np.array([
                candidate_direction[0],
                candidate_direction[1],
                candidate_direction[2],
                speed_fraction
            ]), False

        candidate_clearance = local_map.closest_obstacle_distance(candidate_position)

        if candidate_clearance > best_escape_clearance:
            best_escape_dir = candidate_direction
            best_escape_clearance = candidate_clearance

    if best_escape_dir is not None:
        return np.array([
            best_escape_dir[0],
            best_escape_dir[1],
            best_escape_dir[2],
            speed_fraction * 0.5,
        ]), False
    
    return np.zeros(4), False