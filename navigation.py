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
    tolerance=0.05
):
    current_pos = np.asarray(current_pos, dtype=float)
    goal_pos = np.asarray(goal_pos, dtype=float)

    direction_to_goal = goal_pos - current_pos
    direction_to_goal[2] = 0.0

    distance_to_goal = np.linalg.norm(direction_to_goal)

    if distance_to_goal <= tolerance:
        return np.zeros(4), True

    goal_direction = direction_to_goal / distance_to_goal

    candidate_angles = [
        0.0,
        np.pi/4,
        -np.pi/4,
        np.pi/2,
        -np.pi/2
    ]

    for angle in candidate_angles:
        rotation = np.array([
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle), np.cos(angle)]
        ])

        candidate_xy = rotation @ goal_direction[:2]
        candidate_direction = np.array([
            candidate_xy[0],
            candidate_xy[1],
            0.0
        ])

        candidate_position = current_pos + candidate_direction * lookahead_distance

        if local_map.is_path_safe(current_pos, candidate_position):
            return np.array([
                candidate_direction[0],
                candidate_direction[1],
                candidate_direction[2],
                speed_fraction
            ]), False
    return np.zeros(4), False