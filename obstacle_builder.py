from grid import Grid
import pybullet as p

def create_grid_obstacles(grid: Grid, pybullet_client, obstacle_height, obstacle_color):
    obstacles_ids = []
    
    half_extents = [
        grid.cell_size / 2, 
        grid.cell_size / 2, 
        obstacle_height / 2
    ]

    wall_colission = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=half_extents,
        physicsClientId=pybullet_client,
    )

    wall_visual = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=half_extents,
        rgbaColor=obstacle_color,
        physicsClientId=pybullet_client,
    )

    for y, row in enumerate(grid.grid_map):
        for x, cell in enumerate(row):
            if cell:
                position = [
                    x * grid.cell_size,
                    y * grid.cell_size,
                    obstacle_height/2
                ]

                obs_id = p.createMultiBody(
                    baseMass=0,
                    baseCollisionShapeIndex=wall_colission,
                    baseVisualShapeIndex=wall_visual,
                    basePosition = position,
                    physicsClientId=pybullet_client,
                )

                obstacles_ids.append(obs_id)

    return obstacles_ids

