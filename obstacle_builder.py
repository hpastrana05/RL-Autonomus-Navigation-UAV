import pybullet as p

def create_box_obstacle(pybullet_client, position, half_extents, color=[1.0, 0.0, 0.0, 1.0]):
    collision_shape = p.createCollisionShape(
        p.GEOM_BOX,
        halfExtents=half_extents,
        physicsClientId=pybullet_client
    )

    visual_shape = p.createVisualShape(
        p.GEOM_BOX,
        halfExtents=half_extents,
        rgbaColor=color,
        physicsClientId=pybullet_client
    )

    return p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision_shape,
        baseVisualShapeIndex=visual_shape,
        basePosition=position,
        physicsClientId=pybullet_client
    )
