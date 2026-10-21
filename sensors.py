from numpy.ma.core import minimum
import pybullet as p

class ProximitySensor:
    def __init__(self, pybullet_client, drone_id, radius, show_debug = False):
        self.pybullet_client = pybullet_client
        self.drone_id = drone_id
        self.radius = radius

        self.debug_sphere_id = None
        if show_debug:
            self._create_debug_sphere()
            
    def detect(self, obstacle_ids):
        self._update_debug_sphere()
        detections = []
        
        for obstacle_id in obstacle_ids:
            closest_points = p.getClosestPoints(
                bodyA=self.drone_id,
                bodyB=obstacle_id,
                distance=self.radius,
                physicsClientId=self.pybullet_client
            )

            if closest_points:
                closest_point = min(closest_points, key=lambda point:point[8])

                detections.append({
                    "obstacle_id": obstacle_id,
                    "distance": closest_point[8],
                    "position": tuple(closest_point[6])
                })
    
        return detections

    def _get_drone_horizontal_raidus(self):
        minimum, maximum = p.getAABB(self.drone_id, physicsClientId=self.pybullet_client)

        half_x = (maximum[0] - minimum[0]) /2
        half_y = (maximum[1] - minimum[1]) /2

        return max(half_x, half_y)
        
    def _create_debug_sphere(self):
        debug_radius = self.radius + self._get_drone_horizontal_raidus()
        
        visual_shape = p.createVisualShape(
            p.GEOM_SPHERE,
            radius=debug_radius,
            rgbaColor=[0.0, 0.7, 1.0, 0.2],
            physicsClientId=self.pybullet_client
        )

        self.debug_sphere_id = p.createMultiBody(
            baseMass = 0,
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=visual_shape,
            physicsClientId=self.pybullet_client
        )

    def _update_debug_sphere(self):
        if self.debug_sphere_id is None:
            return

        drone_position, drone_orientation = p.getBasePositionAndOrientation(
            self.drone_id,
            physicsClientId=self.pybullet_client
        )

        p.resetBasePositionAndOrientation(
            self.debug_sphere_id,
            drone_position,
            drone_orientation,
            physicsClientId=self.pybullet_client
        )