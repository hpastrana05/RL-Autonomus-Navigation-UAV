import numpy as np
import time
import pybullet as p

from gym_pybullet_drones.utils.utils import sync
from gym_pybullet_drones.envs.VelocityAviary import VelocityAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics

from map import Map
from sensors import ProximitySensor, LidarSensor
from obstacle_builder import create_box_obstacle
from scenario_loader import load_scenario
from navigation import action_towards_goal, action_with_local_avoidance

# ----- Simulation -----
GUI = True
SIMULATION_DURATION_SECONDS = 10
PYB_FREQUENCY_HZ = 240
CONTROL_FREQUENCY_HZ = 120    # Velocidad de actualizacion de sensores base : 48

PLANNER_FREQ_HZ = 20

# ----- Drone -----
DRONE_MODEL = DroneModel.CF2X
INITIAL_POSITION = [0.0, 0.0, 0.1]
INITIAL_ORIENTATION = [0.0, 0.0, 0.0]

# ----- Movement -----
SPEED_FRACTION = 2.0

# ---- Debug Vector ----
SEE_VELOCITY_VECTOR = True
SCALE_VECTOR = 0.2
COLOR_VECTOR = (0.0, 0.0, 1.0)

# ----- Proximity sensor -----
SENSOR_RADIUS = 0.50
SHOW_SENSOR_BUBBLE = True
# STOP_ON_DETECTION = True

# ---- Lidar sensor -----
LIDAR_RANGE = 0.5
LIDAR_RAY_COUNT = 36
LIDAR_MIN_RANGE = 0.1
LIDAR_ELEVATION_ANGLES = (
    -np.pi/6,
    0.0,
    np.pi/6,
)
LIDAR_DEBUG = False

# ----- Local map -----
DRONE_RADIUS = 0.06
PREFERRED_MARGIN = 0.35
HARD_MARGIN = 0.02

MERGE_DISTANCE = 0.05
SHOW_MAP = True
MAP_MODES = ["2d", "3d"]
LOOKAHEAD_DISTANCE = 0.35
MIN_ALTITUDE = 0.1

# ----- Scenario -----
SCENARIO_PATH = "scenarios/empty.json"


def run():
    scenario = load_scenario(SCENARIO_PATH)

    #------ INITIAL VALUES OF DRONE ------
    initial_position = np.array([scenario["start"]["position"]])
    initial_orientation = np.array([scenario["start"]["orientation"]])

    end_position = np.array(scenario["end"])

    # ------- Starting of the enviroment -----
    env = VelocityAviary(
        drone_model=DRONE_MODEL,
        num_drones=1,
        initial_xyzs=initial_position,
        initial_rpys=initial_orientation,
        physics=Physics.PYB,
        pyb_freq=PYB_FREQUENCY_HZ,
        ctrl_freq=CONTROL_FREQUENCY_HZ,
        gui=GUI,
    )

    planner_update_steps = max(1, round(env.CTRL_FREQ / PLANNER_FREQ_HZ))

    pybullet_client = env.getPyBulletClient()
    drone_id = env.getDroneIds()[0]
    
    velocity_debug_id = -1
    command_debug_id = -1

    print("Drone IDs:", env.getDroneIds())

    # ----- Proximity sensor -----
    #sensor = ProximitySensor(pybullet_client, drone_id, SENSOR_RADIUS, SHOW_SENSOR_BUBBLE)

    sensor = LidarSensor(pybullet_client, drone_id, LIDAR_RANGE, LIDAR_RAY_COUNT, LIDAR_ELEVATION_ANGLES, LIDAR_MIN_RANGE, debug=LIDAR_DEBUG)

    # ---- Map Creation ------

    local_map = Map(
        DRONE_RADIUS,
        PREFERRED_MARGIN,
        MERGE_DISTANCE,
        HARD_MARGIN
    )
    local_map.update_goal_position(end_position)
    local_map.update_drone_position(initial_position[0])

    # ---- Obstacle creation -----
    obstacle_ids = set()

    for definition in scenario["obstacles"]:
        obstacle_id = create_box_obstacle(
            pybullet_client,
            definition["position"],
            definition["half_extents"],
            definition.get("color", [1.0, 0.0, 0.0, 1.0])
        )

        obstacle_ids.add(obstacle_id)


    # ----- Calculates the number of steps the simulation will have -----

    number_of_steps = int(SIMULATION_DURATION_SECONDS * env.CTRL_FREQ)

    # ----- Action will perform -----

    action = np.array([[0.0, 0.0, 0.0, SPEED_FRACTION]])
    goal_reached=False
    start_time = time.time()

    # ----- Primary loop of execution -------
    for step in range(number_of_steps):

        observation, reward, terminated, truncated, info = env.step(action)

        commanded_vel = action[0,:3] * action[0,3]

        current_pos = observation[0, 0:3]
        current_quat = observation[0, 3:7]
        current_vel = observation[0, 10:13]
        # detections = sensor.detect(obstacle_ids)
        detections = sensor.scan(current_pos, current_quat)

        if SEE_VELOCITY_VECTOR:
            velocity_debug_id = draw_velocity_vector(
                pybullet_client,
                velocity_debug_id,
                current_pos,
                current_vel,
            )

            command_debug_id = draw_velocity_vector(
                pybullet_client,
                command_debug_id,
                current_pos,
                commanded_vel,
                color=(1.0, 1.0, 0)
            )

        local_map.update_drone_position(current_pos)
        for detection in detections:
            if detection["obstacle_id"] in obstacle_ids:
                local_map.add_obstacle(detection["position"])

        if step % planner_update_steps == 0:
            
            next_action, goal_reached = action_with_local_avoidance(
                current_pos,
                end_position,
                local_map,
                SPEED_FRACTION,
                LOOKAHEAD_DISTANCE,
                MIN_ALTITUDE
            )

        action[0] = next_action

        sync(step, start_time, env.CTRL_TIMESTEP)

    env.close()

    if SHOW_MAP:
        local_map.plot(MAP_MODES)

def draw_velocity_vector(
    pybullet_client,
    debug_id,
    position,
    velocity,
    scale=SCALE_VECTOR,
    color=COLOR_VECTOR
):
    position = np.asarray(position, dtype=float)
    velocity = np.asarray(velocity, dtype=float)

    vector_end = position + velocity * scale

    return p.addUserDebugLine(
        position,
        vector_end,
        color,
        lineWidth=3,
        replaceItemUniqueId=debug_id,
        physicsClientId=pybullet_client
    )

if __name__ == "__main__":
    run()
