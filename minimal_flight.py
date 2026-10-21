import numpy as np
import time
import pybullet as p

from gym_pybullet_drones.utils.utils import sync
from gym_pybullet_drones.envs.VelocityAviary import VelocityAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics

from map import Map
from sensors import ProximitySensor
from obstacle_builder import create_box_obstacle
from scenario_loader import load_scenario
from navigation import action_towards_goal, action_with_local_avoidance

# ----- Simulation -----
GUI = True
SIMULATION_DURATION_SECONDS = 20
PYB_FREQUENCY_HZ = 240
CONTROL_FREQUENCY_HZ = 48

# ----- Drone -----
DRONE_MODEL = DroneModel.CF2X
INITIAL_POSITION = [0.0, 0.0, 0.1]
INITIAL_ORIENTATION = [0.0, 0.0, 0.0]

# ----- Movement -----
SPEED_FRACTION = 0.5

# ----- Proximity sensor -----
SENSOR_RADIUS = 0.50
SHOW_SENSOR_BUBBLE = True
# STOP_ON_DETECTION = True

# ----- Local map -----
DRONE_RADIUS = 0.06
SAFETY_MARGIN = 0.20
MERGE_DISTANCE = 0.05
SHOW_MAP = True
LOOKAHEAD_DISTANCE = 0.35

# ----- Scenario -----
SCENARIO_PATH = "scenarios/obstacle_in_path.json"


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

    pybullet_client = env.getPyBulletClient()
    drone_id = env.getDroneIds()[0]

    print("Drone IDs:", env.getDroneIds())

    # ----- Proximity sensor -----
    sensor = ProximitySensor(pybullet_client, drone_id, SENSOR_RADIUS, SHOW_SENSOR_BUBBLE)

    # ---- Map Creation ------

    local_map = Map(
        DRONE_RADIUS,
        SAFETY_MARGIN,
        MERGE_DISTANCE
    )
    local_map.update_goal_position(end_position)
    
    # ---- Obstacle creation -----
    obstacle_ids = []

    for definition in scenario["obstacles"]:
        obstacle_id = create_box_obstacle(
            pybullet_client,
            definition["position"],
            definition["half_extents"],
            definition.get("color", [1.0, 0.0, 0.0, 1.0])
        )

        obstacle_ids.append(obstacle_id)
        
    
    # ----- Calculates the number of steps the simulation will have -----
    
    number_of_steps = int(SIMULATION_DURATION_SECONDS * env.CTRL_FREQ)

    # ----- Action will perform -----
    # actually nothing
    action = np.array([[0.0, 1.0, 0.0, SPEED_FRACTION]])
    goal_reached=False
    start_time = time.time()

    # ----- Primary loop of execution -------
    for step in range(number_of_steps):
        
        observation, reward, terminated, truncated, info = env.step(action)

        current_pos = observation[0, 0:3]
        
        detections = sensor.detect(obstacle_ids)
        
        local_map.update_drone_position(observation[0, 0:3])
        for detection in detections:
            local_map.add_obstacle(detection["position"])

        next_action, goal_reached = action_with_local_avoidance(
            current_pos,
            end_position,
            local_map,
            SPEED_FRACTION,
            LOOKAHEAD_DISTANCE
        )

        action[0] = next_action
        
        if step % env.CTRL_FREQ == 0:
            if detections:
                print("Obstacle detected:", detections)
            else:
                print("No obstacle detected")

            #position = observation[0, 0:3]
            #print("drone position: ", position)
        
        sync(step, start_time, env.CTRL_TIMESTEP)

    env.close()

    if SHOW_MAP:
        local_map.plot()

if __name__ == "__main__":
    run()
