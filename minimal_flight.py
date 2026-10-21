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


# ----- Simulation -----
GUI = True
SIMULATION_DURATION_SECONDS = 5
PYB_FREQUENCY_HZ = 240
CONTROL_FREQUENCY_HZ = 48

# ----- Drone -----
DRONE_MODEL = DroneModel.CF2X
INITIAL_POSITION = [0.0, 0.0, 0.1]
INITIAL_ORIENTATION = [0.0, 0.0, 0.0]

# ----- Movement -----
SPEED_FRACTION = 0.25

# ----- Proximity sensor -----
SENSOR_RADIUS = 0.30
SHOW_SENSOR_BUBBLE = True

# ----- Local map -----
DRONE_RADIUS = 0.06
SAFETY_MARGIN = 0.05
MERGE_DISTANCE = 0.05
SHOW_MAP = True

# ----- Scenario -----
SCENARIO_PATH = "scenarios/single_wall.json"


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
    
    start_time = time.time()

    # ----- Primary loop of execution -------
    for step in range(number_of_steps):

        if step < number_of_steps //2:
            action = np.array([[0.0, 1.0, 0.0, SPEED_FRACTION]])
        else:
            action = np.zeros((1,4))
        
        observation, reward, terminated, truncated, info = env.step(action)

        detections = sensor.detect(obstacle_ids)
        local_map.update_drone_position(observation[0, 0:3])

        for detection in detections:
            local_map.add_obstacle(detection["position"])
        
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
        local_map.plot_3d()

if __name__ == "__main__":
    run()
