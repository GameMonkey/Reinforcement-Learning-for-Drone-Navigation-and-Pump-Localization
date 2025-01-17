import random
import argparse
import os
import rclpy
import sys
import threading
import strategoutil as sutil
import time
import math
import csv
from multiprocessing import Process, Queue, Pipe
from bfs import get_path_from_bfs
sys.path.insert(0, '../')
from dotenv import load_dotenv
load_dotenv()

from gz_utils import run_gz, run_xrce_agent, run_launch_file
from ROS import vehicle_odometry, offboard_control, camera_control, lidar_sensor, odom_publisher, map_processing
import time
from model_interface import QueueLengthController
from bridges import init_rclpy, shutdown_rclpy
from environment import generate_environment
from utils import turn_drone, shield_action, build_uppaal_2d_array_string, run_pump_detection, check_map_closed, measure_coverage, store_shielded_state
from classes import State, DroneSpecs, TrainingParameters
from maps import get_baseline_one_pump_config, get_baseline_two_pumps_config, get_baseline_big_room_config, get_baseline_tetris_room_config,get_baseline_cylinder_room_config

global offboard_control_instance
global odom_publisher_instance
global map_drone_tf_listener_instance

ENV_DOMAIN = os.environ['DOMAIN']
ENV_VERIFYTA_PATH = os.environ['VERIFYTA_PATH']
ENV_GZ_PATH = os.environ['GZ_PATH']
ENV_LAUNCH_FILE_PATH = os.environ['LAUNCH_FILE_PATH']

#Experiment settings
TIME_PER_RUN = 600
RUN_START = None
CURR_TIME_SPENT = 0
ALLOWED_GAP_IN_MAP = 1


half_PI_right = 1.57   # 90 degrees right
half_PI_left = -1.57   # 90 degrees left
full_PI_turn = 3.14    # 180 degress turn
e_turn = 0.05
e_move = 0.1
uppaa_e = 0.5

drone_specs = DroneSpecs(drone_diameter=0.6,safety_range=0.4,laser_range=4,laser_range_diameter=3)
training_parameters = TrainingParameters(open=1, turning_cost=20.0, moving_cost=20.0, discovery_reward=10.0, pump_exploration_reward=1000.0)
learning_args = {
    "max-iterations": "3",
    #"reset-no-better": "5",
    "good-runs": "50",
    "total-runs": "300",
    #"runs-pr-state": "100"
    }

global map_config
map_config = get_baseline_one_pump_config()


def write_to_csv(filename, res):
    with open(filename, 'a+') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(res)

def get_current_state():
    x = float(vehicle_odometry.get_drone_pos_x())
    y = float(vehicle_odometry.get_drone_pos_y())
    yaw = offboard_control_instance.yaw
    state = map_processing.process_map_data(x,y, map_config)
    state.yaw = yaw
    return state
 
def run_action_seq(actions:list):
    """
    Returns TRUE if all actions was successfully executed
    Returns FALSE if all actions was not successfully executed.
    """
    while(len(actions) > 0):
        action_was_activated = activate_action_with_shield(actions.pop(0))
        if(action_was_activated == False):
            return False
    return True


def activate_action_with_shield(action):
    """
    Returns TRUE if action is activated
    Returns FALSE if action is not activated / is not safe.
    """
    state = get_current_state()
    if(shield_action(action,state, drone_specs)):
        state = activate_action(action)
    else:
        print("shielded action: {}".format(action))
        store_shielded_state(state, action)
        run_action_seq([4,4,4,4])
        state = get_current_state()
        if(shield_action(action,state,drone_specs)):
            activate_action(action)
        else:
            print("shielded action: {} twice, training again".format(action))
            return False
    
    return True

def predict_state_based_on_action_seq(action_seq):
    """
    Returns the predicted state of the drone, by taking the action_seq
    Returns None if the action_seq contains an unkown action.
    """
    x = float(vehicle_odometry.get_drone_pos_x())
    y = float(vehicle_odometry.get_drone_pos_y())
    yaw = offboard_control_instance.yaw

    action_seq_copy = [act for act in action_seq]

    while(len(action_seq_copy) > 0):
        action = action_seq_copy.pop(0)
        try:
            x,y,yaw = get_drone_pos_based_on_action(action,x,y,yaw)
        except:
            print("could not predict state due to unkown action: " + action)
            return None
    state = map_processing.process_map_data(x, y,  map_config)
    state.yaw = yaw
    return state
        

    

def get_drone_pos_based_on_action(action,x,y,yaw):
    """
    Returns x,y,yaw based on some action.
    Raises exception if action is unkown.
    """
    match action:
        case 10:
            y-=0.5
        case 11:
            x+=0.5
        case 12:
            y+=0.5
        case 13:
            x-=0.5
        case 20:
            y-=1
        case 21:
            x+=1
        case 22:
            y+=1
        case 23:
            x-=1
        case 4:
            yaw = turn_drone(yaw, half_PI_left)
        case 5:
            yaw = turn_drone(yaw, half_PI_right)
        case 6:
            yaw = turn_drone(yaw,full_PI_turn)
        case _:
            raise Exception("Unkown action")
    return x,y,yaw

def activate_action(action):
    global CURR_TIME_SPENT
    global map_config
    x = float(vehicle_odometry.get_drone_pos_x())
    y = float(vehicle_odometry.get_drone_pos_y())
    yaw = offboard_control_instance.yaw
    action_is_move = False
    try:
        x,y,yaw = get_drone_pos_based_on_action(action,x,y,yaw)
    except:
        print("unkown action")
        state = map_processing.process_map_data(x, y, map_config)
        state.yaw = yaw
        return state


    action_is_move = action > 6
    curr_x = float(vehicle_odometry.get_drone_pos_x())
    curr_y = float(vehicle_odometry.get_drone_pos_y())

    if action_is_move:
        offboard_control_instance.x = x
        offboard_control_instance.y = y
        while((x-e_move> curr_x or curr_x > x+e_move) or (y- e_move > curr_y or curr_y > y+e_move)) and CURR_TIME_SPENT < TIME_PER_RUN:
            time.sleep(0.1)
            curr_x = float(vehicle_odometry.get_drone_pos_x())
            curr_y = float(vehicle_odometry.get_drone_pos_y())
            CURR_TIME_SPENT = time.time() - RUN_START
    else:
        offboard_control_instance.yaw = yaw
        while((yaw - e_turn > odom_publisher_instance.yaw or odom_publisher_instance.yaw > yaw + e_turn) and CURR_TIME_SPENT < TIME_PER_RUN):
            time.sleep(0.1)
            CURR_TIME_SPENT = time.time() - RUN_START
            

    
    state = map_processing.process_map_data(curr_x, curr_y,  map_config)
    state.yaw = yaw
    map_config = run_pump_detection(state,map_config,drone_specs)
    return state


def run(template_file, query_file, verifyta_path):
    global CURR_TIME_SPENT
    print("running uppaal")
    controller = QueueLengthController(
        templatefile=template_file,
        state_names=["x", "y", "yaw", "width_map","height_map", "map", "granularity_map", "open", "discovery_reward", "turning_cost", "moving_cost", "drone_diameter", "safety_range", "range_laser", "laser_range_diameter", "pump_exploration_reward"])
    # initial drone state
    x = float(vehicle_odometry.get_drone_pos_x())
    y = float(vehicle_odometry.get_drone_pos_y())
    action_seq = []
    num_of_actions = 0
    N = 0
    optimize = "maxE"
    learning_param = "accum_reward"
    state = map_processing.process_map_data(x,y, map_config)
    state.yaw = offboard_control_instance.yaw
    controller.generate_query_file(optimize, learning_param,
                                   state_vars=["DroneController.DescisionState"], 
                                   point_vars=["yaw", "x", "y"], 
                                   observables=["action"])


    k = 0
    actions_left_to_trigger_learning = 3  
    train = True
    horizon = 10
    learning_time_accum = 0

    use_baseline = False

    run_action_seq([4,4,4,4])

    while not (all(pump.has_been_discovered for pump in map_config.pumps + map_config.fake_pumps) and check_map_closed(state, ALLOWED_GAP_IN_MAP)) and CURR_TIME_SPENT < TIME_PER_RUN:
        K_START_TIME = time.time()

        if train == True or k % horizon == 0:
            N = N + 1
            print("Beginning trainng for iteration {}".format(N))

            controller.init_simfile()
            
            """ if(len(action_seq) == actions_left_to_trigger_learning):
                state = predict_state_based_on_action_seq(action_seq) """
            
            state = get_current_state()
            # insert current state into simulation template
            uppaal_state = {
                "x": state.map_drone_index_x,
                "y": state.map_drone_index_y,
                "yaw":  state.yaw,
                "map": build_uppaal_2d_array_string("int", "map",  state.map),
                "width_map": state.map_width,
                "height_map": state.map_height,
                "granularity_map": state.map_granularity,
                "open": training_parameters.open, 
                "discovery_reward": training_parameters.disovery_reward, 
                "turning_cost": training_parameters.turning_cost, 
                "moving_cost": training_parameters.moving_cost,
                "pump_exploration_reward": training_parameters.pump_exploration_reward, 
                "drone_diameter": drone_specs.drone_diameter,
                "safety_range": drone_specs.safety_range,
                "range_laser": drone_specs.laser_range, 
                "laser_range_diameter": drone_specs.laser_range_diameter
            }
            controller.insert_state(uppaal_state)
            train = False
            UPPAAL_START_TIME = time.time()

            if use_baseline == False:    
                """parent_conn, child_conn = Pipe()
                    t = Process(target=controller.run, args=(child_conn,query_file,learning_args,verifyta_path,))
                    t.start()
                    while t.is_alive():
                        CURR_TIME_SPENT = time.time() - RUN_START
                        if CURR_TIME_SPENT > TIME_PER_RUN:
                            t.terminate()
                            t.join()
                            break
                        if(len(action_seq) > 0):
                            all_actions_were_activated = run_action_seq(action_seq)
                            action_seq = []
                            if(all_actions_were_activated == False):
                                train = True
                                t.terminate()
                                t.join()
                        else:
                            run_action_seq([4,4,4,4])
                        t.join(0.2)
                    action_seq = list(parent_conn.recv())
                    if(train == True):
                        state = get_current_state()
                        continue"""
                action_seq = controller.run(queryfile=query_file,verifyta_path=verifyta_path,learning_args=learning_args)
            else:
                action_seq = get_path_from_bfs(state, drone_specs, map_config)
            
        
            k = 0
            UPPAAL_END_TIME = time.time()
            K_END_TIME = time.time()
            iteration_time = K_END_TIME-K_START_TIME
            learning_time = UPPAAL_END_TIME-UPPAAL_START_TIME
            learning_time_accum += learning_time
            print("Working on iteration {} took: {:0.4f} seconds, of that training took: {:0.4f} seconds.".format(N, iteration_time, learning_time))
            print("Got action sequence from STRATEGO: ", action_seq)
            #write_to_csv(f'experiments/training_time.csv', [state.map_height * state.map_width, learning_time])
        
        k=k+1
        if(len(action_seq) == 0):
            train = True
            k = 0
        else: 
            action = action_seq.pop(0)
            action_was_activated = activate_action_with_shield(action)
            state = get_current_state()

            if action_was_activated == False:
                train = True
                k = 0
                action_seq = []
            """elif len(action_seq) == actions_left_to_trigger_learning:
                train = True
                k = 0 """
            
            if action_was_activated:
                num_of_actions += 1
        CURR_TIME_SPENT = time.time() - RUN_START

    return all(pump.has_been_discovered for pump in map_config.pumps + map_config.fake_pumps), check_map_closed(state, ALLOWED_GAP_IN_MAP), measure_coverage(get_current_state(), map_config), N, learning_time_accum / N, num_of_actions

def main():
    global offboard_control_instance
    global odom_publisher_instance
    global map_drone_tf_listener_instance
    global RUN_START
    RUN_START = time.time()
    init_rclpy(ENV_DOMAIN)
    run_gz(GZ_PATH=ENV_GZ_PATH)
    time.sleep(10)
    run_xrce_agent()
    time.sleep(3)

    offboard_control_instance = offboard_control.OffboardControl()
    offboard_control.init(offboard_control_instance)
    odom_publisher_instance = odom_publisher.FramePublisher()
    odom_publisher.init(odom_publisher_instance)
    map_drone_tf_listener_instance = vehicle_odometry.MapDroneFrameListener()
    vehicle_odometry.init_map_drone_tf(map_drone_tf_listener_instance)
    ap = argparse.ArgumentParser()
    ap.add_argument("-t", "--template-file", default="drone_model_stompc_continuous.xml", 
        help="Path to Stratego .xml file model template")
    ap.add_argument("-q", "--query-file", default="query.q",
        help="Path to Stratego .q query file")
    ap.add_argument("-v", "--verifyta-path", default=ENV_VERIFYTA_PATH, help=
        "Path to verifyta executable")

    args = ap.parse_args()
    base_path = os.path.dirname(os.path.realpath(__file__)) 
    template_file = os.path.join(base_path, args.template_file)
    query_file = os.path.join(base_path, args.query_file)
    
    while offboard_control_instance.has_aired == False:
        #print(offboard_control_instance.vehicle_local_position.z)
        time.sleep(0.1)

    run_launch_file(LAUNCH_PATH=ENV_LAUNCH_FILE_PATH)   
    time.sleep(5)
    pumps_found, map_closed, room_covered, N, learning_time_accum, num_of_actions = run(template_file, query_file, args.verifyta_path)
    print("Run finished. Turning off drone and getting ready for reset")
    offboard_control_instance.shutdown_drone = True
    #kill_gz()
    return [pumps_found, map_closed, room_covered, CURR_TIME_SPENT / 60, N, learning_time_accum, num_of_actions, True if room_covered > 105 else False], False if room_covered < 10 else True


def create_csv(filename):
    """ Used to create initial csv file  """     
    fields = ['found_all_pumps', 'map_closed', 'coverage_of_room', 'time_taken', 'times_trained', 'avg_training_time', 'actions_activated', 'possible_crash']
    #fields = ['total_cells', 'training_time']
    with open(filename, 'w+') as csv_file:
        writer = csv.writer(csv_file, delimiter=',')
        writer.writerow(fields)

if __name__ == "__main__":
    #file_name = f'Experiment_open={1}_turningcost={20}_movingcost={20}_discoveryreward={10}_pumpreward={1000}_safetyrange={40}cm_maxiter={learning_args["max-iterations"]}_rnb={learning_args["reset-no-better"]}_gr={learning_args["good-runs"]}_tr={learning_args["total-runs"]}_rps={learning_args["runs-pr-state"]}.csv'
    file_name = f'experiments/nobug_Experiment_open=1_turningcost=20_movingcost=20_discoveryreward=10_pumpreward=1000_safetyrange=40cm_maxiter=3_rnb=default_gr={learning_args["good-runs"]}_tr={learning_args["total-runs"]}_rps=default_h=20.csv'
    #create_csv(file_name)

    res, takeoff = main()
    print("\nResults for run:\n   Found all pumps: {}\n   Map closed: {}\n   Total coverage of room: {}\n   Total time taken (in minutes): {}\n   Number of times trained: {}\n   Average training time: {}\n   Number of actions activated: {}\n   Possible crash: {}\n   Takeoff: {}\n".format(res[0],res[1],res[2],res[3],res[4],res[5], res[6], res[7], takeoff))
    if takeoff:
        write_to_csv(file_name, res)