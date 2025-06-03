import argparse
import os
import shutil

import rclpy
import sys
import threading
import strategoutil as sutil
import csv
import yaml
import datetime
import signal
from subprocess import Popen, DEVNULL
from bfs import bfs
sys.path.insert(0, '../')
from dotenv import load_dotenv
load_dotenv()


from gz_utils import run_gz, run_xrce_agent, run_launch_file
from ROS import vehicle_odometry, offboard_control, odom_publisher, map_processing
import time
import psutil
from model_interface import QueueLengthController
from bridges import init_rclpy
from utils import turn_drone, shield_action, build_uppaal_2d_array_string, run_pump_detection, check_map_closed, measure_coverage, store_shielded_state
from utils import action_names
from classes import DroneSpecs, TrainingParameters
from maps import get_baseline_one_pump_config, get_baseline_big_room_config, get_baseline_tetris_room_config,get_baseline_cylinder_room_config


import model_construction
from state_to_json import construct_json_state


global offboard_control_instance
global odom_publisher_instance
global map_drone_tf_listener_instance
global res_folder
global actions_taken
global args

ENV_DOMAIN = os.environ['DOMAIN']
ENV_VERIFYTA_PATH = os.environ['VERIFYTA_PATH']
ENV_GZ_PATH = os.environ['GZ_PATH']
ENV_LAUNCH_FILE_PATH = os.environ['LAUNCH_FILE_PATH']

#Non Experimental settings
RUN_START = None
CURR_TIME_SPENT = 0
ALLOWED_GAP_IN_MAP = 1
actions_taken = []
half_PI_right = 1.57   # 90 degrees right
half_PI_left = -1.57   # 90 degrees left
full_PI_turn = 3.14    # 180 degress turn
e_turn = 0.05
e_move = 0.1
uppaa_e = 0.5

ap = argparse.ArgumentParser()
ap.add_argument("-t", "--template-file", default="drone_model_stompc_continuous.xml",
                help="Path to Stratego .xml file model template")
ap.add_argument("-q", "--query-file", default="query.q",
                help="Path to Stratego .q query file")
ap.add_argument("-v", "--verifyta-path", default=ENV_VERIFYTA_PATH, help=
"Path to verifyta executable")
ap.add_argument("-cfg", "--config-file", default="./experiment_setups/default_config/default_run_setup.yaml", help="Path to experiment .yaml config file")

args = ap.parse_args()

#Experiment settings
config_file = args.config_file
global map_config
global USE_BASELINE
USE_BASELINE = False
global HORIZON
HORIZON = 20

with open(config_file) as f:
    config = yaml.safe_load(f)
    drone_cfg = config['experiment_setup']['drone_specs']
    learning_params = config['experiment_setup']['uppaal_params']
    training_params = config['experiment_setup']['training_params']
    TIME_PER_RUN = config['experiment_setup']['run_settings']['time_per_run'] # time allowed in a run in seconds.

    if 'baseline' in config['experiment_setup']['run_settings'].keys():
        USE_BASELINE = config['experiment_setup']['run_settings']['baseline']

    if 'horizon' in config['experiment_setup']['run_settings'].keys():
        HORIZON = config['experiment_setup']['run_settings']['horizon']

    WORLD_TO_USE = config['experiment_setup']['run_settings']['world']

    match WORLD_TO_USE:
        case "Tetris":
            if 'granularity' in config['experiment_setup']['run_settings'].keys():
                granularity = config['experiment_setup']['run_settings']['granularity']
                map_config = get_baseline_tetris_room_config(granularity)
            else:
                map_config = get_baseline_tetris_room_config()
        case "Large":
            if 'granularity' in config['experiment_setup']['run_settings'].keys():
                granularity = config['experiment_setup']['run_settings']['granularity']
                map_config = get_baseline_big_room_config(granularity)
            else:
                map_config = get_baseline_big_room_config()
        case "Cylinder":
            if 'granularity' in config['experiment_setup']['run_settings'].keys():
                granularity = config['experiment_setup']['run_settings']['granularity']
                map_config = get_baseline_cylinder_room_config(granularity)
            else:
                map_config = get_baseline_cylinder_room_config()
        case "Default" | _:
            if 'granularity' in config['experiment_setup']['run_settings'].keys():
                granularity = config['experiment_setup']['run_settings']['granularity']
                map_config = get_baseline_one_pump_config(granularity)
            else:
                map_config = get_baseline_one_pump_config()


    print(map_config.n_cells_in_area)
    drone_specs = DroneSpecs(drone_diameter=drone_cfg['drone_diameter'],
                             safety_range=drone_cfg['safety_range'],
                             laser_range=drone_cfg['laser_range'],
                             laser_range_diameter=drone_cfg['laser_range_diameter'],
                             upper_pump_detection_range=drone_cfg['upper_pump_detection_range']) # To ensure that the model is more pessimistic than the Python Code

    training_parameters = TrainingParameters(open=training_params['open'],
                                             turning_cost=training_params['turning_cost'],
                                             moving_cost=training_params['moving_cost'],
                                             visited_cost=training_params['visited_cost'],
                                             discovery_reward=training_params['discovery_reward'],
                                             pump_exploration_reward=training_params['pump_exploration_reward'],)
    learning_args = {}
    for k,v in learning_params.items():
        learning_args[k] = v



def write_to_csv(filename, res):
    with open(filename, 'a+') as csv_file:
        writer = csv.writer(csv_file)
        csv_file.write("Found all pumps,Map closed,Total coverage of room,Total time taken (in minutes),"
                       "Number of times trained,Average training time,Number of actions activated,Possible crash\n")
        writer.writerow(res)

def get_current_state():
    x = float(vehicle_odometry.get_drone_pos_x())
    y = float(vehicle_odometry.get_drone_pos_y())
    yaw = offboard_control_instance.yaw
    state = map_processing.process_map_data(x,y, map_config)
    state.yaw = yaw
    return state

def run_action_seq(actions:list, step_num, iteration, action_seq):
    """
    Returns TRUE if all actions was successfully executed
    Returns FALSE if all actions was not successfully executed.
    """
    print("Action sequence by names:")
    print([action_names[a] for a in actions])

    while len(actions) > 0:
        action_was_activated = activate_action_with_shield(actions.pop(0), step_num, iteration, action_seq)
        if not action_was_activated:
            return False
    return True


def activate_action_with_shield(action, step_num, iteration, action_seq):
    """
    Returns TRUE if action is activated
    Returns FALSE if action is not activated / is not safe.
    """
    global actions_taken

    state = get_current_state()
    if(shield_action(action,state, drone_specs)):
        try:
            state = activate_action(action)
        except Exception as e:
            Popen("./killall.sh", shell=True).wait()
            raise e
    else:
        print("shielded action: {}".format(action))
        print("shielded action: {}".format(action_names[action]))
        shield_file_location = store_shielded_state(state, action, step_num, iteration, action_seq)
        global res_folder
        os.rename("./" + shield_file_location, "./" +  res_folder + "/" + shield_file_location)
        os.rename("./map.txt", "./" + res_folder + "/" + "./map_iteration-{}_stepNum-{}_action-{}.txt".format(iteration, step_num, action))

        run_action_seq([4,4,4,4], 0, 0, 0)
        actions_taken.append(action_names[4])
        actions_taken.append(action_names[4])
        actions_taken.append(action_names[4])
        actions_taken.append(action_names[4])
        sleep_value = 10
        print("Sleeping {} seconds to give time for the map to update".format(sleep_value))
        time.sleep(sleep_value)

        state = get_current_state()
        if(shield_action(action,state,drone_specs)):
            try:
                activate_action(action)
            except Exception as e:
                Popen("./killall.sh", shell=True).wait()
                raise e
        else:
            print("shielded action: {} twice, training again".format(action))
            print("shielded action: {}".format(action_names[action]))
            shield_file_location = store_shielded_state(state, action, step_num, iteration, action_seq)
            shield_file_location = shield_file_location.split(".")
            os.rename("./" + shield_file_location[0] + "." + shield_file_location[1],
                      "./" + res_folder + "/" + shield_file_location[0] + "_second_shielding." + shield_file_location[1])
            os.rename("./map.txt",
                      "./" + res_folder + "/" + "./map_iteration-{}_stepNum-{}_action-{}_twice.txt".format(iteration,
                                                                                                           step_num, action))
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

    cur_action_start = time.time()
    threshold = 70 # An action should not take more than one 1 and 10 seconds minutes.

    if action_is_move:
        offboard_control_instance.x = x
        offboard_control_instance.y = y
        while((x-e_move> curr_x or curr_x > x+e_move) or (y- e_move > curr_y or curr_y > y+e_move)) and CURR_TIME_SPENT < TIME_PER_RUN:
            time.sleep(0.1)
            curr_x = float(vehicle_odometry.get_drone_pos_x())
            curr_y = float(vehicle_odometry.get_drone_pos_y())
            CURR_TIME_SPENT = time.time() - RUN_START
            if time.time () - cur_action_start > threshold:
                raise Exception("Move action might have crashed")
    else:
        offboard_control_instance.yaw = yaw
        while((yaw - e_turn > odom_publisher_instance.yaw or odom_publisher_instance.yaw > yaw + e_turn) and CURR_TIME_SPENT < TIME_PER_RUN):
            time.sleep(0.1)
            CURR_TIME_SPENT = time.time() - RUN_START
            if time.time () - cur_action_start > threshold:
                raise Exception("Turn action might have crashed")
            

    
    state = map_processing.process_map_data(curr_x, curr_y,  map_config)
    state.yaw = yaw
    map_config = run_pump_detection(state,map_config,drone_specs)
    return state


def run(template_file, template_file_ext, query_file, verifyta_path):
    global CURR_TIME_SPENT
    print("running uppaal")
    controller = model_construction.init_pure_uppaal_controller(template_file)
    controller_ext = model_construction.init_uppaal_using_ext_lib_controller(template_file_ext)

    # initial drone state
    # x = float(vehicle_odometry.get_drone_pos_x())
    # y = float(vehicle_odometry.get_drone_pos_y())
    action_seq = []
    reward_seq = []
    num_of_actions = 0
    N = 0
    # optimize = "maxE"
    # learning_param = "accum_reward + accum_penalty"
    state = None
    # state.yaw = offboard_control_instance.yaw
    model_construction.generate_query_file(controller, HORIZON)

    k = 0
    # actions_left_to_trigger_learning = 3
    train = True
    learning_time_accum = 0

    copy_action_seq = None
# offboard_thread = threading.Thread(target=rclpy.spin, args=(offboard_control_instance,), daemon=True)
    run_drone_thread = threading.Thread(target=run_action_seq, args=([4, 4, 4, 4], 0, 0, 0), daemon=True)
    run_drone_thread.start()

    print("Sending commands to drone.")
    drone_start_time = time.time()
    while run_drone_thread.is_alive():
        # Giving 1 minuttes for the drone to turn
        if time.time() - drone_start_time >= 60:
            print("Drone is non-responsive", file=sys.stderr)
            print("Killing Child Processes", file=sys.stderr)
            pid = os.getpid()
            process = psutil.Process(pid)

            for c in process.children(recursive=True):
                c.kill()

            gz_p = None
            try:
                for p in psutil.process_iter(['cmdline']):
                    if p.info['cmdline'] and 'gz sim' in ' '.join(p.info['cmdline']):
                        gz_p = p
                if gz_p != None:
                    gz_p.send_signal(signal.SIGKILL)
            except (psutil.AccessDenied, psutil.NoSuchProcess):
                print('No process with gz :thinking:')
                pass
            Popen("./killall.sh", shell=True).wait()
            raise Exception("The drone is not responsive")
        time.sleep(0.1)

    run_drone_thread.join()
    print("Drone responds to commands")
    # run_action_seq([4,4,4,4], 0, 0, 0)
    # Initial wait to ensure that the map is updated!
    sleep_value = 10
    print("Started sleeping for {} sec".format(sleep_value))
    time.sleep(sleep_value)


    #while N <= 2:
    while not (all(pump.has_been_discovered for pump in map_config.pumps + map_config.fake_pumps) and measure_coverage(get_current_state(), map_config) > 75) and CURR_TIME_SPENT < TIME_PER_RUN:
        K_START_TIME = time.time()

        if train == True or k % HORIZON == 0:
            N = N + 1
            sleep_value = 10
            print("Started sleeping for {} sec".format(sleep_value))
            time.sleep(sleep_value)
            print("Beginning trainng for iteration {}".format(N))

            controller.init_simfile()
            controller_ext.init_simfile()

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
                "laser_range_diameter": drone_specs.laser_range_diameter,
                "upper_pump_detection_range": drone_specs.upper_pump_detection_range,
                "horizon": HORIZON+2,
                "visited": build_uppaal_2d_array_string("double", "visited", [[state.map_drone_index_x, state.map_drone_index_y, state.yaw] if i == 0 else [0,0,0] for i in range(HORIZON + 2)]),
                "visited_cost": training_parameters.visited_cost,
            }
            controller.insert_state(uppaal_state)
            controller_ext.insert_state(uppaal_state)

            json_state = construct_json_state(uppaal_state, 0.5, 0.2)
            with open("state.json", "w") as f_state:
                f_state.write(json_state)
                shutil.copy("state.json", res_folder + "/state_{}.json".format(N))

            train = False
            UPPAAL_START_TIME = time.time()

            if not USE_BASELINE:
                controller.debug_copy(res_folder + "/Model_of_state_{}.xml".format(N))
                controller_ext.debug_copy(res_folder + "/Model_ext.xml")
                try:
                    action_seq, reward_seq = controller_ext.run(queryfile=query_file,verifyta_path=verifyta_path,learning_args=learning_args, horizon=HORIZON)
                    print("Got action+reward sequence from STRATEGO: ", list(zip(action_seq,reward_seq)))
                    os.rename("./strategy.json", "./" + res_folder + "/strategy_{}.json".format(N))
                except Exception as e:
                    print("UPPAAL might have raised an exception, killing everything and going again.")
                    print(e)
                    Popen("./killall.sh", shell=True).wait()
            else:
                try:
                    action_seq = bfs(state, drone_specs, map_config)
                    print("Got action sequence from BFS approach: ", action_seq)
                    if len(action_seq) == 0:
                        print("Got empty path from BFS, it might be thinking it can see the pump in it's current location, but it needs to turn.")
                        print("Applies four turning actions")
                        action_seq = [4,4,4,4]
                except Exception:
                    print("An exception might have been raised during the baseline search, killing everything and going again.")
                    Popen("./killall.sh", shell=True).wait()

            copy_action_seq = [x for x in action_seq]

            k = 0
            UPPAAL_END_TIME = time.time()
            K_END_TIME = time.time()
            iteration_time = K_END_TIME-K_START_TIME
            learning_time = UPPAAL_END_TIME-UPPAAL_START_TIME
            learning_time_accum += learning_time
            print("Working on iteration {} took: {:0.4f} seconds, of that training took: {:0.4f} seconds.".format(N, iteration_time, learning_time))

            if not USE_BASELINE:
                with open("sequence_{}.csv".format(N), "w") as f:
                    f.write("\n".join([str((action_names[a],r)) for a,r in list(zip(action_seq,reward_seq))]))
                os.rename("./sequence_{}.csv".format(N), "./" + res_folder + "/sequence_{}.csv".format(N))
            else:
                with open("sequence_{}.csv".format(N), "w") as f:
                    f.write("\n".join([action_names[a] for a in action_seq]))
                os.rename("./sequence_{}.csv".format(N), "./" + res_folder + "/sequence_{}.csv".format(N))

            print("Action sequence by name:")
            print([action_names[a] for a in action_seq])

        k=k+1
        if len(action_seq) == 0:
            train = True
            k = 0
        else:
            action = action_seq.pop(0)
            action_was_activated = activate_action_with_shield(action, k, N, copy_action_seq)
            state = get_current_state()

            if not action_was_activated:
                train = True
                k = 0
                action_seq = []
                reward_seq = []
            """elif len(action_seq) == actions_left_to_trigger_learning:
                train = True
                k = 0 """
            if action_was_activated:
                num_of_actions += 1

                if not USE_BASELINE:
                    curr_reward = reward_seq.pop(0)
                    actions_taken.append((action_names[action],curr_reward))

                    print(f'Reward for action {action_names[action]}: {curr_reward}')
                    print("Actions and rewards left: ", list(zip(action_seq,reward_seq)))
                    print("\n\n")
                    ## Checking if there is a value later that is the current value plus one (epsilon) higher.
                    if not any(x > (curr_reward + 1) for x in reward_seq):
                        print('No further action would increase the reward, moving on to next iteration...')
                        train = True
                        k = 0
                        action_seq = []
                        reward_seq = []
                else:
                    actions_taken.append(action_names[action])

        CURR_TIME_SPENT = time.time() - RUN_START

    if N == 0:
        avg_learning_time = 0
    else:
        avg_learning_time = learning_time_accum / N
    return all(pump.has_been_discovered for pump in map_config.pumps + map_config.fake_pumps), check_map_closed(state, ALLOWED_GAP_IN_MAP), measure_coverage(get_current_state(), map_config), N, avg_learning_time, num_of_actions

def main():
    global offboard_control_instance
    global odom_publisher_instance
    global map_drone_tf_listener_instance
    global RUN_START
    global args
    global WORLD_TO_USE
    RUN_START = time.time()
    init_rclpy(ENV_DOMAIN)
    run_gz(GZ_PATH=ENV_GZ_PATH, room_name=WORLD_TO_USE)
    time.sleep(30)
    run_xrce_agent()
    time.sleep(5)

    print("Manually starting onboarding controller")
    executor_controller = rclpy.executors.SingleThreadedExecutor()
    offboard_control_instance = offboard_control.OffboardControl()
    executor_controller.add_node(offboard_control_instance)
    offboard_thread = threading.Thread(target=executor_controller.spin, daemon=True)
    offboard_thread.start()
    time.sleep(10)
    print("Done onboarding controller")

    print("Manually starting Odometry")
    odom_publisher_instance = odom_publisher.FramePublisher()

    executor_odom = rclpy.executors.SingleThreadedExecutor()
    executor_odom.add_node(odom_publisher_instance)
    odom_thread = threading.Thread(target=executor_odom.spin, daemon=True)
    odom_thread.start()
    print("Odom pid:", odom_thread.native_id)
    time.sleep(5)
    print("Done with Odometry")

    print("Manually starting Map Framer")
    map_drone_tf_listener_instance = vehicle_odometry.MapDroneFrameListener()
    #vehicle_odometry.init_map_drone_tf(map_drone_tf_listener_instance)

    executor_frame = rclpy.executors.SingleThreadedExecutor()
    executor_frame.add_node(map_drone_tf_listener_instance)
    frame_thread = threading.Thread(target=executor_frame.spin, daemon=True)
    frame_thread.start()
    print("Frame pid:", frame_thread.native_id)
    time.sleep(10)
    print("Done with Map Framer")


    print("All nodes are spinning")
    base_path = os.path.dirname(os.path.realpath(__file__))
    template_file = os.path.join(base_path, args.template_file)
    template_file_ext = os.path.join(base_path, "drone_model_stompc_continuous_with_external.xml")
    query_file = os.path.join(base_path, args.query_file)

    drone_start_time = time.time()
    while offboard_control_instance.has_aired == False:
        # Giving 1 minuttes for the drone to lift off
        if time.time() - drone_start_time >= 60:
            print("No lift-off of drone", file=sys.stderr)
            print("Killing Child Processes", file=sys.stderr)

            offboard_control_instance.destroy_node()
            odom_publisher_instance.destroy_node()
            map_drone_tf_listener_instance.destroy_node()

            executor_controller.shutdown()
            executor_odom.shutdown()
            executor_frame.shutdown()

            rclpy.shutdown()

            pid = os.getpid()
            process = psutil.Process(pid)

            for c in process.children(recursive=True):
                c.kill()

            gz_p = None
            try:
                for p in psutil.process_iter(['cmdline']):
                    if p.info['cmdline'] and 'gz sim' in ' '.join(p.info['cmdline']):
                        gz_p = p
                if gz_p != None:
                    gz_p.send_signal(signal.SIGKILL)
            except (psutil.AccessDenied, psutil.NoSuchProcess):
                print('No process with gz :thinking:')
                pass

            raise Exception("The drone is stuck in liftoff")

        time.sleep(0.1)

    print("Successful lift-off of drone")

    print("Starting launch")
    run_launch_file(LAUNCH_PATH=ENV_LAUNCH_FILE_PATH)
    time.sleep(15)
    print("Completed Launch")


    try:
        pumps_found, map_closed, room_covered, N, learning_time_accum, num_of_actions = run(template_file, template_file_ext, query_file, args.verifyta_path)
    except Exception as e:
        print("Run Failed. Destroying Nodes")
        print(e)

        offboard_control_instance.destroy_node()
        odom_publisher_instance.destroy_node()
        map_drone_tf_listener_instance.destroy_node()

        executor_controller.shutdown()
        executor_odom.shutdown()
        executor_frame.shutdown()

        pid = os.getpid()
        process = psutil.Process(pid)

        for c in process.children(recursive=True):
            c.kill()

        gz_p = None
        try:
            for p in psutil.process_iter(['cmdline']):
                if p.info['cmdline'] and 'gz sim' in ' '.join(p.info['cmdline']):
                    gz_p = p
            if gz_p != None:
                gz_p.send_signal(signal.SIGKILL)
        except (psutil.AccessDenied, psutil.NoSuchProcess):
            print('No process with gz :thinking:')
            pass

        time.sleep(4)

        raise e

    print("Run finished. Turning off drone and getting ready for reset")
    offboard_control_instance.shutdown_drone = True



    offboard_control_instance.destroy_node()
    odom_publisher_instance.destroy_node()
    map_drone_tf_listener_instance.destroy_node()


    executor_controller.shutdown()
    executor_odom.shutdown()
    executor_frame.shutdown()

    rclpy.shutdown() 
    time.sleep(4)


    return [pumps_found, map_closed, room_covered, CURR_TIME_SPENT / 60, N, learning_time_accum, num_of_actions, True if room_covered > 105 else False], False if room_covered < 10 else True


def create_csv(filename):
    """ Used to create initial csv file  """
    fields = ['found_all_pumps', 'map_closed', 'coverage_of_room', 'time_taken', 'times_trained', 'avg_training_time', 'actions_activated', 'possible_crash']
    #fields = ['total_cells', 'training_time']
    with open(filename, 'w+') as csv_file:
        writer = csv.writer(csv_file, delimiter=',')
        writer.writerow(fields)

def kill_process_and_children(p):
    for child in process.children(recursive=True):  # or parent.children() for recursive=False
        print("Lower Level Child Process: {}".format(child.pid))
        kill_process_and_children(child)
        child.kill()
    p.kill()

if __name__ == "__main__":
    Popen("./killall.sh", shell=True,stdout=DEVNULL, stderr=DEVNULL).wait()
    file_name = "summary.csv"

    global res_folder
    print("Creating results folder")
    res_folder = "./results/"  + datetime.datetime.now().strftime("%Y-%m-%d--%H:%M:%S")
    os.makedirs(res_folder)

    res, takeoff = main()
    print("\nResults for run:\n   Found all pumps: {}\n   Map closed: {}\n   Total coverage of room: {}\n   Total time taken (in minutes): {}\n   Number of times trained: {}\n   Average training time: {}\n   Number of actions activated: {}\n   Possible crash: {}\n   Takeoff: {}\n".format(res[0],res[1],res[2],res[3],res[4],res[5], res[6], res[7], takeoff))
    if takeoff:
        write_to_csv(file_name, res)
        os.rename("./" + file_name,
                  "./" + res_folder + "/" + file_name)

    with open( "./" + res_folder + "/actions_taken.csv", "w") as f:
        f.write("\n".join([str(a) for a in actions_taken]))


    pid = os.getpid()
    print("Main Process: {}".format(pid))
    process = psutil.Process(pid)

    Popen("./killall.sh", shell=True).wait()

    for c in process.children(recursive=True):  # or parent.children() for recursive=False
        print("Child Process: {}".format(c.pid))
        c.kill()

    gz_p = None
    try:
        for p in psutil.process_iter(['cmdline']):
            if p.info['cmdline'] and 'gz sim' in ' '.join(p.info['cmdline']):
                gz_p = p
        if gz_p != None:
            gz_p.send_signal(signal.SIGKILL)
    except (psutil.AccessDenied, psutil.NoSuchProcess):
        print('No process with gz :thinking:')
        pass


    print("Done!")
