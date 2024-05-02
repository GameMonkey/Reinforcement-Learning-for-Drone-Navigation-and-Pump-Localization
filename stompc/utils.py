import math
from classes import State, DroneSpecs, MapConfig, Pump
from collections import Counter

PI_upper = 3.14
PI_lower = -3.14
PI_half_pos = 1.57
PI_half_neg = -1.57

def turn_drone(yaw, yaw_dx):
    if yaw >= PI_upper and yaw_dx > 0: 
        yaw = PI_lower + yaw_dx
    elif yaw <= PI_lower and yaw_dx < 0:
        yaw = PI_upper + yaw_dx
    elif yaw + yaw_dx > PI_upper: 
        if yaw == PI_half_pos or yaw == PI_half_neg:
            yaw = yaw * -1
        else:
            yaw = PI_lower + yaw_dx
    else:
        yaw = yaw + yaw_dx
        
    return yaw


def shield_action(action: int, state:State, drone_specs: DroneSpecs) -> bool:
    """
    Returns TRUE if action is SAFE.
    Returns FALSE if action is UNSAFE.
    """
    step_length = 0
    if(action > 14):
        step_length = 1.0
    else:
        step_length = 0.5
    N_cells_in_dir = int(step_length / state.map_granularity)
    drone_cells_to_cover = int((drone_specs.drone_diameter) / state.map_granularity)
    safety_range_cells = int(drone_specs.safety_range / state.map_granularity)
    
    if drone_cells_to_cover % 2 == 0:
        drone_cells_to_cover += 1
    
    
    match action:
        case 10 | 20:
            lower_bound_x = int(state.map_drone_index_x - (drone_cells_to_cover / 2) - safety_range_cells)
            upper_bound_x = int(state.map_drone_index_x +  (drone_cells_to_cover / 2) + safety_range_cells)
            upper_bound_y = int(state.map_drone_index_y + N_cells_in_dir + safety_range_cells)


            if lower_bound_x < 0 or upper_bound_x > state.map_width or upper_bound_y > state.map_height:
                return False
            
            for i in range(lower_bound_x, upper_bound_x):
                for j in range (state.map_drone_index_y + 1, upper_bound_y):
                    if state.map[j][i] == 100 or state.map[j][i] == -1:
                        return False
                    
                
            
            return True
        case 11 | 21:
            lower_bound_y = int(state.map_drone_index_y - (drone_cells_to_cover / 2) - safety_range_cells)
            upper_bound_y = int(state.map_drone_index_y +  (drone_cells_to_cover / 2) + safety_range_cells)
            upper_bound_x = int(state.map_drone_index_x + N_cells_in_dir + safety_range_cells)


            if lower_bound_y < 0 or upper_bound_y > state.map_height or upper_bound_x > state.map_width:
                return False
            
            for i in range(state.map_drone_index_x + 1, upper_bound_x):
                for j in range (lower_bound_y, upper_bound_y):
                    if state.map[j][i] == 100 or state.map[j][i] == -1:
                        return False
                    
            return True
        case 12 | 22:
            lower_bound_x = int(state.map_drone_index_x - (drone_cells_to_cover / 2) - safety_range_cells)
            upper_bound_x = int(state.map_drone_index_x +  (drone_cells_to_cover / 2) + safety_range_cells)
            lower_bound_y = int(state.map_drone_index_y - N_cells_in_dir - safety_range_cells)

          

            if lower_bound_x < 0 or upper_bound_x > state.map_width or lower_bound_y < 0:
                return False
            

            for i in range(lower_bound_x, upper_bound_x):
                for j in reversed(range(lower_bound_y, state.map_drone_index_y - 1)):
                    if state.map[j][i] == 100 or  state.map[j][i] == -1: 
                        return False
                    
            return True
        case 13 | 23:
            lower_bound_y = int(state.map_drone_index_y - (drone_cells_to_cover / 2) - safety_range_cells)
            upper_bound_y = int(state.map_drone_index_y +  (drone_cells_to_cover / 2) + safety_range_cells)
            lower_bound_x = int(state.map_drone_index_x - N_cells_in_dir - safety_range_cells)

          

            if lower_bound_y < 0 or upper_bound_y > state.map_height or lower_bound_x < 0:
                return False
            

            for i in reversed(range(lower_bound_x, state.map_drone_index_x - 1)):
                for j in range(lower_bound_y, upper_bound_y):
                    if state.map[j][i] == 100 or  state.map[j][i] == -1: 
                        return False
                    
            return True
        case _:
            return True
          
def run_pump_detection(state:State, map_config: MapConfig, drone_specs: DroneSpecs) -> MapConfig:
    """
    Returns MapConfig object, with updated pump.has_been_discovered values.
    """
    for pump in map_config.pumps + map_config.fake_pumps:
        if pump.has_been_discovered == False:
            pump.has_been_discovered = check_if_drone_can_see_pump(state, pump, drone_specs)
            if pump.has_been_discovered:
                print("found something!")
        
    return map_config


def get_map_index_of_pump(state:State, pump: Pump):
    x_index = math.floor((pump.x) / state.map_granularity) + state.map_odom_index_x
    y_index = math.floor((pump.y) / state.map_granularity) + state.map_odom_index_y

    return x_index, y_index

def check_if_drone_can_see_pump(state:State, pump: Pump, drone_specs: DroneSpecs) -> bool:
    """
    Returns true if drone can see given pump.
    Returns false if drone cannot see given pump.
    """
    e_yaw = 0.2
    x_index, y_index = get_map_index_of_pump(state,pump)
    
    drone_diameter_cells = int(0.75 / state.map_granularity) #TODO change to interval like in UPPAAL, 0.75 should be a TAG
    # check if x_index or y_index is out of bounds.
    # this can happend if the pumps has not been explored yet.
    if x_index > state.map_width or y_index > state.map_height:
        return False
    
    n_foward_cells_to_search = int(0.75 / state.map_granularity) #TODO change to interval like in UPPAAL, 0.75 should be a TAG
    n_diamter_cells_to_search = int(drone_specs.laser_range_diameter / state.map_granularity)

    if(n_foward_cells_to_search % 2 == 0):
        n_foward_cells_to_search+=1
    
    if(n_diamter_cells_to_search % 2 == 0):
        n_diamter_cells_to_search+=1
    
    #exploring in positive y direc
    if PI_half_neg - e_yaw < state.yaw and state.yaw < PI_half_neg + e_yaw:
        lower_bound_x = state.map_drone_index_x - int(n_diamter_cells_to_search / 2)
        upper_bound_x = state.map_drone_index_x + int(n_diamter_cells_to_search / 2)
        upper_bound_y = state.map_drone_index_y + n_foward_cells_to_search

        if lower_bound_x < 0:
            lower_bound_x = 0
        if upper_bound_x > state.map_width:
            upper_bound_x = state.map_width
        if upper_bound_y > state.map_height:
            upper_bound_y = state.map_height
        
        for i in range(lower_bound_x, upper_bound_x):
            for j in range(state.map_drone_index_y + 1, upper_bound_y):
                if state.map[j][i] == 100:
                    j = upper_bound_y
                elif i == x_index and j == y_index:
                    return True
                
    #exploring in positive x direction
    elif 0 - e_yaw < state.yaw and state.yaw < 0 + e_yaw:
        lower_bound_y = state.map_drone_index_y - int(n_diamter_cells_to_search / 2)
        upper_bound_y = state.map_drone_index_y + int(n_diamter_cells_to_search / 2)
        upper_bound_x = state.map_drone_index_x + n_foward_cells_to_search

        if lower_bound_y < 0:
            lower_bound_y = 0
        if upper_bound_y > state.map_height:
            upper_bound_y = state.map_height
        if upper_bound_x > state.map_width:
            upper_bound_x = state.map_width

        for j in range(lower_bound_y, upper_bound_y):
            for i in  range(state.map_drone_index_x + 1, upper_bound_x):
                if state.map[j][i] == 100:
                    i = upper_bound_x
                elif i == x_index and j == y_index:
                    return True
    
    #exploring in negative y direction            
    elif PI_half_pos - e_yaw < state.yaw and state.yaw < PI_half_pos + e_yaw:
        lower_bound_x = state.map_drone_index_x - int(n_diamter_cells_to_search / 2)
        upper_bound_x = state.map_drone_index_x + int(n_diamter_cells_to_search / 2)
        lower_bound_y = state.map_drone_index_y - n_foward_cells_to_search

        if lower_bound_x < 0:
            lower_bound_x = 0
        if upper_bound_x > state.map_width:
            upper_bound_x = state.map_width
        if lower_bound_y < 0:
            upper_bound_y = 0
        
        for i in range(lower_bound_x, upper_bound_x):
            for j in reversed(range(lower_bound_y, state.map_drone_index_y - 1)):
                if state.map[j][i] == 100:
                    j = lower_bound_y - 1
                elif i == x_index and j == y_index:
                    return True
    elif ((PI_lower - e_yaw < state.yaw and state.yaw < PI_lower + e_yaw) 
          or (PI_upper - e_yaw < state.yaw and state.yaw < PI_upper + e_yaw)):
        lower_bound_y = state.map_drone_index_y - int(n_diamter_cells_to_search / 2)
        upper_bound_y = state.map_drone_index_y + int(n_diamter_cells_to_search / 2)
        lower_bound_x = state.map_drone_index_x - n_foward_cells_to_search
        

        if lower_bound_y < 0:
            lower_bound_y = 0
        if upper_bound_y > state.map_height:
            upper_bound_y = state.map_height
        if lower_bound_x < 0:
            lower_bound_x = 0

        for j in range(lower_bound_y, upper_bound_y):
            for i in reversed(range(lower_bound_x, state.map_drone_index_x - 1)):
                if state.map[j][i] == 100:
                    i = lower_bound_x - 1
                elif i == x_index and j == y_index:
                    return True 

    return False
  
def build_uppaal_2d_array_string(type, name, array):
    """
    Builds a 2D array to be inserted into uppaal from a python lists of lists.

    @type: denotes the type of the array when it's inserted into uppaal
    @name: the name the array is going to have in uppaal
    @array: the content of the array
    """
    uppaal_array = "{} {}[{}][{}] = ".format(type, name, len(array), len(array[0]))
    uppaal_array += "{\n"

    lst_strings = []
    for lst_ele in array:
        arr_string = "  {"
        arr_string += ','.join([str(x) for x in lst_ele])
        arr_string += "}"
        lst_strings.append(arr_string)
    uppaal_array += ',\n'.join(lst_strings) + "\n}"


    return uppaal_array

def unpack_array(array, array_name):
    """
    Unpacks an array so that it can be used in training query
    
    @arr: the array to unpack
    @arr_name: the name of the array in uppaal
    """
    lst_string = []
    for i in range(0, len(array)):
        for j in range(0, len(array[0])):
            lst_string.append("{}[{}][{}]".format(array_name, i,j))

    return ",".join(lst_string) 

def measure_coverage(state: State, map_cfg: MapConfig) -> float:
    """
    measure_coverage: State -> MapConfig -> float
    @state: the state containing map
    @map_cfg: the map config containing the number of cells that the floor comprise of

    Returns the % measure of how much is discovered compared to how many cells we know there are.
    """
    map = state.map
    N_cells_total = map_cfg.n_cells_in_area
    N_cells_covered= 0

    for row in map:
        cnt = Counter(row)
        N_cells_covered += cnt[0]
    
    return (N_cells_covered / N_cells_total) * 100


def check_map_closed(state: State, skip:int) -> bool:
    """
    @state: the state containing the map to be checked and all relevant information
    @skip: how many cells can be "open" before it's determined that the map is not closed. This should be in meters because it will get converted to cells with the map_granularity

    Returns true if the map is closed, false otherwise
    """
    map = state.map
    map_width = state.map_width
    map_height = state.map_height 
    open_cells = math.floor(skip / state.map_granularity)

    cnt_open_left = 0
    cnt_open_right = 0

    for i in range(0,map_height):
        found_left = None
        found_right = None
        for j in range(0,map_width):
            if found_left == None:
                if map[i][j] == '+':
                    cnt_open_left += 1
                    if cnt_open_left == open_cells:
                        return False
                    found_left = True
                elif map[i][j] == '-':
                    cnt_open_left = 0
                    found_left = True
            if found_right == None:
                if map[i][map_width-j-1] == '+':
                    cnt_open_right += 1
                    if cnt_open_right == open_cells:
                        return False
                    found_right = True
                elif map[i][map_width-j-1] == '-':
                    cnt_open_right = 0
                    found_left = True
            if found_left and found_right:
                break

    cnt_open_down = 0
    cnt_open_up = 0
    
    for i in range(0, map_width):
        found_down = None
        found_up = None
        for j in range(0, map_height):
            if found_down == None:
                if map[j][i] == '+':
                    cnt_open_down += 1
                    if cnt_open_down == open_cells:
                        return False
                    found_down = True
                elif map[j][i] == '-':
                    cnt_open_down = 0
                    found_down = True
            if found_up == None:
                if map[map_height-j-1][i] == '+':
                    cnt_open_up += 1
                    if cnt_open_up == open_cells:
                        return False
                    found_up = True
                elif map[map_height-j-1][i] == '-':
                    cnt_open_up = 0
                    found_up = True
            if found_down and found_up:
                break
                    
    return True
