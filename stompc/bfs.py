from collections import deque
from classes import State, DroneSpecs, TrainingParameters, MapConfig
from utils import check_if_drone_can_see_pump, shield_action_bfs, turn_drone
import copy
def get_path_from_bfs(state:State, drone_specs: DroneSpecs, map_config: MapConfig ):
    start = (state.map_drone_index_y, state.map_drone_index_x)
    rows, cols = len(state.map), len(state.map[0])
    queue = deque([(state.map_drone_index_y, state.map_drone_index_x, -1)])
    visited = [[False for _ in range(cols)] for _ in range(rows)]
    parents = [[None for _ in range(cols)] for _ in range(rows)]

    directions = [(10, 0), (0, 10), (-10, 0), (0, -10)]

    start_row, start_col = state.map_drone_index_y, state.map_drone_index_x
    visited[start_row][start_col] = True

    path_to_poi = None
    path_to_unkown_cell = None

    # BFS
    while queue:
        r, c, dir = queue.popleft()

        #update the drone pos, so shield is handled correctly
        #this wont affect the actual state, since this state is never returned. 
        temp_state = copy.deepcopy(state)
        temp_state.map_drone_index_y = r
        temp_state.map_drone_index_x = c

        #found a poi
        for i in range(0,4):
            for pump in map_config.pumps + map_config.fake_pumps:
                if pump.has_been_discovered == False:
                    if check_if_drone_can_see_pump(temp_state,pump,drone_specs):
                        path_to_poi = []
                        while (r, c) != start:
                            parent_r, parent_c = parents[r][c]
                            for i, (dr, dc) in enumerate(directions):
                                if (parent_r + dr, parent_c + dc) == (r, c):
                                    path_to_poi.append(i + 10)
                            r, c = parents[r][c]
                        for j in range(0,i):
                            path_to_poi.append(4)
                        path_to_poi.reverse()
                        return path_to_poi
            temp_state.yaw = turn_drone(temp_state.yaw, -1.57)
        
        #found a unkown cell
        if state.map[r][c] == -1 and path_to_unkown_cell is None:
            print("Found an unknown cell.")
            path_to_unkown_cell = []

            temp_r, temp_c = r, c
            while (temp_r, temp_c) != start:
                parent_r, parent_c = parents[temp_r][temp_c]
                for i, (dr, dc) in enumerate(directions):
                    if (parent_r + dr, parent_c + dc) == (temp_r, temp_c):
                        path_to_unkown_cell.append(i + 10)
                temp_r, temp_c = parents[temp_r][temp_c]
            path_to_unkown_cell.reverse()
            
        

        for i, (dr, dc) in enumerate(directions):
            nr, nc = r + dr, c + dc
            #add the cell to queue if it has not been visited, and it is safe to take
            if 0 <= nr < rows and 0 <= nc < cols and not visited[nr][nc]:
                if shield_action_bfs(i + 10, temp_state,drone_specs) == True:
                    visited[nr][nc] = True
                    parents[nr][nc] = (r, c)
                    queue.append((nr,nc, i + 10))

    
    if path_to_unkown_cell != None:
        return path_to_unkown_cell
    else:
        print("Didn't find any POI or unkown cells")
        return [4,4,4,4] 
