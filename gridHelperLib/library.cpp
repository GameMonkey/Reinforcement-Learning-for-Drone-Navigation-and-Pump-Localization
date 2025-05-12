

#include <cstring>
#include <iostream>
#include <fstream>
#include <vector>

//#include "map_data.h"
#include "json_reader.h"


extern "C" {
    bool init_setup();
    double calculate_reward();
    bool can_move(int action, int step_length);
    bool turn_drone_ext(double yaw_dx);
    bool move_ext(int dir_x, int dir_y);
    void stop_logging();
    void init_logging();
}

// Utilitise
double e;
double e_yaw;
const double PI_upper = 3.14;
const double PI_lower = -3.14;
const double PI_half_pos = 1.57;
const double PI_half_neg = -1.57;
const double map_granularity = 0.05;

// Drone specs, all values in meters
double drone_diameter;
double safety_range;
int laser_range;
int laser_range_diameter;


const double half_PI_right = 1.57;   // 90 degrees right
const double half_PI_left = -1.57;   // 90 degrees left
const double full_PI_turn = 3.14;    // 180 degress turn


double discovery_reward;
double turning_cost;
double moving_cost;
double pump_exploration_reward;
bool open_policy;


const double upper_range_pump_detection = 1.25;
//const double lower_range_pump_detection = 0.55;


int x;
int y;
double yaw;
int map_width;
int map_height;
std::vector<std::vector<short>> init_map{};
std::vector<std::vector<short>> map{};


// Log file
static std::ofstream log_file;

bool init_setup() {
    //log_file.open("/home/martin/Desktop/UppaalSpeedUp/log.txt");
    if (map.size() == 0) {
        load_json_file();

        x = get_init_x();
        y = get_init_y();
        yaw = get_init_yaw();

        map_width = get_width();
        map_height = get_height();
        open_policy = is_map_open();

        e = get_epsilon();
        e_yaw = get_epsilon_yaw();

        // Setting drone configurations
        drone_diameter = get_drone_diameter();
        safety_range = get_safety_range();
        laser_range = get_laser_range();
        laser_range_diameter = get_laser_range_diameter();

        // Setting rewards and costs
        discovery_reward = get_discovery_reward();
        turning_cost = get_turning_cost();
        moving_cost = get_moving_cost();
        pump_exploration_reward = get_pump_reward();

        init_map = std::vector(map_height, std::vector<short>(map_width));
        map = std::vector(map_height, std::vector<short>(map_width));

        load_map(init_map);
    }

        // Copy the initial map
        map = init_map;

        // log_file << "Starting New Run...!" << std::endl;
        // log_file.flush();

    return true;
}

void init_logging() {
    // log_file.open("/home/martin/Desktop/UppaalSpeedUp/log.txt");
    //log_file.close();
}

void stop_logging() {
    // log_file.flush();
    // log_file.close();
}


double calculate_reward() {
    // log_file << "Calculating reward...!" << std::endl;
    // log_file << "x = " << x << std::endl;
    // log_file << "y = " << y << std::endl;
    // log_file << "yaw = " << yaw << std::endl;
    // log_file.flush();
    double accum_reward = 0.0;

    //int cells_updated = 0; // The number of cells that have been discovered / changed from unknown
    int N_forward_cells_to_update = static_cast<int>(laser_range / map_granularity); // Gives us the number of drones to check in front of the drone
    int N_diameter_cells_to_update = static_cast<int>(laser_range_diameter / map_granularity); // Gives us the number of cells to update in to the left and right of the drone
    int upper_range_pump_detection_cells = static_cast<int>(upper_range_pump_detection / map_granularity);
    //int lower_range_pump_detection_cells = static_cast<int>(lower_range_pump_detection / map_granularity);

    if(N_forward_cells_to_update % 2 == 0) {
        N_forward_cells_to_update += 1;
    }

    if(N_diameter_cells_to_update % 2 == 0) {
        N_diameter_cells_to_update += 1;
    }

    if(PI_half_neg - e_yaw < yaw && yaw < PI_half_neg + e_yaw) { // exploring in positive y direction
        int lower_bound_x = x - (N_diameter_cells_to_update / 2);
        int upper_bound_x = x + (N_diameter_cells_to_update / 2);
        int upper_bound_y = y + N_forward_cells_to_update;

        int i;
        int j;
        int jj;
        int ui = x;
        int li = x-1;

        if(lower_bound_x < 0) {
            lower_bound_x = 0; // if we reach a point where the cells would go outside of the map in the negative direction, we set the lower bound to 0
        }
        if(upper_bound_x > map_width){
            upper_bound_x = map_width; // if we reach a point where the cells would go outside of the map in the positive direction, we set the upper bound to the width of the map
        }
        if(upper_bound_y > map_height){
            upper_bound_y = map_height; // if we are reaching a point where the range of the drone would go outside the map in the posivtive direction, we set the upper bound for y to map height
        }

        // This is the first fix of the while-for loops
        //while(lower_bound_x <= li || ui < upper_bound_x){
        for(j = y + 1; j < upper_bound_y; j++) {
            while(lower_bound_x <= li || ui < upper_bound_x){
                if(ui < upper_bound_x){
                    if(map[j][ui] == 2) {
                        if(j == y + 1) {
                            ui = upper_bound_x;
                        }
                        j = upper_bound_y;
                    }
                    else if(map[j][ui] == 3) {
                        if(abs(j - y) <= upper_range_pump_detection_cells) {
                            accum_reward = accum_reward + pump_exploration_reward;
                            map[j][ui] = 0;

                        }

                    }
                    else if(map[j][ui] == 1) {
                        accum_reward = accum_reward + discovery_reward;
                        if(!open_policy){
                            map[j][ui] = 2;
                            j = upper_bound_y;
                        }else {
                        map[j][ui] = 0;
                        }
                    }
                }
                ui += 1;
                li -= 1;
            }
        }

        ui = x;
        li = x-1;

        for(j = y + 1; j < upper_bound_y; j++){
            while(lower_bound_x <= li || ui < upper_bound_x){
                if(lower_bound_x <= li){
                    if(map[j][li] == 2) {
                        if(j == y + 1) {
                            li = lower_bound_x - 1;
                        }
                        j = upper_bound_y;
                    }
                    else if(map[j][li] == 3) {
                        if(abs(j - y) <= upper_range_pump_detection_cells) {
                            accum_reward = accum_reward + pump_exploration_reward;
                            map[j][li] = 0;
                        }
                    }
                    else if(map[j][li] == 1) {
                        accum_reward = accum_reward + discovery_reward;
                        if(!open_policy){
                            map[j][li] = 2;
                            j = upper_bound_y;
                        }else {
                            map[j][li] = 0;
                        }
                    }
                }
                ui += 1;
                li -= 1;
            }
        }
    }else if(0 - e_yaw < yaw && yaw < 0 + e_yaw) { // exploring in positive x direction
        int lower_bound_y = y - (N_diameter_cells_to_update / 2);
        int upper_bound_y = y +  (N_diameter_cells_to_update / 2);
        int upper_bound_x = x + N_forward_cells_to_update;

        int i;
        //int j;
        int uj = y;
        int lj = y-1;

        if(lower_bound_y < 0) {
            lower_bound_y = 0; // if we reach a point where the cells would go outside of the map in the negative direction, we set the lower bound to 0
        }
        if(upper_bound_y > map_height){
            upper_bound_y = map_height; // if we reach a point where the cells would go outside of the map in the positive direction, we set the upper bound to the width of the map
        }
        if(upper_bound_x > map_width){
            upper_bound_x = map_width; // if we reach a point where the cells would go outside of the map in the positive direction, we set the upper bound to the width of the map
        }

        while(lower_bound_y <= lj || uj < upper_bound_y) {
            for(i = x + 1; i < upper_bound_x; i++) {

                if(uj < upper_bound_y) {

                    if(map[uj][i] == 2) {
                        if(i == x + 1) {
                            uj = upper_bound_y;
                        }
                        i = upper_bound_x;

                    }
                    else if(map[uj][i] == 3) {
                            if(abs(i - x) <= upper_range_pump_detection_cells) {
                            accum_reward = accum_reward + pump_exploration_reward;
                            map[uj][i] = 0;
                        }

                    }
                    else if(map[uj][i] == 1) {
                        accum_reward = accum_reward + discovery_reward;
                        if(!open_policy){
                            map[uj][i] = 2;
                            i = upper_bound_x;
                        }else {
                            map[uj][i] = 0;
                        }
                    }
                }
            }
            uj += 1;
            lj -= 1;
        }

        // Resetting values
        uj = y;
        lj = y-1;
        while(lower_bound_y <= lj || uj < upper_bound_y) {
            for(i = x + 1; i < upper_bound_x; i++){
                if(lower_bound_y <= lj){
                    if(map[lj][i] == 2) {
                        if( i == x + 1) {
                            lj = lower_bound_y - 1;
                        }
                        i = upper_bound_x;
                    }
                    else if(map[lj][i] == 3) {
                        if(abs(i - x) <= upper_range_pump_detection_cells) {
                            accum_reward = accum_reward + pump_exploration_reward;
                            map[lj][i] = 0;
                        }

                    }
                    else if(map[lj][i] == 1) {
                        accum_reward = accum_reward + discovery_reward;
                        if(!open_policy){
                            map[lj][i] = 2;
                            i = upper_bound_x;
                        }else {
                            map[lj][i] = 0;
                        }
                    }
                }
            }
            uj += 1;
            lj -= 1;
        }
    }else if(PI_half_pos - e_yaw < yaw && yaw < PI_half_pos + e_yaw) { // exploring in negative y direction
        int lower_bound_x = x - (N_diameter_cells_to_update / 2);
        int upper_bound_x = x + (N_diameter_cells_to_update / 2);
        int upper_bound_y = y - N_forward_cells_to_update;

        //int i;
        int j;
        int ui = x;
        int li = x;

        if(lower_bound_x < 0) {
            lower_bound_x = 0; // if we reach a point where the cells would go outside of the map in the negative direction, we set the lower bound to 0
        }
        if(upper_bound_x > map_width){
            upper_bound_x = map_width; // if we reach a point where the cells would go outside of the map in the positive direction, we set the upper bound to the width of the map
        }
        if(upper_bound_y < 0){
            upper_bound_y = 0; // if we are reaching a point where the range of the drone would go outside the map in the negative direction, we set the upper bound for y to 0
        }

        for(j = y; j >= upper_bound_y; j--) {
            while(lower_bound_x <= li || ui < upper_bound_x){
                if(ui < upper_bound_x){
                    if(map[j][ui] == 2) {
                        if(j == y) {
                             ui = upper_bound_x;
                        }
                        j = upper_bound_y;

                    }
                    else if(map[j][ui] == 3) {
                        if(abs(j - y) <= upper_range_pump_detection_cells) {
                            accum_reward = accum_reward + pump_exploration_reward;
                            map[j][ui] = 0;
                         }

                    }
                    else if(map[j][ui] == 1) {
                        accum_reward = accum_reward + discovery_reward;
                        if(!open_policy){
                            map[j][ui] = 2;
                            j = upper_bound_y;
                        }else {
                        map[j][ui] = 0;
                        }
                    }
                }
                ui += 1;
                li -= 1;
            }
        }

        ui = x;
        li = x;

        for(j = y; j >= upper_bound_y; j--){
            while(lower_bound_x <= li || ui < upper_bound_x){
                if(lower_bound_x <= li){
                    if(map[j][li] == 2) {
                        if(j == y) {
                           li = lower_bound_x-1;
                        }
                        j = upper_bound_y;
                    }
                    else if(map[j][li] == 3) {
                        if(abs(j - y) <= upper_range_pump_detection_cells) {
                            accum_reward = accum_reward + pump_exploration_reward;
                            map[j][li] = 0;
                        }

                    }
                    else if(map[j][li] == 1) {
                        accum_reward = accum_reward + discovery_reward;
                        if(!open_policy){
                            map[j][li] = 2;
                            j = upper_bound_y;
                        }else {
                            map[j][li] = 0;
                        }
                    }
                }
                ui += 1;
                li -= 1;
            }
        }
    }else if((PI_lower - e_yaw < yaw && yaw < PI_lower + e_yaw) || (PI_upper - e_yaw < yaw && yaw < PI_upper + e_yaw)) { // exploring in negative x direction
        int lower_bound_y = y - (N_diameter_cells_to_update / 2);
        int upper_bound_y = y +  (N_diameter_cells_to_update / 2);
        int upper_bound_x = x - N_forward_cells_to_update;

        int i;
        //int j;
        int uj = y;
        int lj = y-1;

        if(lower_bound_y < 0) {
            lower_bound_y = 0; // if we reach a point where the cells would go outside of the map in the negative direction, we set the lower bound to 0
        }
        if(upper_bound_y > map_height){
            upper_bound_y = map_height; // if we reach a point where the cells would go outside of the map in the positive direction, we set the upper bound to the width of the map
        }
        if(upper_bound_x < 0){
            upper_bound_x = 0; // if we reach a point where the cells would go outside of the map in the positive direction, we set the upper bound to the width of the map
        }

         while(lower_bound_y <= lj || uj < upper_bound_y) {
            for(i = x - 1; i >= upper_bound_x; i--) {
                if(uj < upper_bound_y) {
                    if(map[uj][i] == 2) {

                        if(i == x -1) {
                           uj = upper_bound_y;
                        }
                        i = upper_bound_x;
                    }
                    else if(map[uj][i] == 3) {
                        if(abs(i - x) <= upper_range_pump_detection_cells) {
                            accum_reward = accum_reward + pump_exploration_reward;
                            map[uj][i] = 0;
                        }

                    }
                    else if(map[uj][i] == 1) {
                        accum_reward = accum_reward + discovery_reward;
                        if(!open_policy){
                            map[uj][i] = 2;
                            i = upper_bound_x;
                        }else {
                            map[uj][i] = 0;
                        }
                    }
                }
            }
            uj += 1;
            lj -= 1;
        }

        uj = y;
        lj = y-1;

        while(lower_bound_y <= lj || uj < upper_bound_y) {
            for(i = x - 1; i >= upper_bound_x; i--) {
                if(lower_bound_y <= lj){
                    if(map[lj][i] == 2) {
                        if(i == x-1) {
                            lj = lower_bound_y-1;
                        }
                        i = upper_bound_x;
                    }
                    else if(map[lj][i] == 3) {
                        if(abs(i - x) <= upper_range_pump_detection_cells) {
                            accum_reward = accum_reward + pump_exploration_reward;
                            map[lj][i] = 0;
                        }

                    }
                    else if(map[lj][i] == 1) {
                        accum_reward = accum_reward + discovery_reward;
                        if(!open_policy){
                            map[lj][i] = 2;
                            i = upper_bound_x;
                        }else {
                            map[lj][i] = 0;
                        }
                    }
                }
            }

            uj += 1;
            lj -= 1;
        }
    }
    // log_file << "Reward: " << accum_reward << std::endl;
    return accum_reward;
}


//          0: West (+y),
//          1: South (+x)
//          2: East (-y),
//          3: North (-x)
bool can_move(int action, int step_length) {
    // log_file << "Checking Can Move with action '" << action << "' and step_length '" << step_length <<   "'" << std::endl;
    // log_file.flush();
    //int N_cells_in_dir = static_cast<int>(((float)step_length / 2.0) / map_granularity);
    int N_cells_in_dir = static_cast<int>(((float)step_length / 2.0) / map_granularity);
    int drone_cells_to_cover = static_cast<int>((drone_diameter) / map_granularity);
    int safety_range_cells = static_cast<int>(safety_range / map_granularity);

    if(drone_cells_to_cover % 2 == 0) {
        drone_cells_to_cover += 1;
    }

    if(action == 0) {
        int lower_bound_x = x - (drone_cells_to_cover / 2) - safety_range_cells;
        int upper_bound_x = x +  (drone_cells_to_cover / 2) + safety_range_cells;
        int upper_bound_y = y + N_cells_in_dir + safety_range_cells;

        int i;
        int j;

        if(lower_bound_x < 0 || upper_bound_x > map_width || upper_bound_y > map_height) {
            return false;
        }
        for(i = lower_bound_x; i < upper_bound_x; i++) {
            for(j = y + (drone_cells_to_cover / 2); j < upper_bound_y; j++) {
                if( map[j][i] == 2 || map[j][i] == 1) {
                    return false;
                }
            }
        }
        // log_file << "Drone can move!"<< std::endl;
        return true;
    }else if(action == 1) {
        int lower_bound_y = y - (drone_cells_to_cover / 2) - safety_range_cells;
        int upper_bound_y = y +  (drone_cells_to_cover / 2) + safety_range_cells;
        int upper_bound_x = x + N_cells_in_dir + safety_range_cells;

        int i;
        int j;

         if(lower_bound_y < 0 || upper_bound_y > map_height || upper_bound_x > map_width) {
            return false;
        }
        for(i = x + (drone_cells_to_cover / 2); i < upper_bound_x; i++) {
            for(j = lower_bound_y; j < upper_bound_y; j++) {
                if(map[j][i] == 2 || map[j][i] == 1) {
                    return false;
                }
            }
        }
        // log_file << "Drone can move!"<< std::endl;
        return true;
    }else if(action == 2) {
        int lower_bound_x = x - (drone_cells_to_cover / 2) - safety_range_cells;
        int upper_bound_x = x +  (drone_cells_to_cover / 2) + safety_range_cells;
        int lower_bound_y = y - N_cells_in_dir - safety_range_cells;

        int i;
        int j;

        if(lower_bound_x < 0 || upper_bound_x > map_width || lower_bound_y < 0) {
            return false;
        }
        for(i = lower_bound_x; i < upper_bound_x; i++) {
            for(j = y - (drone_cells_to_cover / 2); j >= lower_bound_y; j--) {
                if(map[j][i] == 2 || map[j][i] == 1) {
                    return false;
                }
            }
        }
        // log_file << "Drone can move!"<< std::endl;
        return true;
    }else if(action == 3) {
        int lower_bound_y = y - (drone_cells_to_cover / 2) - safety_range_cells;
        int upper_bound_y = y +  (drone_cells_to_cover / 2) + safety_range_cells;
        int lower_bound_x = x - N_cells_in_dir - safety_range_cells;

        int i;
        int j;

       if(lower_bound_y < 0 ||upper_bound_y > map_height || lower_bound_x < 0) {
            return false;
        }
        for(i = x - (drone_cells_to_cover / 2); i >= lower_bound_x; i--) {
            for(j = lower_bound_y; j < upper_bound_y; j++) {
                if(map[j][i] == 2 || map[j][i] == 1) {
                    return false;
                }
            }
        }
        // log_file << "Drone can move!"<< std::endl;
        return true;
    }
    return false;
}

bool turn_drone_ext(double yaw_dx) {
    // log_file << "Turning Drone...!" << std::endl;
    // log_file << "Dir: " << yaw_dx << std::endl;
    // log_file.flush();
    if(yaw >=PI_upper && yaw_dx > 0) {
        yaw = PI_lower + yaw_dx;
    } else if (yaw <= PI_lower && yaw_dx < 0) {
        yaw = PI_upper + yaw_dx;
    }else if(yaw + yaw_dx > PI_upper) {
        if(yaw == PI_half_pos || yaw == PI_half_neg) {
            yaw = yaw * -1;
        }else {
            yaw = PI_lower + yaw_dx;
        }
    }else {
        yaw = yaw + yaw_dx;
    }

    return true;
}

bool move_ext(int dir_x, int dir_y) {
    // log_file << "Moving Drone...!" << std::endl;
    // log_file << "dir_x: " << dir_x << std::endl;
    // log_file << "dir_y: " << dir_y << std::endl;
    double d_dir_x = 0.0;
    double d_dir_y = 0.0;

    if(dir_x == 1) {
        d_dir_x = 0.5;
    }else if(dir_x == -1) {
        d_dir_x = -0.5;
    }else if(dir_y == 1) {
        d_dir_y = 0.5;
    }else if(dir_y == -1) {
        d_dir_y = -0.5;
    }

    if(dir_x == 2) {
        d_dir_x = 1.0;
    }else if(dir_x == -2) {
        d_dir_x = -1.0;
    }else if(dir_y == 2) {
        d_dir_y = 1.0;
    }else if(dir_y == -2) {
        d_dir_y = -1.0;
    }

    // log_file << "d_dir_x: " << d_dir_x << std::endl;
    // log_file << "d_dir_y: " << d_dir_y << std::endl;

    x = x + static_cast<int>(d_dir_x / map_granularity);
    y = y + static_cast<int>(d_dir_y / map_granularity);

    // log_file << "New coordinate: [" << x << "][" << y << "]" << std::endl;
    // log_file.flush();

    return true;
}

void hello() {
    std::cout << "Hello, World!" << std::endl;
}

int main() {
    ///hello();
    load_json_file();
    init_setup();
    // std::cout << "This is the initial x-value: " << get_init_x() << std::endl;
    // std::cout << "This is the initial y-value: " << get_init_y() << std::endl;
    // std::cout << "This is the initial yaw-value: " << get_init_yaw() << std::endl;
    // std::cout << "This is the width-value: " << get_width() << std::endl;
    // std::cout << "This is the height-value: " << get_height() << std::endl;
    // std::cout << "This is the initial epsilon-value: " << get_epsilon() << std::endl;
    // std::cout << "This is the initial epsilon-yaw-value: " << get_epsilon_yaw() << std::endl;
    // std::cout << "This is the granularity-value: " << get_granularity() << std::endl;
    // std::cout << "Is the map 'open': " << is_map_open << std::endl;


    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 2; j++) {
            std::cout << "(i, j) = " << i << ", " << j << std::endl;
            std::cout << can_move(i, j) << std::endl;
        }
    }

    move_ext(2, 1);

    // for (int i = 0; i < map_height; i++) {
    //     for (int j = 0; j < map_width; j++) {
    //         std::cout << map[i][j] << ", ";
    //     }
    //     std::cout << std::endl;
    // }
    //
    // map[0][0] = 10;
    //
    // std::cout << "The value of the first field is: " << map[0][0] << std::endl << std::endl;
    //
    init_setup();
    //
    //
    // for (int i = 0; i < map_height; i++) {
    //     for (int j = 0; j < map_width; j++) {
    //         std::cout << map[i][j] << ", ";
    //     }
    //     std::cout << std::endl;
    // }

}