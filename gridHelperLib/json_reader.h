#ifndef JSON_READER_H
#define JSON_READER_H

#include <vector>

void load_json_file();
// Map config functions
int get_init_x();
int get_init_y();
double get_init_yaw();
double get_granularity();
bool is_map_open();
int get_height();
int get_width();

void load_map(std::vector<std::vector<short>>& init_map);

// Epsilon values
double get_epsilon();
double get_epsilon_yaw();

// Drone values
double get_drone_diameter();
double get_safety_range();
int get_laser_range();
int get_laser_range_diameter();


// Rewards and costs
int get_discovery_reward();
int get_turning_cost();
int get_moving_cost();
int get_pump_reward();



#endif //JSON_READER_H
