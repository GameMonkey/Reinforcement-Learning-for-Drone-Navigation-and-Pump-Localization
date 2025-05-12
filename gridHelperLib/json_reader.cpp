#include "json_reader.h"

#include <fstream>

#include "nlohmann/json.hpp"
using json = nlohmann::json;

std::ifstream json_file;

json data;

void load_json_file() {
    std::ifstream json_file{"/home/martin/Desktop/UppaalSpeedUp/gridHelperLib/test_file.json"};
    data = json::parse(json_file);
}

int get_init_x() {
    return data["map_config"]["init_x"];
}

int get_init_y() {
    return data["map_config"]["init_y"];
}

double get_init_yaw() {
    return data["map_config"]["init_yaw"];
}

double get_granularity() {
    return data["map_config"]["granularity"];
}

bool is_map_open() {
    return data["map_config"]["open"];
}

int get_width() {
    return data["map_config"]["width"];
}

int get_height() {
    return data["map_config"]["height"];
}

double get_epsilon() {
    return data["uncertainty"]["epsilon"];
}

double get_epsilon_yaw() {
    return data["uncertainty"]["epsilon_yaw"];
}

double get_drone_diameter() {
    return data["drone_config"]["drone_diameter"];
}

double get_safety_range() {
    return data["drone_config"]["safety_range"];
}

int get_laser_range() {
    return data["drone_config"]["laser_range"];
}

int get_laser_range_diameter() {
    return data["drone_config"]["laser_range_diameter"];
}

int get_discovery_reward() {
    return data["rewards_costs"]["discovery_reward"];
}

int get_turning_cost() {
    return data["rewards_costs"]["turning_cost"];
}

int get_moving_cost() {
    return data["rewards_costs"]["moving_cost"];
}

int get_pump_reward() {
    return data["rewards_costs"]["pump_exploration_reward"];
}


void load_map(std::vector<std::vector<short>>& init_map) {

    for (int i = 0; i < get_height(); i++) {
        for (int j = 0; j < get_width(); j++) {
            init_map[i][j] = data["map_config"]["init_map"][i][j];
        }
    }

}