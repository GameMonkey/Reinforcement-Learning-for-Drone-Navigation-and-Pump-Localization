#include "json_reader.h"

#include <fstream>
#include <iostream>

#include "nlohmann/json.hpp"
using json = nlohmann::json;

std::ifstream json_file;

json data;

void load_json_file() {
    std::ifstream json_file{"/home/gp/Reinforcement-Learning-for-Drone-Navigation-and-Pump-Localization/stompc/state.json"};
    data = json::parse(json_file);
}

int get_init_x() {
    try {
        return data["map_config"]["x"];
    }
    catch (json::type_error& e) {
        throw std::runtime_error("Error reading 'x' from JSON configuration");
    }
}

int get_init_y() {
    try {
        return data["map_config"]["y"];
    }
    catch (json::type_error& e) {
        throw std::runtime_error("Error reading 'y' from JSON configuration");
    }
}

double get_init_yaw() {
    try {
        return data["map_config"]["yaw"];
    }
    catch (json::type_error& e) {
        throw std::runtime_error("Error reading 'yaw' from JSON configuration");
    }
}

double get_granularity() {
    try {
        return data["map_config"]["granularity_map"];
    }
    catch (json::type_error& e) {
        throw std::runtime_error("Error reading 'granularity_map' from JSON configuration");
    }
}

bool is_map_open() {
    try {
        int i_value = data["map_config"]["open"];
        return static_cast<bool>(i_value);
    }
    catch (json::type_error& e) {
        throw std::runtime_error("Error reading 'open' from JSON configuration");
    }
}

int get_width() {
    try {
        return data["map_config"]["width_map"];
    }
    catch (json::type_error& e) {
        throw std::runtime_error("Error reading 'width_map' from JSON configuration");
    }
}

int get_height() {
    try {
        return data["map_config"]["height_map"];
    }
    catch (json::type_error& e) {
        throw std::runtime_error("Error reading 'height_map' from JSON configuration");
    }
}

double get_epsilon() {
    try{
        return data["uncertainty"]["epsilon"];
    }
    catch (json::type_error& e) {
        throw std::runtime_error("Error reading 'epsilon' from JSON configuration");
    }
}

double get_epsilon_yaw() {
    try {
        return data["uncertainty"]["epsilon_yaw"];
    }
    catch (json::type_error& e) {
        throw std::runtime_error("Error reading 'epsilon_yaw' from JSON configuration");
    }
}

double get_drone_diameter() {
    try {
        return data["drone_config"]["drone_diameter"];
    }
    catch (json::type_error& e) {
        throw std::runtime_error("Error reading 'drone_diameter' from JSON configuration");
    }
}

double get_safety_range() {
    try {
        return data["drone_config"]["safety_range"];
    }
    catch (json::type_error& e) {
        throw std::runtime_error("Error reading 'safety_range' from JSON configuration");
    }
}

int get_laser_range() {
    try {
        return data["drone_config"]["laser_range"];
    }
    catch (json::type_error& e) {
        throw std::runtime_error("Error reading 'laser_range' from JSON configuration");
    }
}

int get_laser_range_diameter() {
    try {
        return data["drone_config"]["laser_range_diameter"];
    }
    catch (json::type_error& e) {
        throw std::runtime_error("Error reading 'laser_range_diameter' from JSON configuration");
    }
}

double get_pump_detection_range() {
    try {
        return data["drone_config"]["upper_pump_detection_range"];
    }
    catch (json::type_error& e) {
        throw std::runtime_error("Error reading 'upper_pump_detection_range' from JSON configuration");
    }
}


int get_discovery_reward() {
    try {
        return data["rewards_costs"]["discovery_reward"];
    }
    catch (json::type_error& e) {
        throw std::runtime_error("Error reading 'discovery_reward' from JSON configuration");
    }
}

int get_turning_cost() {
    try {
        return data["rewards_costs"]["turning_cost"];
    }
    catch (json::type_error& e) {
        throw std::runtime_error("Error reading 'turning_cost' from JSON configuration");
    }
}

int get_moving_cost() {
    try {
        return data["rewards_costs"]["moving_cost"];
    }
    catch (json::type_error& e) {
        throw std::runtime_error("Error reading 'moving_cost' from JSON configuration");
    }
}

int get_pump_reward() {
    try {
        return data["rewards_costs"]["pump_exploration_reward"];
    }
    catch (json::type_error& e) {
        throw std::runtime_error("Error reading 'moving_cost' from JSON configuration");
    }
}


void load_map(std::vector<std::vector<short>>& init_map) {
    try {
        for (int i = 0; i < get_height(); i++) {
            for (int j = 0; j < get_width(); j++) {
                init_map[i][j] = data["map_config"]["map"][i][j];
            }
        }
    }
    catch (json::type_error& e) {
        throw std::runtime_error("Error converting initial map from JSON configuration into C++ map");
    }

}