#include "gridHelper.h"
#include "json_reader.h"


#include <iostream>

int main() {
    load_json_file();
    init_setup();
    // std::cout << "This is the initial x-value: " << get_init_x() << std::endl;
    // std::cout << "This is the initial y-value: " << get_init_y() << std::endl;
    // std::cout << "This is the initial yaw-value: " << get_init_yaw() << std::endl;
    // std::cout << "This is the width-value: " << get_width() << std::endl;
    // std::cout << "This is the height-value: " << get_height() << std::endl;
    // std::cout << "This is the initial epsilon-value: " << get_epsilon() << std::endl;
    // std::cout << "This is the initial epsilon-yaw-value: " << get_epsilon_yaw() << std::endl;
    //
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