#ifndef GRIDHELPER_H
#define GRIDHELPER_H


extern "C" {
    bool init_setup();
    double calculate_reward();
    bool can_move(int action, int step_length);
    bool turn_drone_ext(double yaw_dx);
    bool move_ext(int dir_x, int dir_y);
    void stop_logging();
    void init_logging();
}


#endif //GRIDHELPER_H
