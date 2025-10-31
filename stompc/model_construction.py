import state_to_json as stj
from model_interface import QueueLengthController


def init_pure_uppaal_controller(template_file):
    controller = QueueLengthController(
        templatefile=template_file,
        state_names=["x", "y", "yaw",
                     "width_map", "height_map",
                     "map",
                     "granularity_map",
                     "open",
                     "discovery_reward",
                     "turning_cost",
                     "moving_cost",
                     "drone_diameter",
                     "safety_range",
                     "range_laser",
                     "laser_range_diameter",
                     "pump_exploration_reward",
                     "upper_pump_detection_range",
                     "horizon",
                     "visited",
                     "visited_cost"])

    return controller


def init_uppaal_using_ext_lib_controller(template_file):
    controller = QueueLengthController(
        templatefile=template_file,
        state_names=["horizon",
                     "visited",
                     "visited_cost"])

    return controller


def generate_query_file(controller, horizon):
    optimize = "maxE"
    learning_param = "accum_reward + accum_penalty"

    controller.generate_query_file(optimize, learning_param,
                                   state_vars=["DroneController.DecisionState", "yaw", "x", "y"],
                                   point_vars=["time"],
                                   observables=["action", "accum_reward"],
                                   horizon=horizon)


