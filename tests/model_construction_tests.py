import sys
import os
import pathlib
sys.path.append("../stompc")

import model_construction


test_state = {
        "x": 79,
        "y": 133,
        "yaw": -1.57,
        "map": "{1}",
        "width_map": 126,
        "height_map": 206,
        "granularity_map": 0.05,
        "open": 1,
        "drone_diameter": 0.6,
        "safety_range": 0.4,
        "laser_range": 4,
        "laser_range_diameter": 3,
        "discovery_reward": 1,
        "turning_cost": 10,
        "moving_cost": 10,
        "pump_exploration_reward": 10000000,
        "dummy": 53243
    }

if __name__ == "__main__":
    print(os.getcwd())
    base_path = os.path.dirname(os.path.realpath(__file__))
    base_path = pathlib.Path(base_path).parent.absolute()
    template_file = os.path.join(base_path, "stompc/drone_model_stompc_continuous.xml")
    print(template_file)
    controller = model_construction.init_pure_uppaal_controller(template_file)
    controller.init_simfile()
    controller.insert_state(test_state)
    controller.debug_copy("controller_ext.xml")