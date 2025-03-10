
# Reinforcement Learning for Drone Navigation and Pump Localization

This project is built as part of our master thesis, and aims to discover the theisability of controlling a drone using Reinforcement Learning. The main task of the drone is to map an entire unkown room, as well as finding points of interest (POI).  

## Authors

- [@sandborg](https://github.com/Sandborg)
- [@maggedelle](https://www.github.com/maggedelle)


## Installation
This installation guide will explain how to run the PX4 part of the software, as well as the Turtlebot 3 demo.

#### Everything runs using ROS2 Humble on a Ubuntu 22.04 LTS


### Needed for both demos

* ROS2 Humble which can be installed [here](https://docs.ros.org/en/humble/Installation.html).

* UPPAAL 5

* Stratego UTILS, installed by the following command
```
pip install strategoutil
```



### PX4 Demo
* Install PX4 and Gazebo using this [guide](https://docs.px4.io/main/en/ros/ros2_comm.html).

* Add SLAM Toolbox to your ROS2 workspace using this [guide](https://github.com/SteveMacenski/slam_toolbox/tree/humble).

* Add ROS_GZ to your ROS2 workspace using this [guide](https://github.com/gazebosim/ros_gz). 

* Install PSUTIL running the command below 

```
pip install psutil
```


* Install Pointcloud to LaserScan using this [guide](https://github.com/ros-perception/pointcloud_to_laserscan/tree/humble).


Start the DEMO by running the following code

```
cd ~/ws_ros2
source install/local_setup.bash

cd ~/Reinforcement-Learning-for-Drone-Navigation-and-Pump-Localization/stompc
python3 stompc.py


[OPTIONAL]
rviz2 -d ~/RL-Controlled-Drone-Using-UPPAAL-ROS/rviz2_cfg/basic_cfg.rviz

```

### Turtlebot3 Demo

* You need a configured Turtlebot3 robot. Follow [this](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/) guide to set it up (Remember to select Humble).

* Install SLAM Cartographer using their own [guide](https://github.com/cartographer-project/cartographer). A guide to start Cartographer can be found [here](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-slam-node) (Remember to select Humble).

* Checkout [this branch](https://github.com/Maggedelle/Reinforcement-Learning-for-Drone-Navigation-and-Pump-Localization/tree/68-showcase-modify-the-codebase-such-that-it-works-on-a-turtlebot3-robot-in-real-life). 

* Run the following code

```bash
    cd ~/ws_ros2
    source install/local_setup.bash

    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_cartographer cartographer.launch.py


    cd ~/Reinforcement-Learning-for-Drone-Navigation-and-Pump-Localization/stompc 
    python3 stompc_turtle.py

    [OPTIONAL]
    rviz2 -d ~/RL-Controlled-Drone-Using-UPPAAL-ROS/rviz2_cfg/basic_cfg.rviz

```
    
## License

[MIT](https://choosealicense.com/licenses/mit/)

