# Reinforcement Learning for Drone Navigation and Pump Localization

This project is built as part of the paper "Exploring Unknown Environments with Uppaal STRATEGO: Safe Reinforcement Learning for Navigation and Pump Localization" presented at [SEFM 2025](https://sefm-conference.github.io/2025/). 

## Authors

- [Magnus Kallestrup Axelsen](https://www.github.com/maggedelle)
- [Martin Kristjansen](https://www.github.com/GameMonkey)
- Kim Guldstrand Larsen
- [Thomas Grubbe Sandborg Lauritsen](https://github.com/Sandborg)

## Paper Abstract

As the capabilities and technologies of Unmanned Aerial Ve-
hicles (UAVs) improve, new ways of utilizing them are being investigated.
We investigate the use of reinforcement learning to navigate a UAV in
an unknown environment, where the room layout is initially unknown.
We present a novel approach for exploring and controlling a UAV, which
must locate points of interest in such rooms. In this approach, we use
reinforcement learning in an online fashion, meaning that the learning
is performed multiple times as our knowledge of the room improves.
We present the implementation of a stochastic model predictive control
approach paired with Q-learning and partition refinement, using Up-
paal Stratego to synthesize near-optimal strategies for UAVs to ex-
plore, map, and locate objects in environments with no prior knowledge.
To ensure the safety of those strategies, we add a pre-shield during learn-
ing and employ a post-shield on the proposed actions to be executed. We
evaluate our approach using simulation and compare it against a greedy
approach, in which the UAV always visits the nearest unexplored part of
the map. Our evaluation shows that the approach explores all points of
interest approximately 11% faster than the baseline, while also reducing
the number of times a new plan must be synthesized by 33%.

## Setup and Installation
The project is built to work is built using [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) on Ubuntu 22.04 LTS (Jammy Jellyfish).
As part of this project, we provide an installation script that will handle the download, installation and setup of the required software.

### Installed with the script:
* [ROS2 Humble](https://docs.ros.org/en/humble/Installation.html),
	* and the required ROS2 packages
* [Gazebo Garden](https://gazebosim.org/docs/garden/install/)
* [PX4](https://docs.px4.io/main/en/ros/ros2_comm.html)
* [Micro XRCE-DDS](https://micro-xrce-dds.docs.eprosima.com/en/latest/)
* The following python packages:
	* [Strategoutil](https://pypi.org/project/strategoutil/)
	* [Numpy="1.26.4"](https://numpy.org/doc/1.26/)
	* [setuptools="70.0.0"](https://pypi.org/project/setuptools/)
	* [python-dotenv](https://pypi.org/project/python-dotenv/)
	* [psutil](https://github.com/giampaolo/psutil)

It is expected that this project is unpacked in the default $HOME folder. Because of this, we recommend using a machine specifically for setting up and running this project.
The script will save the various folder associated with PX4, Micro-XRCE and the workspace for ROS2 at the default $HOME folder for Ubuntu.


### How to use the script:
To use the script, simply run the setup script like so:
```
bash <file_name>.sh
```

During installation, the user will be asked if they want to add the sourcing of two ROS2 specific setup programs to the users .bashrc file.
If the user is using a machine specifically for testing this, we recommend answering yes to all prompts.
If not, the following two lines have to be run in the terminal before running the project: 

```
source /opt/ros/humble/setup.bash
source ~/ws/install/local_setup.bash
```

### UPPAAL
UPPAAL is not installed with the script, but can be downloaded [here](https://uppaal.org/downloads/). Installation instructions can be found on the site aswell.
For the paper UPPAAL 5.0 was used.

## Running the project
The project uses environmen files to let the program know where to locate the verifita component of UPPAAL, the Gazebo/PX4 install location and the launch file for the ROS2 packages used.
Additionally, it is also possible to select a specific ROS2 domain ID, in case multiple people on the same network are working with ROS2. 

The .env file should be placed in the root folder of the project.

The environment file have the following structure:
```
DOMAIN="0" 
VERIFYTA_PATH="<path to the uppaal>/bin/verifyta"
GZ_PATH="<path to PX4 folder>" 
LAUNCH_FILE_PATH="<path-to-repo>/Reinforcement-Learning-for-Drone-Navigation-and-Pump-Localization/launch"
```

Example of a .env file:
```
DOMAIN="0" 
VERIFYTA_PATH="/home/test/Desktop/uppaal-5.0.0-linux64/bin/verifyta" 
GZ_PATH="/home/test/PX4-Autopilot" 
LAUNCH_FILE_PATH="/home/test/Reinforcement-Learning-for-Drone-Navigation-and-Pump-Localization/launch"
```

The ***DOMAIN*** variable is only used to tell the program which `ROS_DOMAIN_ID` to use, however it should also be changed to match the value the the system environment variable for `ROS_DOMAIN_ID`. More information on how to change the `ROS2_DOMAIN_ID` can be found [here](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#the-ros-domain-id-variable), with more information about the `ROS2_DOMAIN_ID` variable specially, [here](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Domain-ID.html).


## Running the project
To run the project, we provide a script that will run all the experiment setups located in the `Reinforcement-Learning-for-Drone-Navigation-and-Pump-Localization/stompc/experiments_setups`. Additionally, one can provide the desired number of runs for each of the setups. If nothing is provided, it will default to a single run.
The script is placed in the ***stompc*** folder and should be run from there.

### Example:
To run the script one can do the following:
```
./run-script.sh 5
``` 
If no setup files are placed in the previously mentioned folder, this will run the default setup configuration 5 times. If one or more setups configurations are placed in the folder, these will be used instead, each with 5 runs.
Each of the 5 runs will have a folder created, each named with the timestamp when the respective run began.
After 5 runs, the script will collect each of the folders associated with the individual runs and save them in a .zip named after the configuration file that was used for the runs.


## Experiment Configuration Setups
In order to be able to define multiple different experiments, each with their own configuration, we have made it possible to create .yaml files describing.
The default configuration, which can be used to test if everything have been installed currectly, can be found in `stompc/experiment_setups/default_config`.
If the user creates custom setups, they should be placed in `stompc/experiment_setups`, so that run-script.sh can use it.

| run_settings | Description                                             | Values                           |
| ------------ | ------------------------------------------------------- | -------------------------------- |
| world        | Which world the experiment should be in                 | Default, Tetris, Large, Cylinder |
| time_per_run | How long each run should take, in seconds               |    			      	    | 
| granularity  | The granularity used in the slam toolbox configuration, it is important that they match  | ideally between 0.05 and 1       | 
| baseline     | whether or not to use the baseline or learning approach | True, False                      |
| horizon      | How many actions we want to learn ahead                 | Should be larger than 0          |


| [uppaal_params](https://docs.uppaal.org/toolsandapi/verifyta/) | Description  | Values                           |
| ----------------- | ------------------------------------------------------------- | -------------------------------- |
| max-iterations    | Maximum total number of iterations in the learning algorithm  | Larger than 0                    |
| good-runs         | How many good runs for each learning                          | Larger than 0                    |
| total-runs        | Number of total runs to attempt for learning                  | Larger than 0                    |

| training_params  | Description                                                           | Values                           |
| ----------------- | ------------------------------------------------------------- | -------------------------------- |
| open         | Whether or not to use the open of closed approach when updating map       | 0 or 1                           |
| turning_cost | The cost in the reward function when taking turn actions                  | Any non-negative real            |
| moving_cost  | The cost in the reward function when taking move actions                  | Any non-negative real            |
| visited_cost | The cost in the reward function when ending in cells already visisted     | Any non-negative real            |
| discovery_reward | The reward when changing a cell in the map                            | Any non-negative real            |
| pump_exploration_reward | The reward when discovering a pump in the map                  | Any non-negative real            |

We do not recommend changing drone_diameter value, since it matches the drone used in this project. However if done, it should match the used drone.
The laser_range and laser_diameter should also match the laser that it being used.
Safety_range can be changed if one is experimenting with how close the drone should be able to get to objects.

| drone_specs | Description                                                           | Values                           |
| ----------------- | ------------------------------------------------------------- | -------------------------------- |
| drone_diameter | How large the drone is, in meters, should match the used drone     |                                  |
| safety_range | Denotes, in meters, how close the drone is allowed to get to objects | Any non-negative real            |
| laser_range | How far the laser can point                                           | Any non-negative real            |
| laser_diameter | How far the laser spreads, from the leftmost dot to the rightmost  | Any non-negative real            |
| upper_pump_detection_range | How close the drone should be to a pump in order to see it | Any non-negative real        | 

## License

[MIT](https://choosealicense.com/licenses/mit/)



