#!/usr/bin/env bash
set -e  # Exit on error

# Timestamped log file
TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
LOGFILE="install_${TIMESTAMP}.log"

# Print log path at the very beginning
echo "===================================================="
echo "Installation starting..."
echo "All output is being logged to: $LOGFILE"
echo "===================================================="

# Start logging (both terminal + file)
exec > >(tee -i "$LOGFILE") 2>&1

# Helper for pretty step headers
step() {
    echo ""
    echo "===================================================="
    echo "[$1/$TOTAL_STEPS] $2"
    echo "===================================================="
    echo ""
}

TOTAL_STEPS=8
CURRENT_STEP=1

# 1. System update
step $CURRENT_STEP "System Update"
CURRENT_STEP=$((CURRENT_STEP+1))
sudo apt-get update
sudo apt-get upgrade -y

# 2. Python deps
step $CURRENT_STEP "Installing Python dependencies"
CURRENT_STEP=$((CURRENT_STEP+1))
sudo apt install python3-pip -y
pip3 install "numpy==1.26.4" "setuptools==70.0.0" python-dotenv psutil strategoutil

# 3. ROS2 Humble
step $CURRENT_STEP "Installing ROS2 Humble"
CURRENT_STEP=$((CURRENT_STEP+1))
sudo apt install software-properties-common -y
sudo add-apt-repository universe -y
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update
sudo apt install ros-humble-desktop ros-dev-tools=1.0.1 python3-colcon-common-extensions -y

# Ask about sourcing ROS
read -p "Do you want to add ROS sourcing to ~/.bashrc? [y/N]: " add_ros
if [[ "$add_ros" =~ ^[Yy]$ ]]; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi
source /opt/ros/humble/setup.bash

# 4. Gazebo Garden
step $CURRENT_STEP "Installing Gazebo Garden"
CURRENT_STEP=$((CURRENT_STEP+1))
sudo apt-get install curl lsb-release gnupg -y
sudo curl https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
| sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-garden -y

# 5. PX4 (correct copy logic)
step $CURRENT_STEP "Installing PX4 Autopilot"
CURRENT_STEP=$((CURRENT_STEP+1))
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git || true
cd PX4-Autopilot
git reset --hard e35380d6ae8196ce658b5059388593ec0c3b63d2
git submodule update --recursive
bash ./Tools/setup/ubuntu.sh --no-sim-tools

# Source folders (your repo paths)
SRC_MODELS="$HOME/Reinforcement-Learning-for-Drone-Navigation-and-Pump-Localization/gazebo_models"
SRC_WORLDS="$HOME/Reinforcement-Learning-for-Drone-Navigation-and-Pump-Localization/gazebo_worlds"

PX4_WORLD_DST="$HOME/PX4-Autopilot/Tools/simulation/gz/worlds"
PX4_MODEL_DST="$HOME/PX4-Autopilot/Tools/simulation/gz/models"

# Copy worlds
if [[ -d "$SRC_WORLDS" && -d "$PX4_WORLD_DST" ]]; then
    echo "Copying worlds from $SRC_WORLDS to $PX4_WORLD_DST..."
    cp -r "$SRC_WORLDS/"* "$PX4_WORLD_DST"/
else
    echo "WARNING: Either source worlds folder ($SRC_WORLDS) or destination ($PX4_WORLD_DST) not found."
fi

# Copy models (everything except 'worlds')
if [[ -d "$SRC_MODELS" && -d "$PX4_MODEL_DST" ]]; then
    echo "Copying models from $SRC_MODELS to $PX4_MODEL_DST (excluding 'worlds')..."
    for item in "$SRC_MODELS"/*; do
        [[ -e "$item" ]] || continue
        base="$(basename "$item")"
        if [[ "$base" == "worlds" ]]; then
            continue
        fi
        cp -r "$item" "$PX4_MODEL_DST"/
    done
else
    echo "WARNING: Either source models folder ($SRC_MODELS) or destination ($PX4_MODEL_DST) not found."
fi

# Build px4 sitl
echo "Running make px4_sitl (may take some time)..."
make px4_sitl

# 6. Micro XRCE Agent
step $CURRENT_STEP "Installing Micro XRCE Agent"
CURRENT_STEP=$((CURRENT_STEP+1))
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git || true
cd Micro-XRCE-DDS-Agent
mkdir -p build && cd build
# TODO: Manually update CMakeLists.txt with FastDDS 3.2.2
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/

# 7. ROS2 Workspace
step $CURRENT_STEP "Creating ROS2 Workspace and Packages"
CURRENT_STEP=$((CURRENT_STEP+1))
cd ~
mkdir -p ws/src
cd ws/src
git clone https://github.com/PX4/px4_msgs.git || true
cd px4_msgs && git reset --hard f29c1d961e34489ef0e505c74467e980a0f0a988 && git submodule update --recursive
cd ..

git clone https://github.com/PX4/px4_ros_com.git || true
cd px4_ros_com && git reset --hard 5e46ae598447a5a3f59aa5ecbf3422dfe12261a6 && git submodule update --recursive
cd ~/ws

source /opt/ros/humble/setup.bash
colcon build

sudo rosdep init || true
rosdep update
export GZ_VERSION=garden
echo "export GZ_VERSION=garden" >> ~/.bashrc

cd ~/ws/src
git clone https://github.com/gazebosim/ros_gz.git || true
cd ros_gz && git reset --hard b5dffdeb9ce8d99fdb502f3b1c73e20325d04c3f

cd ~/ws/src
git clone https://github.com/SteveMacenski/slam_toolbox.git -b 2.6.8 || true
# Copy mapper_params_online_async.yaml into slam_toolbox/config
SRC_PARAM_FILE="$HOME/Reinforcement-Learning-for-Drone-Navigation-and-Pump-Localization/mapper_params_online_async.yaml"
DEST_CONFIG_DIR="$HOME/ws/src/slam_toolbox/config"

if [[ -f "$SRC_PARAM_FILE" && -d "$DEST_CONFIG_DIR" ]]; then
    echo "Copying mapper_params_online_async.yaml into slam_toolbox/config..."
    cp "$SRC_PARAM_FILE" "$DEST_CONFIG_DIR/"
else
    echo "WARNING: Could not copy mapper_params_online_async.yaml (source file or destination missing)."
fi

cd ~/ws
rosdep install -q -y -r --from-paths src --ignore-src --rosdistro humble

git clone https://github.com/ros-perception/pointcloud_to_laserscan.git || true
cd pointcloud_to_laserscan && git reset --hard 59bf996fb3ee7db0026a5cd3ce0d2a39d2e602ea

cd ~/ws
colcon build
source /opt/ros/humble/setup.bash

# Ask about sourcing local setup
read -p "Do you want to add workspace sourcing to ~/.bashrc? [y/N]: " add_ws
if [[ "$add_ws" =~ ^[Yy]$ ]]; then
    echo "source ~/ws/install/local_setup.bash" >> ~/.bashrc
fi
if [[ -f ~/ws/install/local_setup.bash ]]; then
    source ~/ws/install/local_setup.bash
else
    echo "Note: ~/ws/install/local_setup.bash not found yet (colcon build may have failed or not completed)."
fi


# 8. Build the gridHelperLib from our repo
step $CURRENT_STEP "Building gridHelperLib"
CURRENT_STEP=$((CURRENT_STEP+1))
GRID_HELPER_LIB="$HOME/Reinforcement-Learning-for-Drone-Navigation-and-Pump-Localization/gridHelperLib"
cd "$GRID_HELPER_LIB"
mkdir build && cd build
cmake .. && make

# Final note
echo ""
echo "===================================================="
echo "Installation finished (or reached end of script)."
echo "⚠️  Reminder: Install Uppaal manually from uppaal.org"
echo "Log file saved at: $LOGFILE"
echo "===================================================="
