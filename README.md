# DAA Landing Repository

The Detect and Avoid (DAA) Landing repository goal is to provide an accessible platform to test and use DAA algorithms in a realistic simulation environment. This  repository is made to work specifically in Gazebo Sim Garden and ROS2 Humble, but there is a simpler version to be used in Gazebo Classic inside [DRAFT Simulation Environment](https://github.com/Darkjrcy/FAA_Recognition_Draft.git). 

**Key Features:**
- **Realistic Sensor:** Includes accurate positioning sensors, such as GNSS and ADS-B transmitters, to mimic real-world data collection.
- **Custom Flight Dynamics:** Features a flexible [`3d_movement_plugin`](https://github.com/Darkjrcy/DAA_Landing_Repository/tree/main/ros_ws/src/movement_plugin) plugin that allows users to easily create and apply custom dynamics models to their UAVs.
- **Advanced Simulation Controls:** Built-in helper tools let you fully customize the simulation environment. You can easily configure:
    - The total number of active UAVs.
    - UAV spawning methods and initial states.
    - The specific type of avoidance algorithms being tested.

## Installation
### Clone Repository
This repository relies on submodules for the GNSS multipath plugin. You must clone it recursively to ensure all necessary files are downloaded properly:
``` bash
git clone --recursive https://github.com/Darkjrcy/DAA_Landing_Repository.git
```
### Prerequisites
The system requires the installation of Gazebo Garden and ROS 2 Humble, along with their respective developer packages.

**Install Gazebo Garden:** 
Run the following commands to install Gazebo Garden:
``` bash
sudo apt-get update
sudo apt-get install wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-garden
```

**Install the ROS 2 Gazebo Bridge:**
Because Gazebo Garden is not configured to work natively with ROS 2 Humble by default, you must install the ROS 2 Gazebo bridge tools:
``` bash
sudo apt install ros-humble-ros-gzgarden
```

**MATLAB Pre-compiled Functions:**
In order for the real-time Fast Marching Tree (FMT) to work correctly using the UAV Dubins paths, the simulation utilizes pre-compiled functions from MATLAB. To install these, please follow the specific setup process detailed inside the [matlab_obj](https://github.com/Darkjrcy/DAA_Landing_Repository/tree/main/ros_ws/src/uav_dynamics/src)  directory.

## Setup & Configuration
There are different ways to modify the simulation to fit your specific testing needs. The two primary files where you can customize the environment and simulation parameters are:
- **World Generator:** By default, Gazebo does not have a built-in tool to generate environments directly from real-world geographical positions. To solve this, this repository includes a custom [world generator function](https://github.com/Darkjrcy/DAA_Landing_Repository/tree/main/ros_ws/src/uav_bringup/worlds). This allows users to test their DAA scenarios in realistic environments with buildings, visual clutter, and accurate terrain. Additionally, the inclusion of 3D buildings enhances the realism of the GNSS plugin, which incorporates a multipath error logic based on line-of-sight signal obstructions.
- **Launch File ([`main_uav_launch`](https://github.com/Darkjrcy/DAA_Landing_Repository/tree/main/ros_ws/src/uav_bringup/launch)):** The launch file is the central hub where the user defines the core characteristics, parameters, and behaviors of the main simulation.

## Launch the Simulation:
Before using any of the packages, executables, or launch files, you must build the workspace. It is recommended to build the `libpredict` package first before building the rest of the repository.

Run the following commands:
``` bash
cd ros_ws
# Build the libpredict package first:
colcon build --packages-select libpredict
# Build the otehr packages:
colcon build
```

Next, source your ROS 2 installation and the simulation repository workspace:
``` bash 
# Source the ROS 2 and Gazebo installation (if you haven't already):
source /opt/ros/humble/setup.bash

# Source the DAA Repository:
source .../DAA_Landing_Repository/ros_ws/install/setup.bash
```

Once everything is built and sourced, you can start the simulation environment:
```bash
ros2 launch uav_bringup main_uav_fligth.launch.py 

```

**Data Logging:** 
After the simulation ends, the UAV encounter information is automatically saved in `.../uav_bringup/saving_data/Test_n`, where `n` corresponds to the sequence number of the test you are currently running.

