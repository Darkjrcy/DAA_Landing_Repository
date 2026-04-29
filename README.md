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
