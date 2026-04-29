# DAA Landing Repository

The Detect and Avoid (DAA) Landing repository goal is to provide an accessible platform to test and use DAA algorithms in a realistic simulation environment. This  repository is made to work specifically in Gazebo Sim Garden and ROS2 Humble, but there is a simpler version to be used in Gazebo Classic inside [DRAFT Simulation Environment](https://github.com/Darkjrcy/FAA_Recognition_Draft.git). 

**Key Features:**
- **Realistic Sensor:** Includes accurate positioning sensors, such as GNSS and ADS-B transmitters, to mimic real-world data collection.
- **Custom Flight Dynamics:** Features a flexible [`3d_movement_plugin`](https://github.com/Darkjrcy/DAA_Landing_Repository/tree/main/ros_ws/src/movement_plugin) plugin that allows users to easily create and apply custom dynamics models to their UAVs.
- **Advanced Simulation Controls:** Built-in helper tools let you fully customize the simulation environment. You can easily configure:
    - The total number of active UAVs.
    - UAV spawning methods and initial states.
    - The specific type of avoidance algorithms being tested.
