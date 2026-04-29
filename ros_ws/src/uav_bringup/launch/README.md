# Simulation Launch File:
### Launch File Configuration (`main_uav_launch.py`)

The launch file is the central control hub of the DAA simulation. By modifying the variables inside the `launch` function, you can fully customize the environment, the number of aircraft, their trajectories, and the specific avoidance algorithms being tested. The parameters the user can modify are:

> **Important:** All list variables (like `robot_names`, `type_uav`, `has_avoidance`, etc.) must have the exact same number of elements. If you are simulating 3 UAVs, every list must contain 3 items!

#### UAV Characteristics
- **`robot_names`**: A list of string identifiers for each model in the simulation.
    - Example:   `["airplane_1", "airplane_2"]`
- **`type_uav`**: Defines the physical and dynamical model of each aircraft.
    - Example: `["fixed_wing", "fixed_wing"]` (Other options include `"vtol"` is in development).
- **`robot_scale`**: Adjusts the visual size of the spawned models in Gazebo.
    - Example:  `["0.7", "0.7"]`
- **`camera_num`**: Enables or disables the onboard camera for each UAV.
    - Example:  `["1.0", "0.0"]` (1.0 = ON, 0.0 = OFF).

#### DAA Algorithm & Trajectory Setup
- **`has_avoidance`**: A boolean list defining whether a specific UAV is actively running a Detect and Avoid algorithm, or if it is acting as a blind intruder.
    - Example: `[True, False]` (UAV 1 is the ownship; UAV 2 represents the intruder).
- **`type_avoidance`**: Specifies which guidance algorithm to use for the avoidance maneuver.
    - Example: `["FMT", "NONE"]` (Supported types include `"FMT"`, `"GEOMETRIC"`, or `"NONE"`).
- **`trajectories`**: Defines the path each UAV will follow. By default, it a string structure to be send to the DAA simulation executable. The trajectories are in the NED reference frame with the airspeed velocity represented as: `"N0,E0,D0,Va0;N1,E1,D1,Va1;...;Ngoal,Egoal,Dgoal,Vagoal`;. In case multiple trajectories are simulated at the same time divide them with "%". 
	- Example: `2 traejctories for the two UAVs = [ #UAV 1 (EX: 2 Trajectories): "-3000, 0, -280, 80; 5200, 0, -300, 85; 6400, 0, -310, 90; 7600, 0, -320, 90; 8800, 0, -325, 90; 9200, 0, -325, 90; 10000, 0, -325, 90; 11000, 0, -325, 90; 14000, 0, -325, 90; 16000, 0, -340, 90 %  "4000, 0, -280, 80; 5200, 0, -300, 85; 6400, 0, -310, 90; 7600, 0, -320, 90; 8800, 0, -325, 90; 9200, 0, -325, 90; 10000, 0, -325, 90; 12000, 0, -325, 90; 14000, 0, -325, 90; 16000, 0, -340, 90",  # UAV 2 (EX: 2 tRAJECTORIES): "7800, 0, -300, 80; 7600, 0, -300, 80; 6400, 0, -300, 80; 5200, 0, -300, 80; 4000, 0, -300, 80; 1000, 0, -300, 80; -800, 0, -300, 80 % 6150, 1230, -300, 50; 7739, 1230, -300, 50; 9476, 1230, -300, 50; 11178, 1230, -300, 50; 12616, 1230, -300, 50; 14000, 1230, -300, 50]`

#### Environment & Spawning

- **`world`**: Defines the Gazebo environment file to load, if you don't have one, create one using the world_generator.
    - _Example:_ `"MIT.sdf"`
- **Weather Constraints (Fog):** You can simulate visual clutter or poor weather conditions by modifying the sdf scene. However, that is only available in Gazebo Classic version at the moment. 

#### Data Logging

- **`ws_root` & `MAIN_DIR`**: The script automatically tracks and saves the flight data, encounter metrics, and avoidance performance. Every time you run the launch file, it dynamically creates a new folder (e.g., `Test_1`, `Test_2`) inside `.../uav_bringup/saving_data/` so your previous simulation results are never overwritten.
