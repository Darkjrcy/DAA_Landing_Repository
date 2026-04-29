import os
import re
import json
import pandas as pd
import random
import math

import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch.actions import IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch import LaunchDescription

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

# Get the world name form the sdf file:
def get_world_name(world):
    world_name = world.split(".")
    return world_name[0]


# Generate t arandom position to spawn the uavs and don't overlap:
def generate_random_position(n, min_dist, x_range = (-100, 100), y_range = (-50, 50), z_range= (40, 100)):
    positions = []
    attemps = 0
    max_iter = 1000

    while len(positions) < n and attemps < max_iter:
        new_x = random.uniform(x_range[0], x_range[1])
        new_y = random.uniform(y_range[0], y_range[1])
        new_z = random.uniform(z_range[0], z_range[1])

        # Check if there are not overlaping:
        not_overl = True
        for (ex_x, ex_y, ex_z) in positions:
            dist = math.sqrt((new_x - ex_x)**2 + (new_y - ex_y)**2 + (new_z - ex_z)**2)
            if dist < min_dist:
                not_overl = False
                break
        
        if not_overl:
            positions.append((new_x, new_y, new_z))
        attemps += 1
    
    while len(positions) < n:
        positions.append((0.0, 0.0, 70.0 + len(positions) * 10.0)) 
        
    return positions


def launch(context, *args, **kwargs):
    # Define the packages:
    # Package of the launch files and launch tools:
    pkg_bringup = "uav_bringup"
    # Deifne the isntallation directory:
    install_dir = get_package_share_directory(pkg_bringup)
    # DAA simulation logic and tools used for opening and closing the simulation
    pkg_daa_sim = "daa_exec"

    # Define a list with all the launch processes:
    launch_process = []

    # Launch Arguments for Spawning:
    # Type of the UAVs (fixed_wing, vtol):
    type_uav = ["fixed_wing", "fixed_wing"]
    # Launch airplane arguments:
    robot_names = ["airplane_1", "airplane_2"]
    # Create the spawning positions randomlly:
    spawn_coords = generate_random_position(len(robot_names), min_dist= 15.0)
    robot_scale = ["0.7", "0.7"]
    camera_num = ["1.0", "0.0"]
    # Launch arguments for the world:
    world = "MIT.sdf"
    world_name = get_world_name(world)
    # Fog characteristis:
    fog_lifetime="10.0"
    fog_rate="1.0"
    enable_fog="false"
    
    ######################## IMportant charcateristics for hte DAA simulation ######################################
    # # Define the trajectories the UAVs have to follow (N,E,D RF and x_velocity):
    # trajectories = [
    #     # UAV 1 (EX: 2 Trajectories):
    #     "-3000, 0, -280, 80; 5200, 0, -300, 85; 6400, 0, -310, 90; 7600, 0, -320, 90; 8800, 0, -325, 90; 9200, 0, -325, 90; 10000, 0, -325, 90; 11000, 0, -325, 90; 14000, 0, -325, 90; 16000, 0, -340, 90"
    #     " % "
    #     "4000, 0, -280, 80; 5200, 0, -300, 85; 6400, 0, -310, 90; 7600, 0, -320, 90; 8800, 0, -325, 90; 9200, 0, -325, 90; 10000, 0, -325, 90; 12000, 0, -325, 90; 14000, 0, -325, 90; 16000, 0, -340, 90",
    #     # UAV 2 (EX: 2 tRAJECTORIES)
    #     "7800, 0, -300, 80; 7600, 0, -300, 80; 6400, 0, -300, 80; 5200, 0, -300, 80; 4000, 0, -300, 80; 1000, 0, -300, 80; -800, 0, -300, 80"
    #     " % "
    #     "6150, 1230, -300, 50; 7739, 1230, -300, 50; 9476, 1230, -300, 50; 11178, 1230, -300, 50; 12616, 1230, -300, 50; 14000, 1230, -300, 50",
    #     # UAV 3 (EX: 2 tRAJECTORIES)
    #     "9000, 4800, -325, 80; 9000, -1284, -325, 80; 9000, -3443, -325, 80; 9000, -6000, -325, 80; 9000, -8608, -325, 80; 9000, -11000, -325, 80"
    #     " % "
    #     "10000, -4800, -300, 80; 9867, -3278, -290, 85; 9679, -834, -280, 85; 9415, 2595, -250, 90; 9241, 4855, -240, 90; 9000, 8000, -230, 90",
    # ]

    # MIT Defined trejcteries:
    trajectory_info = os.path.join(install_dir, "encounter_data/test_trajectories/case_2uavs.json")
    # OPen the json file:
    with open(trajectory_info, 'r') as file:
        data = json.load(file)
        trajectory_dict = data["trajectories"]
        roll_dict_init = data["roll_init"]
        pitch_dict_init = data["pitch_init"]
        head_dict_init = data["head_init"]
    
    num_uavs = len(robot_names)
    # Define the ttrajectories:
    trajectories = [trajectory_dict[f"uav_{i+1}"] for i in range(num_uavs)]
    # Create the intial attitude strings to send it to the MIT base system:
    roll = [roll_dict_init[f"roll_{i+1}"] for i in range(num_uavs)]
    pitch = [pitch_dict_init[f"pitch_{i+1}"] for i in range(num_uavs)]
    heading = [head_dict_init[f"head_{i+1}"] for i in range(num_uavs)]

    # Define which of the UAVs have aviodance:
    has_avoidance = [True, False]
    # The type of guidance DAA algorithm it uses ("GEOMETRIC", "FMT" ...) if doesn';t use put "NONE":
    type_avoidance = ["GEOMETRIC", "NONE"]


    ############# Important change the location where the AIrplane data is going to be saved#########################
    # Root:
    ws_root = os.path.abspath(os.path.join(install_dir, '../../../../'))
    # Deifne the main directory:
    MAIN_DIR = os.path.join(ws_root, 'src', pkg_bringup, 'saving_data')
    # Create a new fodler every time is launched:
    tests = [f for f in os.listdir(MAIN_DIR) if os.path.isdir(os.path.join(MAIN_DIR,f))]
    # Add a new folder:
    test_numbers = []
    for test in tests:
        match = re.match(r'Test_(\d+)', test)
        if match:
            test_numbers.append(int(match.group(1)))
    # Determine the next test folder number
    if test_numbers:
        next_folder_number = max(test_numbers) + 1
    else:
        next_folder_number = 1
    # Generate the folder of the test:
    new_folder_name = f'Test_{next_folder_number}'
    new_folder_path = os.path.join(MAIN_DIR, new_folder_name)


    # Open the world:
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(pkg_bringup),
                'launch',
                'start_world.launch.py'
            ])
        ]),
        launch_arguments = {
            'world': PathJoinSubstitution([
                FindPackageShare(pkg_bringup),
                'worlds',
                world
            ]),
            'camera': TextSubstitution(text='true')
        }.items()
    )
    launch_process.append(start_world)


    # Iterativelly spawn the aircrafts using a json file, Important this is not required but is done to start the simulation
    # The daa simualtion file has a spawner that is going to be changed depoending on the Trajectories:
    # Spawn the airplanes:
    for i in range(len(robot_names)):
        x_pos, y_pos, z_pos = spawn_coords[i]
        spawn_uav = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare(pkg_bringup),
                    'launch',
                    'uav_launcher.launch.py'
                ])
            ]),
            launch_arguments={
                'type_uav': type_uav[i],
                'robot_scale': robot_scale[i],
                'robot_name': robot_names[i],
                'camera_num': camera_num[i],
                'world_name': world_name,
                'fog_lifetime': fog_lifetime,
                'fog_rate': fog_rate,
                'enable_fog': enable_fog,
                'spawn_x':str(x_pos),
                'spawn_y':str(y_pos),
                'spawn_z':str(z_pos),
            }.items()
        )
        launch_process.append(spawn_uav)
    
    # Connect ros2 of the UAV with the Gazebo Sim:
    gz_world_topic_bridge = Node(
        package = 'ros_gz_bridge',
        executable ='parameter_bridge',
        arguments = [
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        remappings = [
        ],
        output = 'screen'
    )
    launch_process.append(gz_world_topic_bridge)

    # Open the Node that waits unitl gz is open:
    wait_for_gazebo_node = Node(
        package= pkg_daa_sim,
        executable='wait_for_gz',
        name='wait_for_gz',
        output='screen'
    )
    launch_process.append(wait_for_gazebo_node)

    # Create the daa main programm that launches all the uav dynamics, trajectories and saving data nodes:
    daa_sim = Node(
        package= pkg_daa_sim,
        executable='mit_simulation',
        name= 'mit_simulation',
        output = 'screen',
        parameters=[{
            'data_directory': new_folder_path,
            'world_name': world_name,
            'model_names': robot_names,
            'uav_type': type_uav,
            'model_waypoints': trajectories,
            'initial_roll' : roll,
            'initial_pitch': pitch,
            'initial_heading': heading,
            'has_avoidance': has_avoidance,
            'avoidance_types': type_avoidance,
        }]
    )

    # Launch the DAA simulation only after gz sim is stable:
    launch_daa_sim = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_gazebo_node,
            on_exit=[daa_sim]
        )
    )
    launch_process.append(launch_daa_sim)

    # Add the stop simulation node to stop evrything when the trejctories are completed:
    stop_sim = Node(
        package=pkg_daa_sim,
        executable='stop_simulation',
        name= 'stop_simulation',
        output = 'screen',
    )
    launch_process.append(stop_sim)




    # Return all the launch processes;
    return launch_process




# Create the laucnh description:
def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch)
    ])