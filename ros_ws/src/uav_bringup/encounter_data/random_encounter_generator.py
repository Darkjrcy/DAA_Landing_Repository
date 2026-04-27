import pandas as pd
import numpy as np
import itertools
import math
import os
import json
import matplotlib.pyplot as plt
from pathlib import Path

# Define the data file where the terminal_encounter_info_20200630.csv is located if you don't have it go to: https://www.ll.mit.edu/r-d/datasets/unmanned-aircraft-terminal-area-encounters
MAIN_DIR = Path(__file__).resolve().parent
ENCOUNTER_DATA = MAIN_DIR / "terminal_encounter_info_20200630.csv"
# Define the direction where hte trajectories are defined:
TRAJECTORY_FOLDER = MAIN_DIR / "terminal_encounter_state_data_20200630"
# FOlder witht eh json files of the trajectories in teh format required byt he simulation:
JSON_FOLDER = MAIN_DIR / "test_trajectories"

# Find how many encoeunters are required from teh lsit to create n eocuenter sets with x UAVS:
def encouenter_numb(n, x):
    # Number of encounters needed per sim (must be an integer for factorials)
    num_encount = int(x / 2)
    
    # If we only need 1
    if num_encount == 1:
        return n
        
    # Get how many base encounters you need to generate 'n' combinations:
    i = num_encount
    while True:
        count = math.factorial(i) // (math.factorial(num_encount) * math.factorial(i - num_encount))
        if count >= n:
            return i
        i += 1



# Get the intruder and ownship trajecotires from  a encoutner:
def get_trajectories_from_enc(id):
    # Define the folder name:
    folder_num = math.floor((id-1)/1000)
    folder_name = f"encounters_{folder_num:03d}001"

    # Get the intruder and ownship directiones:
    in_csv_name = f"intstates_{id:06d}.csv"
    in_encounter_path = os.path.join(TRAJECTORY_FOLDER, folder_name, in_csv_name)
    own_csv_name = f"ownstates_{id:06d}.csv"
    own_encounter_path = os.path.join(TRAJECTORY_FOLDER, folder_name, own_csv_name)

    return (in_encounter_path, own_encounter_path)



# FUnction to format the trajectries:
def format_trajectory(traj_array):
    # Format each row as "N, E, D, V" (rounded to 0 decimal places)
    string_rows = [f"{row[0]:.0f}, {row[1]:.0f}, {row[2]:.0f}, {row[3]:.0f}" for row in traj_array]
    # Join the rows with "; "
    return "; ".join(string_rows)





# Open the csv file:
encounter_info = pd.read_csv(ENCOUNTER_DATA, skipinitialspace=True)
# Add a new column with teh distance between UAVs at the CPA:
encounter_info['tca_distance'] = np.sqrt(encounter_info['hmd_ft']**2 + encounter_info['vmd_ft']**2)
encounter_info_sorted = encounter_info.sort_values(by='tca_distance', ascending=True)

# Define the number of encouters you want to create:
num_tests = 10
# Define the cases how many UAVS are going to be there needs to be a multiple of 2:
num_uavs = [2, 4]

# Option to plot trajectories:
plot_enable = False

# Start a forloop to generate each waypoints set:
for uav in num_uavs:
    # Get the number of base encounters needed
    used_encounter = encouenter_numb(num_tests, uav)
    # Grab the top closest trajectories from the sorted dataframe
    top_encounters = encounter_info_sorted.head(used_encounter).copy()
    # Deifne the IDs of each encoutner:
    top_encounters['encounter_id'] = [chr(65 + i) for i in range(used_encounter)]
    encounter_ids = top_encounters['id'].tolist()
    # Generate ALL possible combinations using itertools
    items_per_combo = int(uav / 2)
    all_combinations = list(itertools.combinations(encounter_ids, items_per_combo))
    selected_combinations = [list(combo) for combo in all_combinations[:num_tests]]

    # Obtain the waypoints from the folder:
    trajectories_lists = {f"uav_{k+1}": [] for k in range(uav)}
    case_idx = 0

    for case in selected_combinations:
        case_idx += 1
        
        # Start the plot:
        if plot_enable:
            fig = plt.figure(figsize=(10, 8))
            ax = fig.add_subplot(111, projection='3d')
            ax.set_title(f"Simulation Case {case_idx} ({uav} UAVs)\nEncounters: {case}")

        # CReate a list to temporarlly save the trajectoriesL:
        case_arrrays = []
        for enc_id in case:
            in_encounter_path, own_encounter_path = get_trajectories_from_enc(enc_id)
            # Open both csv files:
            df_intruder = pd.read_csv(in_encounter_path)
            df_ownship = pd.read_csv(own_encounter_path)

            # Get the trajectories:
            in_traj_array =  np.column_stack([
                df_intruder[' y_ft'].values,
                df_intruder[' x_ft'].values,
                df_intruder[' alt_ft'].values * -1,
                df_intruder[' speed_ftps'].values,
            ])  

            own_traj_array =  np.column_stack([
                df_ownship[' y_ft'].values,
                df_ownship[' x_ft'].values,
                df_ownship[' alt_ft'].values * -1,
                df_ownship[' speed_ftps'].values,
            ])  

            # Temporarlly save the array:
            case_arrrays.append((enc_id, own_traj_array, in_traj_array))
        
        # Calculate the mean from teh first trahjectrory
        first_own_traj = case_arrrays[0][1]
        avg_N = np.mean(first_own_traj[:, 0])
        avg_E = np.mean(first_own_traj[:, 1])

        # Get the trajectorie:
        uav_idx = 0
        for enc_id, in_traj_array, own_traj_array in case_arrrays:
            # Take out the mean values:
            in_traj_array[:, 0] -= avg_N
            in_traj_array[:, 1] -= avg_E
            in_traj_array[:, 2] -= 150
            in_traj_array[:, 3] = in_traj_array[:, 3]*0.75
            
            own_traj_array[:, 0] -= avg_N
            own_traj_array[:, 1] -= avg_E
            own_traj_array[:, 2] -= 150
            own_traj_array[:, 3] = own_traj_array[:, 3]*0.75
            # Format awways into the requested strings:
            trajectories_lists[f"uav_{uav_idx+1}"].append(format_trajectory(own_traj_array))
            trajectories_lists[f"uav_{uav_idx+2}"].append(format_trajectory(in_traj_array))

            # Update the number of the UAV:
            uav_idx += 2

            # PLot them: 
            if plot_enable:
                ax.plot(own_traj_array[:, 1], own_traj_array[:, 0], own_traj_array[:, 2], 
                        label=f'UAV {uav_idx-1} (Ownship, Enc {enc_id})', linestyle='-')
                ax.plot(in_traj_array[:, 1], in_traj_array[:, 0], in_traj_array[:, 2], 
                        label=f'UAV {uav_idx} (Intruder, Enc {enc_id})', linestyle='--')
            
        # Dsiplay teh overall plot:
        if plot_enable:
            ax.set_xlabel('East (x_ft)')
            ax.set_ylabel('North (y_ft)')
            ax.set_zlabel('Altitude (alt_ft)')
            ax.legend()
            plt.show()

    # Create the iveral string:
    final_trajectories = {}
    for k in range(uav):
        final_trajectories[f"uav_{k+1}"] = " % ".join(trajectories_lists[f"uav_{k+1}"])

    # JOin the trajectories ina  onys tring:
    file_name = f"case_{uav}uavs.json"
    file_path = JSON_FOLDER / file_name
    
    # Save them in json file:
    with open(file_path, 'w') as json_file:
        json.dump(final_trajectories, json_file, indent=4)
    

