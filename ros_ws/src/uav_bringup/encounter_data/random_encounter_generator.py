import pandas as pd
import numpy as np
import itertools
import math
from pathlib import Path

# Define the data file where the terminal_encounter_info_20200630.csv is located if you don't have it go to: https://www.ll.mit.edu/r-d/datasets/unmanned-aircraft-terminal-area-encounters
MAIN_DIR = Path(__file__).resolve().parent
ENCOUNTER_DATA = MAIN_DIR / "terminal_encounter_info_20200630.csv"


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





# Open the csv file:
encounter_info = pd.read_csv(ENCOUNTER_DATA, skipinitialspace=True)
# Add a new column with teh distance between UAVs at the CPA:
encounter_info['tca_distance'] = np.sqrt(encounter_info['hmd_ft']**2 + encounter_info['vmd_ft']**2)
encounter_info_sorted = encounter_info.sort_values(by='tca_distance', ascending=True)

# Define the number of encouters you want to create:
num_tests = 10
# Define the cases how many UAVS are going to be there needs to be a multiple of 2:
num_uavs = [2, 4, 6, 8, 10]

# Start a forloop to generate each waypoints set:
for uav in num_uavs:
    # Get the number of base encounters needed
    used_encounter = encouenter_numb(num_tests, uav)
    
    # Grab the top closest trajectories from the sorted dataframe
    top_encounters = encounter_info_sorted.head(used_encounter).copy()
    
    # Map the real dataset IDs to letters (A, B, C...)
    # chr(65) is 'A', chr(66) is 'B', etc.
    top_encounters['letter_id'] = [chr(65 + i) for i in range(used_encounter)]
    letters = top_encounters['letter_id'].tolist()
    
    # Generate ALL possible combinations using itertools
    items_per_combo = int(uav / 2)
    all_combinations = list(itertools.combinations(letters, items_per_combo))
    
    # We only want 'num_tests' (e.g., exactly 10) of them
    selected_combinations = all_combinations[:num_tests]
    
    # Format them nicely as strings (e.g., 'AB', 'ABC')
    string_combinations = ["".join(combo) for combo in selected_combinations]
    
    # Print the results!
    print(f"--- {uav} UAVs ---")
    print(f"Base encounters needed: {used_encounter}")
    print(f"Dataset IDs used: {top_encounters['id'].tolist()}")
    print(f"Mapped to Letters: {letters}")
    print(f"Generated {len(string_combinations)} Sets: {string_combinations}\n")