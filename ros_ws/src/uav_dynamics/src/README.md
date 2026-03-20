# UAV Fixed-Wing Dynamics: MATLAB Object Files

If you have just cloned this repository, you might notice that the `ros_ws/src/uav_dynamics/src/matlab_obj/` folder is empty. 

To keep the repository lightweight and prevent cross-platform compilation errors, we do not track compiled binary (`.o`) files in Git. You will need to generate these object files locally using MATLAB's UAV Fixed-Wing library before building the ROS 2 workspace.

## Prerequisites
* MATLAB 
* MATLAB UAV Toolbox
* MATLAB Coder (for generating C/C++ code and object files)

##\ How to Generate the `.o` Files

1. Open MATLAB and navigate to the directory containing your UAV dynamics/Dubins path MATLAB scripts. Create a fucntion that uses the `uavDubinsConnection()` for example:
``` matlab 
function [lengths, pathSeg] = uav_dubins_paths(state1, state2, roll_max, Vair, fpa_lim)
%#codegen

% REMOVE extrinsic calls to allow standalone C++ generation
% coder.extrinsic('uavDubinsConnection', 'connect'); 

% 1. Initialize the connection object
conn = uavDubinsConnection( ...
    'MaxRollAngle',         roll_max, ...
    'AirSpeed',             Vair,     ...
    'FlightPathAngleLimit', fpa_lim);

% 2. Connect the states (returns a 1x1 cell array containing the segment)
pathSegCell = connect(conn, state1, state2);

% 3. Extract the object from the cell for the output
pathSeg = pathSegCell{1};

% 4. Extract lengths
lengths = pathSeg.MotionLengths;
end
```
2. Now generate the binary MATLAB functions using `codegen`, in case you use the function from above you must use:
``` matlab 
% Define example inputs
% UAV Dubins states are typically [x, y, z, heading]
state_ex = zeros(1, 4);
val_ex   = 0.0;
% FlightPathAngleLimit is usually a 1x2: [min, max]
fpa_ex   = [-0.1, 0.1]; 

% Create configuration
cfg = coder.config('lib', 'ecoder', false);
cfg.FilePartitionMethod = 'SingleFile';
cfg.TargetLang = 'C++'; 

% Generate code
% This will produce 'uav_dubins_paths.cpp' and 'uav_dubins_paths.h'
codegen -config cfg uav_dubins_paths ...
    -args {state_ex, state_ex, val_ex, val_ex, fpa_ex} ...
    -report
```
5. Copy the next files inside the matlab_obj folder:
* autonomouscodegen_dubins.o
* uavdubinscodegen_connection_api.o
* uavdubinscodegen_connection.o
