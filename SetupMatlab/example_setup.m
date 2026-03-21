clc;close all;clear all;
%% Example of the MATLAB setup used for the GUAM co-simulation system:
% Source where the packages are installed:
src_path = "/home/jorge/DAA_Landing_Repository/ros_ws/src";

% Spwaning characteristics of the vtol
east = 100; north = 100; alt = 50; heading = pi/4; fpa = 20/57.3; roll = 10/57.3;
% Define the number of the vtol if you are spawning more than one:
vtol_num = 1; 

% Run teh setup file:
ros2setup(src_path, east, north, alt, heading, fpa, roll, vtol_num)