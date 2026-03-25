#ifndef RT_FMT_HPP
#define RT_FMT_HPP

// Standard libraries needed for the definitio
#include <vector>
#include <string>
#include <optional>
#include <Eigen/Dense>

// ROS 2 Messages needed for the inputs
#include "uav_dynamics/msg/avoidance_states.hpp"
#include "gnss_multipath_plugin/msg/states_info.hpp"
// Include the UAV dubins connector:
#include "uav_dynamics/uav_dubins_paths.hpp"



// Strcuture that contains all the information initial variables of the FMT:
struct RTFMTOptions {
    int N = -1; // Number of points that need to be sampled
    double w1; //weigthing fator to penalize heading terms
    double w2; //weigthing fator to penalize fpa terms
    double goal_radius; // How far the end point could be from the goal to do the trajecotry
    int expandTreeRate = 32; // How muich it search in the three
    double safeRadiusDObstacle; // AVoidance radius from the UAS to obstacle
};



// Struct to define the planner state:
struct  RTFMTPLannerState{
    // Parameters:
    std::vector<int> map;
    std::vector<double> limits; // Limits shoudl be in NED
    Eigen::Vector4d start;
    Eigen::Vector4d goal;
    double w1;
    double w2;
    int N;
    int expandTreeRate;
    double rn;
    double goalRadius;
    double safeRadiusDObstacle;

    // Positions [N, E, D, heading]:
    std::vector<Eigen::Vector4d> V;

    // Important indices:
    int startIdx;
    int goalIdx;
    int rootIdx;
    std::vector<int> goalRegionIdx;

    // Sets and Flags of the tree:
    std::vector<int> parent; // Parent nodes
    std::vector<double> cost; // Cost of the nodes
    std::vector<int> state; // 0=Undefined, 1=Open, 2=Unvisited, 3=Closed
    std::vector<int> unvis;     // Unvisited set indices
    std::vector<bool> isOpen; // NOdes that are open
    int z; // Current Node

    // RT Boolean lists (blockedd, openNew, PATHS, ETCX)
    std::vector<bool> blocked;
    std::vector<bool> openNew;
    std::vector<bool> closedToOpen;
    std::vector<bool> checkedPath;
    std::vector<int> checkedPathCandidates;

    // Dynamic Obstacle Masks
    std::vector<bool> dynamicObstructed;
    std::vector<bool> dynamicObstructedPrev;

    // Tree characteristics and Neighbours;
    int lastRootIdx = -1;
    std::vector<bool> rewireRootSeen;
    std::deque<int> rewireRootList;
    std::pair<std::vector<int>, std::vector<Eigen::VectorXd>> Nz; // Neighbors of z
    std::vector<std::pair<int, int>> E;

    // Unvisited cnadidate nodes of the trree:
    std::vector<int> XNear;
};



// Structure that contains all the varibale required to change inside the Geomtric DAA
struct FMTNavigationState {
    // Waypoints in the [N,E,D, Heading]
    std::vector<Eigen::Vector4d>& waypoints;
    std::vector<double>& cmd_vel;
    size_t& current_idx;
    double& transition_radius;
    double& look_ahead_distance;
    bool& start_the_avoidance;
    std::optional<Eigen::Vector3d>& avoidance_last_point_enu;
    std::optional<size_t>& end_of_arc;
};



// Structure to save all the FMT planner charactersitics:
struct FMTPlanner {
    // Dubbins connector:
    FMTDubinsConnector dubins_connector;
    // Structure to define the RT FMT nodes and characteristics:
    RTFMTPLannerState ft_fmt_mask;
    // OPtions of the RT FMT:
    RTFMTOptions rt_fmt_opts;
};



// Function to start the rt_ftm planner and create the mask witht he dubins path:
void start_rt_fmt( const std::vector<int>& map, const std::vector<std::vector<double>>& limits, const std::vector<double>& start, 
    const std::vector<double>& goal, double rn, FMTPlanner& rt_fmt_planner, double max_roll, double air_speed, double fpa_limits[2]);

#endif