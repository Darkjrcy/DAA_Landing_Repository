#include "uav_dynamics/rt_fmt.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>


// Structure to save the DAA qualities of an obstacle:
struct FMTDetect {
    // Index of obstacle:
    std::string obstacle;
    // Position of the obstacle:
    Eigen::Vector3d obs_pos;
    // Minum avoidance radius:
    double dm;
    // Goal for the FMT and index in the waypoints
    Eigen::Vector3d fmt_goal;
    int goal_idx;
};



// Structre to save the active obstacles characteristics and number of intesece obstacles:
struct FMTBundle {
    // Add a number of FCA detection resume:
    std::vector<FMTDetect> act_obs;

    // Number of obstacles start with zero:
    double conflicts = 0;
};



// ENU -> NED conversion
inline static Eigen::Vector3d ENU_to_NED(const Eigen::Vector3d &enu) {
    return Eigen::Vector3d(enu.y(), enu.x(), -enu.z()); // (N,E,-U)
}



// Strcuture that contains all the information initial variables of the FMT:
struct RTFMTOptions {
    int N; // Number of points that need to be sampled
    double w; //weigthing fator to penalize heading terms
    double goal_radius; // How far the end point could be from the goal to do the trajecotry
    int expandTreeRate = 32; // How muich it search in the three
    double safeRadiusDObstacle; // AVoidance radius from the UAS to obstacle
};



// Struct to define the planner state:
struct  RTFMTPLannerState{
    // Parameters:
    std::vector<int> map;
    std::vector<double> limits;
    std::vector<double> start;
    std::vector<double> goal;
    double w;
    int N;
    int expandTreeRate;
    double rn;
    double goalRadius;
    double safeRadiusDObstacle;

    // Positions:
    std::vector<std::vector<double>> V;

    // Important indices:
    int startIdx;
    int goalIdx;
    int rootIdx;
    std::vector<int> goalRegionIdx;

    // Sets and Flags of the tree:
    std::vector<int> parent; // Parent nodes
    std::vector<double> cost; // Cost of the nodes
    std::vector<int> state; // 0=Undefined, 1=Open, 2=Unvisited, 3=Closed
    std::vector<int> W;     // Unvisited set indices
    std::vector<bool> isOpen; // NOdes that are open
    int z; // Current Node

    // RT Boolean lists (blockedd, openNew, PATHS, ETCX)
    std::vector<bool> blocked;
    std::vector<bool> openNew;
    std::vector<bool> closedToOpen;
    std::vector<bool> checkedPath;
    std::vector<int> checkedPathCandidates;
};
 



// Function to create the 



// Function to identify if an obstacle is interfering the actual Trajectory thresholded by a range limit:
FMTBundle FMT_Detect(const uav_dynamics::msg::AvoidanceStates &moving_obstacles, double own_e, double own_n, double own_u,double own_ve, double own_vn, double own_vu,
    double crit_time, NavigationState& nav_state, double min_radius){
    // Add a structure to save all teh conflicting obstacles:
    FMTBundle fmt_bundle;
    // Own velocity norm:
    const double own_vel_a = std::sqrt(own_vn*own_vn+own_ve*own_ve+own_vu*own_vu);
    // Ownship position:
    const Eigen::Vector3d own_pos(own_n,own_e,-own_u);

    // Loop betwen all the moving obstacles to see if they intersect with the waypoints:
    for (size_t j = 0; i<n; i++){
        // Get the posstion of the obstacles:
        const auto &obs = moving_obstacles.states[j];
        // Define the obstacle position:
        const Eigen::Vector3d obs_pos(obs.north,obs.east,-obs.up);
        // Get the relative position:
        const Eigen::Vector3d rel_pos = obs_pos - own_pos;
        // RElative position norm:
        double r_rel = rel_pos.norm();
        // Define the avoidance radius:
        const double obs_velocity_a = std::sqrt(obs.v_north*obs.v_north + obs.v_east*obs.v_east + obs.v_up*obs.v_up);
        const double dm = 1.5*(obs_velocity_a*1.0+125+1.5*6+1.0*own_vel_a);
        // Define the obstacle id:
        const std::string &id = moving_obstacles.obstacles_id[i];

        // Define the current idx of the Waypoints:
        const size_t current_idx =  nav_state.current_idx;
        // Save the searching obstacle distance:
        const double search_radius = 1.25 * (dm + min_radius + crit_time * own_vel_a);

        // Chek if the obstacle is even near the ownship to try to analize it:
        if (r_rel > search_radius * 1.5) continue;

        // Obtian the waypoint that is near the search distance from the ownship:
        if  (current_idx + 1 < nav_state.waypoints.size() - 1) {
            for (size_t wp_idx = current_idx + 1; wp_idx < nav_state.waypoints.size() - 1; ++wp_idx) {
                const double d_own = (nav_state.waypoints[wp_idx] - own_pos).norm();
                // If the waypoint is within the lookahead search distance
                if (d_own < 1.25 * search_radius) {
                    Eigen::Vector3d dW = nav_state.waypoints[wp_idx + 1] - nav_state.waypoints[wp_idx];
                    double ang_dW_rel = std::acos(std::clamp(rel_pos.dot(dW) / (r_rel * dW.norm()), -1.0, 1.0));
                    if (r_rel * std::sin(ang_dW_rel) <= dm) {
                        // Save the infermotation from the active obstacle
                        ++fmt_bundle.conflicts;
                        FMTDetect det;
                        det.obstacle = id;
                        det.obs_pos  = obs_pos;
                        det.dm = dm;

                        // Now search the goal position from the waypoints used in the RFT:
                        bool found_safe_goal = false;
                        for (size_t k = wp_idx + 1; k < nav_state.waypoints.size(); ++k) {
                            double dist_to_obs = (nav_state.waypoints[k] - obs_pos).norm();
                            if (dist_to_obs >= dm) {
                                det.fmt_goal = nav_state.waypoints[k];
                                det.goal_idx = k;
                                found_safe_goal = true;
                                break; 
                            }
                        }

                        // If doesn't find a agoal use the last weaypoint as a goal:
                        if (!found_safe_goal) {
                            det.goal_idx = nav_state.waypoints.size() - 1;
                            det.fmt_goal = nav_state.waypoints[det.goal_idx];
                        }

                        // Piut the obstacle info int he bundele:
                        fmt_bundle.act_obs.push_back(det);
                        break;


                    }
                }
            } 
        }
    }

    // Return teh active obstacles:
    return fmt_bundle;
}



// Start by defining a searching function of obstacles:
void  computerFMTAvoidance(const uav_dynamics::msg::AvoidanceStates& moving_obstacles, const gnss_multipath_plugin::msg::AdsbInfo& own_state,
    double min_radius, double crit_time, NavigationState& nav_state){
    // Get the ownship data:
    const double own_e  = own_state.east;
    const double own_n  = own_state.north;
    const double own_u  = own_state.up;
    const double own_ve = own_state.v_east;
    const double own_vn = own_state.v_north;
    const double own_vu = own_state.v_up;
    const double own_course = own_state.course;
    const double own_fpa = own_state.fpa;

    // Identify it there is an obstacle neart the next waypoints:
    FMTBundle active_obs = FMT_Detect(moving_obstacles, own_e, own_n, own_u, own_ve, own_vn, own_vu, crit_time, nav_state, min_radius);

    // If there nor conflict exit:
    if (active_obs.conflicts == 0){
        return;
    }
        
    
}