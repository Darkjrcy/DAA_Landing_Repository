#ifndef GEOMETRIC_GUIDANCE_HPP
#define GEOMETRIC_GUIDANCE_HPP

// Standard libraries needed for the definitions
#include <vector>
#include <string>
#include <optional>
#include <Eigen/Dense>

// ROS 2 Messages needed for the inputs
#include "uav_dynamics/msg/avoidance_states.hpp"
#include "gnss_multipath_plugin/msg/adsb_info.hpp"

// Structure that contains all the varibale required to change inside the Geomtric DAA
struct NavigationState {
    std::vector<Eigen::Vector3d>& waypoints;
    std::vector<double>& cmd_vel;
    size_t& current_idx;
    double& transition_radius;
    double& look_ahead_distance;
    bool& start_the_avoidance;
    std::optional<Eigen::Vector3d>& avoidance_last_point_enu;
    std::optional<size_t>& end_of_arc;
};

// Declare the main geometric DAA fucntion
void computeGeometricAvoidance(const uav_dynamics::msg::AvoidanceStates& obstacles, const gnss_multipath_plugin::msg::AdsbInfo& own_state, 
    double min_radius,  double crit_time, NavigationState& nav_state);

#endif // GEOMETRIC_GUIDANCE_HPP