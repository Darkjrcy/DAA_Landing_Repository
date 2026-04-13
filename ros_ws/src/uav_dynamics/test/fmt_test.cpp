#include "uav_dynamics/rt_fmt.hpp"
#include <iostream>
#include <vector>
#include <fstream> 
#include <limits>
#include <algorithm>
#include <Eigen/Dense>

int main() {
    std::cout << "--- Starting RT-FMT Test ---" << std::endl;

    // ENvironment limits
    std::vector<std::vector<double>> limits = {{0.0, 100.0}, {0.0, 100.0}, {-50.0, 0.0}};
    std::vector<int> map; 

    // Start and goal
    std::vector<double> start = {10.0, 10.0, -20.0, 0.0};
    std::vector<double> goal = {90.0, 90.0, -20.0, 0.0};
    Eigen::Vector3d goal_pt(90.0, 90.0, -20.0);

    // RT-FMT planner parameters:
    FMTPlanner rt_fmt_planner;
    rt_fmt_planner.rt_fmt_opts.N = 1000;              
    
    rt_fmt_planner.rt_fmt_opts.w1 = 2.0;  
    rt_fmt_planner.rt_fmt_opts.w2 = 1.0;
    rt_fmt_planner.rt_fmt_opts.expandTreeRate = 20.0; 
    rt_fmt_planner.rt_fmt_opts.goal_radius = 5.0; 
    rt_fmt_planner.rt_fmt_opts.safeRadiusDObstacle = 5.0;

    // SImple UAV parameters used for the dubins paths:
    double rn = 10.0; 
    double max_roll = 0.5;
    double air_speed = 15.0;
    double fpa_limits[2] = {-0.2, 0.2};

    // Initialize teh rt fmt planner structure
    start_rt_fmt(map, limits, start, goal, rn, rt_fmt_planner, max_roll, air_speed, fpa_limits);
    
    // Setup the CSV files uses to plot in matlab
    std::ofstream path_file("fmt_path_results.csv");
    path_file << "North,East,Down,Yaw\n";
    std::ofstream edges_file("fmt_tree_edges.csv");
    edges_file << "Tick,ParentID,ChildID\n";
    std::ofstream nodes_file("fmt_tree_nodes.csv");
    nodes_file << "NodeID,North,East,Down,Yaw\n";

    // Simple simulation loop:
    Eigen::VectorXd current_pose(4);
    current_pose << 10.0, 10.0, -20.0, 0.0;
    FMTBundle act_obs; 
    act_obs.conflicts = 0;
    int tick_count = 0;
    double dist_to_goal = (current_pose.head(3) - goal_pt).norm();

    // Use current_state outside the loop so we can access it at the very end to save the nodes
    RTFMTPLannerState current_state;
    while (dist_to_goal > 5.0 && tick_count < 1000) {
        path_file << current_pose[0] << "," << current_pose[1] << "," << current_pose[2] << "," << current_pose[3] << "\n";
        // Tick the planner
        current_state = tick(rt_fmt_planner, act_obs, current_pose);

        // Save the edges:
        for (const auto& edge : current_state.E) {
            edges_file << tick_count << "," << edge.first << "," << edge.second << "\n";
        }

        // Generate the path
        PathResult current_path = generatePath(current_state);

        // Move the drone along the path
        if (current_path.path_found && current_path.waypoints.size() >= 2) {
            
            double min_d = std::numeric_limits<double>::infinity();
            int pathIdx = 0;
            for (size_t k = 0; k < current_path.waypoints.size(); ++k) {
                double d = (current_path.waypoints[k].head(3) - current_pose.head(3)).norm();
                if (d < min_d) {
                    min_d = d;
                    pathIdx = k;
                }
            }

            // Define the taget position that you want to go from the current path:
            int targetIdx = pathIdx + 1;
            if (targetIdx >= static_cast<int>(current_path.waypoints.size())) {
                targetIdx = current_path.waypoints.size() - 1; 
            }
            Eigen::Vector4d target = current_path.waypoints[targetIdx];
            Eigen::Vector3d diff = target.head(3) - current_pose.head(3);
            double dist_to_wp = diff.norm();

            // Get the heading assuming is moving like a reeal UAV:
            if (dist_to_wp > 0.5) { 
                Eigen::Vector3d dir = diff.normalized();
                double step = std::min(2.0, dist_to_wp); 
                current_pose.head(3) += dir * step; 
                
                current_pose[3] = std::atan2(dir.y(), dir.x()); 
                if (current_pose[3] < 0) current_pose[3] += 2.0 * M_PI;
            }
        }

        // Get the distance to the goal:
        dist_to_goal = (current_pose.head(3) - goal_pt).norm();
        tick_count++;
        
        if (tick_count % 100 == 0){
            std::cout << "Goal distance " << dist_to_goal << " m." << std::endl;
        }

        if (dist_to_goal <= 5.0) { 
            std::cout << "Goal reached in " << tick_count << " iterations." << std::endl;
            break;
        }
    }

    // Save the path that the UAV followed
    for (size_t i = 0; i < current_state.V.size(); ++i) {
        nodes_file << i << "," 
                   << current_state.V[i][0] << "," 
                   << current_state.V[i][1] << "," 
                   << current_state.V[i][2] << "," 
                   << current_state.V[i][3] << "\n";
    }

    path_file.close();
    edges_file.close();
    nodes_file.close();
    return 0;
}