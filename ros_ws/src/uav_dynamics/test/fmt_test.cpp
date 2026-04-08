#include "uav_dynamics/rt_fmt.hpp"
#include <iostream>
#include <vector>
#include <fstream> 
#include <limits>
#include <algorithm>
#include <Eigen/Dense>

int main() {
    std::cout << "--- Starting RT-FMT Test ---" << std::endl;

    // 1. Define environment and limits
    std::vector<std::vector<double>> limits = {{0.0, 100.0}, {0.0, 100.0}, {-50.0, 0.0}};
    std::vector<int> map; 

    // 2. Define Start and Goal
    std::vector<double> start = {10.0, 10.0, -20.0, 0.0};
    std::vector<double> goal = {90.0, 90.0, -20.0, 0.0};
    Eigen::Vector3d goal_pt(90.0, 90.0, -20.0);

    // 3. Planner Parameters
    FMTPlanner rt_fmt_planner;
    rt_fmt_planner.rt_fmt_opts.N = 1000;              
    
    // REDUCED w1: High heading penalties completely break the 'near' search 
    rt_fmt_planner.rt_fmt_opts.w1 = 5.0;  
    rt_fmt_planner.rt_fmt_opts.w2 = 1.0;
    rt_fmt_planner.rt_fmt_opts.expandTreeRate = 30.0; 
    
    // INCREASED goal_radius: We want to accept any node within 20m of the goal
    rt_fmt_planner.rt_fmt_opts.goal_radius = 20.0; 
    rt_fmt_planner.rt_fmt_opts.safeRadiusDObstacle = 5.0;

    // --- CRITICAL KINEMATIC FIX ---
    // A UAV with AirSpeed=15 and MaxRoll=0.5 has a MinTurningRadius of ~42m.
    // 'rn' must be large enough to allow these massive sweeping turns!
    double rn = 100.0; 
    
    double max_roll = 0.5;
    double air_speed = 15.0;
    double fpa_limits[2] = {-0.2, 0.2};

    // 4. Initialize
    start_rt_fmt(map, limits, start, goal, rn, rt_fmt_planner, max_roll, air_speed, fpa_limits);
    
    // 5. Setup CSV Logging for Graphing
    std::ofstream data_file("fmt_path_results.csv");
    data_file << "North,East,Down,Yaw\n";

    // 6. Simulation Loop
    Eigen::VectorXd current_pose(4);
    current_pose << 10.0, 10.0, -20.0, 0.0;
    
    FMTBundle act_obs; 
    act_obs.conflicts = 0;

    int tick_count = 0;
    double dist_to_goal = (current_pose.head(3) - goal_pt).norm();

    std::cout << "Flying to goal..." << std::endl;

    while (dist_to_goal > 5.0 && tick_count < 1000) {
        data_file << current_pose[0] << "," << current_pose[1] << "," << current_pose[2] << "," << current_pose[3] << "\n";

        RTFMTPLannerState current_state = tick(rt_fmt_planner, act_obs, current_pose);

        int next_wp = -1;

        auto getNextWp = [&](int target_node) {
            if (target_node == current_state.rootIdx || target_node == -1) return -1;
            int curr = target_node;
            while (curr != -1 && current_state.parent[curr] != -1 && current_state.parent[curr] != current_state.rootIdx) {
                curr = current_state.parent[curr];
            }
            if (curr != -1 && current_state.parent[curr] == current_state.rootIdx) {
                return curr; 
            }
            return -1; 
        };

        // --- NEW GOAL REGION LOGIC ---
        // Instead of demanding the exact goalIdx, check the entire Goal Region array
        for (int g_idx : current_state.goalRegionIdx) {
            if (current_state.parent[g_idx] != -1) {
                int test_wp = getNextWp(g_idx);
                if (test_wp != -1) {
                    next_wp = test_wp; // We found a valid path to the goal area!
                    break; 
                }
            }
        }
        
        // 2. If the goal region hasn't been connected yet, head toward the closest explored node
        if (next_wp == -1) {
            double min_d = std::numeric_limits<double>::infinity();
            int best_node = -1;
            for (size_t i = 0; i < current_state.V.size(); ++i) {
                if (current_state.parent[i] != -1) {
                    int test_wp = getNextWp(i);
                    if (test_wp != -1) {
                        double d = (current_state.V[i].head(3) - goal_pt).norm();
                        if (d < min_d) {
                            min_d = d;
                            best_node = i;
                        }
                    }
                }
            }
            if (best_node != -1) {
                next_wp = getNextWp(best_node);
            }
        }

        // 3. Move the drone
        if (next_wp != -1) {
            Eigen::Vector4d target = current_state.V[next_wp];
            Eigen::Vector3d diff = target.head(3) - current_pose.head(3);
            double dist_to_wp = diff.norm();

            if (dist_to_wp > 0.5) { 
                Eigen::Vector3d dir = diff.normalized();
                double step = std::min(2.0, dist_to_wp); 
                current_pose.head(3) += dir * step; 
                current_pose[3] = target[3]; 
            }
        } else {
            std::cout << "Waiting for valid path..." << std::endl;
        }

        dist_to_goal = (current_pose.head(3) - goal_pt).norm();
        tick_count++;

        if (tick_count % 10 == 0) {
            std::cout << "Tick: " << tick_count << " | Dist to Goal: " << dist_to_goal << "m" << std::endl;
        }
    }

    data_file.close();
    std::cout << "\nSimulation finished. Data saved." << std::endl;
    return 0;
}