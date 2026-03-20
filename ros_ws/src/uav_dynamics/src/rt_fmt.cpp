#include "uav_dynamics/rt_fmt.hpp"
#include "uav_dynamics/uav_dubins_paths.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <random>



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
    std::vector<double> limits;
    std::vector<double> start;
    std::vector<double> goal;
    double w1;
    double w2;
    int N;
    int expandTreeRate;
    double rn;
    double goalRadius;
    double safeRadiusDObstacle;

    // Positions:
    std::vector<Eigen::Vector3d> V;

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
};



// Enumerate the type of analytical osbtacle;
enum ObstacleType {
    TYPE_CYLINDER = 0,
    TYPE_SPHERE = 1
};
struct AnalyticObstacle {
    Eigen::Vector3d p1; // [x1, y1, z1]
    Eigen::Vector3d p2; // [x2, y2, z2] (Used for cylinder)
    double radius;     
    int type;          
};



// Get the number of samples that the FMT requires:
int computeSamples(const std::vector<double>& limits){
    // Define the length between the limits:
    double Lx = limits[0][1] - limits[0][0];
    double Ly = limits[1][1] - limits[1][0];
    double Lz = limits[2][1] - limits[2][0];

    // Get the number fo nodes per dimension:
    long Nx = static_cast<long>(std::ceil(Lx/2)+1);
    long Ny = static_cast<long>(std::ceil(Ly/2)+1);
    long Nz = static_cast<long>(std::ceil(Lz/2)+1);
    // Obtian the oveall number of nodes:
    long N = Nx * Ny * Nz;
    return N;
}



// Check teh Occupancy of maps in teh node space:
double checkOccupancy(const std::vector<int>& map, const Eigen::Vector3d& pt) {
    return 0.0; // At the moment the maps are not defined so we stay them as 0
}



// Check if hte node is inside an annalytical obstacle:
bool isInsideObstacles(const Eigen::Vector3d& pt, const std::vector<AnalyticObstacle>& obstacles) {
    // If there is not obstacles go back:
    if (obstacles.empty()) return false;
 
    // Forloop between the obstacle and define the type of obstacle:
    for (const auto& obs : obstacles){
        // If is a sphere:
        if (obs.type == TYPE_SPHERE) {
            double distSq = (pt - obs.p1).squaredNorm();
            if (distSq <= (obs.radius * obs.radius)) {
                return true;
            }
        }

        // If is a cylinder:
        else if (obs.type == TYPE_CYLINDER) {
            Eigen::Vector3d AB = obs.p2 - obs.p1;
            Eigen::Vector3d AP = pt - obs.p1;
            double AB2 = AB.squaredNorm();
            if (AB2 < 1e-6) continue; 
            double t = AP.dot(AB) / AB2;

            // Check if projection falls within the finite segment [0, 1]
            if (t >= 0.0 && t <= 1.0) {
                // Distance from point to axis: ||AP - t*AB||^2
                Eigen::Vector3d orthogonalVec = AP - (t * AB);
                if (orthogonalVec.squaredNorm() <= (obs.radius * obs.radius)) {
                    return true;
                }
            }
        }

    }

    // If any of the obstacles are noy inside the radius threshold return false:
    return false;
}



// Create the Sample Free Space of the nodes inside the limits:
std::vector<Eigen::Vector3d> sampleFree(const std::vector<int>& map, const std::vector<std::vector<double>>& limits, 
    const Eigen::Vector3d& start, const Eigen::Vector3d& goal, int N, const std::vector<AnalyticObstacle>& anObst){
    // Create the sample vector:
    std::vector<Eigen::Vector3d> samples;
    // Increase the numebr of terms respect to the nuber of nodes:
    samples.reserve(N);

    // Create a random uniform vlues to lcoate the nodes within the limits:
    std::random_device rd;
    std::mt19937 gen(rd());
    // Get the uniform real distribution:
    std::uniform_real_distribution<double> distX(limits[0][0], limits[0][1]);
    std::uniform_real_distribution<double> disY(limits[1][0], limits[1][1]);
    std::uniform_real_distribution<double> disZ(limits[2][0], limits[2][1]);

    // Prevent an infinite loop while creating the free space by using a while loop
    int count = 0;
    while (count < N){
        // Get teh random position:
        Eigen::Vector3d pt(disX(gen), disY(gen), disZ(gen));

        // Check if an obstacle is inside:
        if (isInsideObstacles(pt, anObst)) {
            continue;
        }

        // Check the agains the map (Actually is not working):
        if (checkOccupancy(map, pt) < 0.5) {
            samples.push_back(pt);
            count++;
        }
    }

    // Add th nodes in addition to the start and end that should be free:
    std::vector<Eigen::Vector3d> nodes;
    nodes.reserve(samples.size() + 2);
    nodes.push_back(start);
    nodes.insert(nodes.end(), samples.begin(), samples.end());
    nodes.push_back(goal);

    return nodes;
}



// Get the cost of the nodes inside the search space:
std::vector<double> costMetric(const std::vector<Eigen::Vector3d>& V, const Eigen::VectorXd& x, double w1, double w2) {
    // Define the number of nodes:
    int N = V.size();
    // Number of entries that the vector has:
    int d = x.size();
    // Create a vector for the costs:
    std::vector<double> costs(N);

    // Get the costs of each node:
    for (int i=0; i <N; ++i) {
        double dist2 = (V[i].head(3) - x.head(3)).squaredNorm();
        // If the position is the goal or is only a 3D position only ocnsider the distance:
        if (d==3) {
            costs[i] = std::sqrt(dist2);
        }
        // Id is the state oif the UAS during fligth (NED (1-3), heading (4), fpa (5)):
        else if (d == 5){
            // Get the horizn and change in orth and east:
            double de = V[i](1) - x(1);
            double dn = V[i](0) - x(0);
            double dalt = V[i](2) - x(2);
            double horiz = std::max(std::sqrt(dn*dn + de*de), 1e-6);
            // Get the change in fpa and heading:
            double d_fpa = std::min(std::atan2(dalt, horiz) - x(4), 2 * M_PI -(std::atan2(dalt, horiz) - x(4)));
            double d_heding = std::min(std::atan2(dn, std::max(de, 1e-9)) - x(3), 2 * M_PI -(std::atan2(dn, std::max(de, 1e-9)))); 

            // Get the cost:
            costs[i] = std::sqrt(dist2 + std::pow(w1 * d_heding, 2) + std::pow(w2 * d_fpa, 2));
        }
    }
    // Retrun the new costs:
    return costs;

}



// Function to identify the neighbours and indices of node from a specific position:
std::pair<std::vector<int>, std::vector<Eigen::VectorXd>> near(const std::vector<Eigen::Vector3d>& V, const Eigen::Vector3d& x, double r, double w1, double w2){
    // Create a vector with the indices and neighbours nodes near the position:
    std::vector<int> indices;
    std::vector<Eigen::VectorXd> neighbors;
    // Get the costs of each node:
    std::vector<double> dAll = costMetric(V, x, w1, w2);

    // Define the neighbours a the points with cotst lower than a defined radius:
    for (int i = 0; i < static_cast<int>(dAll.size()); ++i) {
        if (dAll[i] < r) {
            indices.push_back(i);
            neighbors.push_back(V[i]);
        }
    }
    return {indices, neighbors};
}
 


// Function to create the base planner:
RTFMTPLannerState rt_fmt_planner( const std::vector<int>& map, const std::vector<std::vector<double>>& limits, const std::vector<double>& start, 
    const std::vector<double>& goal, double rn, RTFMTOptions opts = RTFMTOptions()){
    // Start the state structure:
    RTFMTPLannerState S;

    // Obtain the number of nodes if is not defined:
    if (opts.N == -1){
        opts.N = computeSamples(limits);
    }

    // Assign the planner parameters:
    S.map = map;
    S.limits = limits;
    S.start = start;
    S.goal = goal;
    S.w1 = opts.w1; // Heading weigth
    S.w2 = opts.w2; // Fpa weigth 
    S.N = opts.N + 2; 
    S.expandTreeRate = opts.expandTreeRate;
    S.rn = rn;

    // Create the Sample Free Space made by the nodes:
    S.V = sampleFree(S.map, S.limits, S.start, S.goal, opts.N);

    // Defind the goal radius and obstacle in case in not defined:
    S.goalRadius = std::isnan(opts.goalRadius) ? (S.rn / 2.0) : opts.goalRadius;
    S.safeRadiusDObstacle = std::isnan(opts.safeRadiusDObstacle) ? S.rn : opts.safeRadiusDObstacle;

    // Define the start idx from the root:
    S.startIdx = 0;
    S.goalIdx = S.V.size() - 1;
    S.rootIdx = S.startIdx;

    // Goal Region computatation:
    S.goalRegionIdx.push_back(S.goalIdx);
    auto neighbor_pair = near(S.V, S.V[S.goalIdx], S.goalRadius, S.w1, S.w2);
    for (int idx : neighbor_pair.first) {
        if (idx != S.goalIdx) {
            S.goalRegionIdx.push_back(idx);
        }
    }

    // Setup dimensions to the boolean flags:
    size_t numNodes = S.V.size();
    // Add all the flags bollena vectors used in the RT-FMT:
    S.parent.assign(numNodes, -1); 
    S.cost.assign(numNodes, std::numeric_limits<double>::infinity());
    S.state.assign(numNodes, 2); // (2) means unvisited
    
    // Initial idx:
    S.state[S.startIdx] = 1;
    S.cost[S.startIdx] = 0.0;
    S.z = S.startIdx;

    // Populate in teh unvisited stes:
    for (size_t i = 0; i < numNodes; ++i) {
        if (i != static_cast<size_t>(S.startIdx)) {
            S.unvis.push_back(i);
        }
    }

    // sET MASKS:
    S.isOpen.assign(numNodes, false);
    S.isOpen[S.startIdx] = true;
    S.blocked.assign(numNodes, false);
    S.openNew.assign(numNodes, false);
    S.closedToOpen.assign(numNodes, false);
    S.checkedPath.assign(numNodes, false);
    S.checkedPathCandidates.clear();

    // Returnt the structure related to the Search sapce in FMT;
    return S;

};



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
    for (size_t i = 0; i < moving_obstacles.states.size(); i++){
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