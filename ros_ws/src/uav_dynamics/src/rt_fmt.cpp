#include "uav_dynamics/rt_fmt.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <tuple>
#include <algorithm>
#include <Eigen/Dense>
#include <random>
#include <optional>



// Structure to define the dubins interpolator results:
struct DubinsInterpResult {
    std::vector<Eigen::VectorXd> intposes; // The interpolated 3D poses (N, E, D, yaw)
    double totalCost;
    int bestIdx;
};



// ENU -> NED conversion
inline static Eigen::Vector3d ENU_to_NED(const Eigen::Vector3d &enu) {
    return Eigen::Vector3d(enu.y(), enu.x(), -enu.z()); // (N,E,-U)
}



// Helper to wrap angles between 0 and 2*PI
inline double wrapTo2Pi(double theta) {
    double wrap = std::fmod(theta, 2.0 * M_PI);
    if (wrap < 0.0) wrap += 2.0 * M_PI;
    return wrap;
}



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
int computeSamples(const std::vector<std::vector<double>>& limits){
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
std::vector<Eigen::Vector4d> sampleFree(const std::vector<int>& map, const std::vector<std::vector<double>>& limits, 
    const Eigen::Vector4d& start, const Eigen::Vector4d& goal, int N, const std::vector<AnalyticObstacle>& anObst,
    const std::vector<Eigen::Vector3d>& waypoints){
    // Create the sample vector:
    std::vector<Eigen::Vector4d> samples;
    // Increase the numebr of terms respect to the nuber of nodes:
    samples.reserve(N);

    // Create a random uniform vlues to lcoate the nodes within the limits:
    std::random_device rd;
    std::mt19937 gen(rd());
    // Get the uniform real distribution:
    std::uniform_real_distribution<double> disX(limits[0][0], limits[0][1]);
    std::uniform_real_distribution<double> disY(limits[1][0], limits[1][1]);
    std::uniform_real_distribution<double> disZ(limits[2][0], limits[2][1]);
    // Generate random heading angles in the nodes:
    std::uniform_real_distribution<double> disYaw(0.0, 2.0 * M_PI);

    // Prevent an infinite loop while creating the free space by using a while loop
    int count = 0;
    while (count < N){
        // Get teh random position:
        Eigen::Vector4d pt(disX(gen), disY(gen), disZ(gen), disYaw(gen));

        // Check if an obstacle is inside:
        if (isInsideObstacles(pt.head(3), anObst)) {
            continue;
        }

        // Check the agains the map (Actually is not working):
        if (checkOccupancy(map, pt.head(3)) < 0.5) {
            samples.push_back(pt);
            count++;
        }
    }

    // Add th nodes in addition to the start and end that should be free:
    std::vector<Eigen::Vector4d> nodes;
    if (!waypoints.empty()){
        nodes.reserve(samples.size() + 2 + waypoints.size());
        nodes.push_back(start);
        nodes.insert(nodes.end(), samples.begin(), samples.end());

        // Create teh waypoints nodes adn add them:
        for (size_t i = 0; i < waypoints.size(); ++i) {
            Eigen::Vector3d curr_wp = waypoints[i];
            Eigen::Vector3d next_wp;

            // If is not teh lwast wayoint point, point to the next one:
            if (i < waypoints.size() - 1) {
                next_wp = waypoints[i + 1];
            } else {
                next_wp = goal.head(3);
            }
            // Push the fully defined 4D waypoint into the tree
            // Get the difference in position to get the heading:
            double dx = next_wp.x() - curr_wp.x();
            double dy = next_wp.y() - curr_wp.y();
            double yaw = std::atan2(dy, dx);
            if (yaw < 0) yaw += 2.0 * M_PI;
            nodes.push_back(Eigen::Vector4d(curr_wp.x(), curr_wp.y(), curr_wp.z(), yaw));
        }

        // Puh it inside the goal:
        nodes.push_back(goal);

    } else{
        nodes.reserve(samples.size() + 2);
        nodes.push_back(start);
        nodes.insert(nodes.end(), samples.begin(), samples.end());
        nodes.push_back(goal);
    }

    return nodes;
}



// Get the cost of the nodes inside the search space:
std::vector<double> costMetric(const std::vector<Eigen::Vector4d>& V, const Eigen::Vector4d& x, double w1, double w2) {
    // Define the number of nodes:
    int N = V.size();
    // Create a vector for the costs:
    std::vector<double> costs(N);

    // Get the costs of each node:
    for (int i = 0; i < N; ++i) {
        // 3D Distance squared
        double dist2 = (V[i].head(3) - x.head(3)).squaredNorm();
        
        // Heading difference wrapped to [0, pi]
        double rawPsi = std::abs(V[i](3) - x(3)); 
        double dpsi = std::min(rawPsi, 2.0 * M_PI - rawPsi);
        
        // Total cost including the heading penalty
        costs[i] = std::sqrt(dist2 + std::pow(w1 * dpsi, 2));
    }
    return costs;
}



// Function to identify the neighbours and indices of node from a specific position:
std::pair<std::vector<int>, std::vector<Eigen::VectorXd>> near(const std::vector<Eigen::Vector4d>& V, const Eigen::Vector4d& x, double r, double w1, double w2){
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
void start_rt_fmt( const std::vector<int>& map, const std::vector<std::vector<double>>& limits, const std::vector<double>& start, 
    const std::vector<double>& goal, double rn, FMTPlanner& rt_fmt_planner, double max_roll, double air_speed, double fpa_limits[2],
    const std::vector<Eigen::Vector3d>& waypoints){
    // Start the state structure:
    RTFMTPLannerState S;

    // Define the options:
    RTFMTOptions opts = rt_fmt_planner.rt_fmt_opts;

    // Obtain the number of nodes if is not defined as one node seperade by each meter
    if (opts.N == -1){
        opts.N = computeSamples(limits);
    }

    // Create the dubins connector:
    FMTDubinsConnector dubins_conn(max_roll, air_speed, fpa_limits[0], fpa_limits[1]);

    // Assign the planner parameters:
    S.map = map;
    S.limits = limits;
    S.w1 = opts.w1; // Heading weigth
    S.w2 = opts.w2; // Fpa weigth 
    S.expandTreeRate = opts.expandTreeRate;
    S.rn = rn;
    // Add waypoitns for gaol and start adn waypoints inc ase it has it:
    if (!waypoints.empty()){
        S.N = opts.N + 2 + waypoints.size(); 
    } else {
        S.N = opts.N + 2;
    }

    // Define the starting point and the goal:
    S.start << start[0], start[1], start[2], start[3];
    S.goal << goal[0], goal[1], goal[2], goal[3];
    
    // Create the Sample Free Space made by the nodes:
    std::vector<AnalyticObstacle> empty_obstacles;
    S.V = sampleFree(S.map, S.limits, S.start, S.goal, opts.N, empty_obstacles, waypoints);

    // Defind the goal radius and obstacle in case in not defined:
    S.goalRadius = std::isnan(opts.goal_radius) ? (S.rn / 2.0) : opts.goal_radius;
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
    S.z = S.startIdx; // Current node

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

    // Add the mask and the dubins path to the 
    rt_fmt_planner.ft_fmt_mask = S;
    rt_fmt_planner.dubins_connector = dubins_conn;
};



// Function to identify if an obstacle is interfering the actual Trajectory thresholded by a range limit:
FMTBundle FMT_Detect(const uav_dynamics::msg::AvoidanceStates &moving_obstacles, double own_e, double own_n, double own_u,double own_ve, double own_vn, double own_vu,
    double crit_time, FMTNavigationState& nav_state, double min_radius){
    // Add a structure to save all teh conflicting obstacles:
    FMTBundle fmt_bundle;
    // Own velocity norm:
    const double own_vel_a = std::sqrt(own_vn*own_vn+own_ve*own_ve+own_vu*own_vu);
    // Ownship position:
    const Eigen::Vector3d own_pos(own_n,own_e,-own_u);

    // Loop betwen all the moving obstacles to see if they intersect with the waypoints:
    for (size_t i = 0; i < moving_obstacles.intruder_states.size(); i++){
        // Get the posstion of the obstacles:
        const auto &obs = moving_obstacles.intruder_states[i];
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
                const double d_own = (nav_state.waypoints[wp_idx].head(3) - own_pos).norm();
                // If the waypoint is within the lookahead search distance
                if (d_own < 1.25 * search_radius) {
                    Eigen::Vector3d dW = nav_state.waypoints[wp_idx + 1].head(3) - nav_state.waypoints[wp_idx].head(3);
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
                            double dist_to_obs = (nav_state.waypoints[k].head(3) - obs_pos).norm();
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



// FUnction to interpolate the poses during the Dubins motion primitives:
Eigen::VectorXd interpPose(const coder::uavDubinsPathSegment& seg, double target_dist){
    // Initialize te positiom:
    Eigen::VectorXd pose(4);
    pose << seg.StartPose[0], seg.StartPose[1], seg.StartPose[2], seg.StartPose[3];

    // Clamp distance to the maximum length of the path:
    target_dist = std::max(0.0, std::min(target_dist, seg.Length));
    double remaining_dist = target_dist;

    // Define the fpa characteritics:
    double fpa = seg.FlightPathAngle;
    double cos_fpa = std::cos(fpa);
    double sin_fpa = std::sin(fpa);

    // Loop between the 4 possible motion segments:
    for (int i = 0; i < 4; i++){
        if (remaining_dist <= 1e-6) break;

        // Define segment lenght:
        double seg_len = seg.MotionLengths[i];
        if (seg_len <= 1e-6) continue;

        // How much of the current segemnt the UAV is traveling:
        double d = std::min(remaining_dist, seg_len);
        // HOrizontal y vertical distance:
        double d_xy = d * cos_fpa;
        double d_z = d * sin_fpa;
        // Type of dubins trajectory:
        char type = seg.MotionTypes[i].f1.data[0];

        // Detect if the type is a helix and its direction:
        char dir = type;
        if (type == 'H' && seg.MotionTypes[i].f1.size[1] > 1) {
            dir = seg.MotionTypes[i].f1.data[1];
        }

        // Substract the intial position coordiantes to get the next segment:
        double x0 = pose[0];
        double y0 = pose[1];
        double z0 = pose[2];
        double psi0 = pose[3];
        // Straight:
        if (dir == 'S') {
            // Straight line kinematics
            pose[0] = x0 + d_xy * std::cos(psi0);
            pose[1] = y0 + d_xy * std::sin(psi0);
            pose[2] = z0 + d_z;
            pose[3] = psi0; 
        }
        // Left turn or Left Heliz kinematics:
        else if (dir == 'L' || dir == 'l') {
            double R = (type == 'H') ? seg.HelixRadius : seg.MinTurningRadius;
            double delta_psi = d_xy / R;

            pose[0] = x0 + R * (std::sin(psi0 + delta_psi) - std::sin(psi0));
            pose[1] = y0 - R * (std::cos(psi0 + delta_psi) - std::cos(psi0));
            pose[2] = z0 + d_z;
            pose[3] = wrapTo2Pi(psi0 + delta_psi);
        }
        // Right turn or Right helix kinematics
        else if (dir == 'R' || dir == 'r') {
            double R = (type == 'H') ? seg.HelixRadius : seg.MinTurningRadius;
            double delta_psi = -d_xy / R;

            pose[0] = x0 - R * (std::sin(psi0 + delta_psi) - std::sin(psi0));
            pose[1] = y0 + R * (std::cos(psi0 + delta_psi) - std::cos(psi0));
            pose[2] = z0 + d_z;
            pose[3] = wrapTo2Pi(psi0 + delta_psi);
        }

        remaining_dist -= d;
    }

    // Retrun the trajectory
    return pose;
}



// FUNction to interpolate using the dubins connector:
DubinsInterpResult interDubins(FMTDubinsConnector& conn, const double state1[4], const double state2[4], double costs_y, bool interp_pos){
    DubinsInterpResult interp_results;
    double outLengths[4] = {0.0, 0.0, 0.0, 0.0};

    // Create teh dubins path:
    coder::uavDubinsPathSegment pathSeg = conn.computePath(state1, state2, outLengths);

    // Extract length and get th the total costs:
    double dist = pathSeg.Length;
    interp_results.totalCost = costs_y + dist;
    interp_results.bestIdx = 0;

    // Interpolate dubins positions only if necessary:
    if (interp_pos){
        if (dist > 1e-6){
            // Define an interval to do the segmentation:
            double interval_dist = 50.0;

            // Loop though and clacualte the exact pose at each distance:
            for (double current_dist = 0.0; current_dist < dist; current_dist += interval_dist) {
                interp_results.intposes.push_back(interpPose(pathSeg, current_dist));
            }

            // Ensure the exact goal state is always pushed as the final point
            Eigen::VectorXd final_pose(4);
            final_pose << state2[0], state2[1], state2[2], wrapTo2Pi(state2[3]);
            interp_results.intposes.push_back(final_pose);
        } else {
            // If the distance is 0, just push the start state
            Eigen::VectorXd start_pose(4);
            start_pose << state1[0], state1[1], state1[2], wrapTo2Pi(state1[3]);
            interp_results.intposes.push_back(start_pose);
        }
    }

    // Return the interpolated resutls witht eh segment:
    return interp_results;
}



// FUnction to recall the costs of childrens nodes:
void recalChildrenCost(int seed, RTFMTPLannerState& S, FMTDubinsConnector& conn, std::optional<double> oldSeedCost = std::nullopt){
    // Define the numer of nodes:
    size_t N = S.V.size();

    // In case there is an old cost value only update it with a delta:
    if (oldSeedCost.has_value()){
        double delta = S.cost[seed] - oldSeedCost.value();
        if (std::abs(delta) < 1e-9){
            return;
        }

        // FInd the children of the seed node
        std::queue<int> Q;
        for (size_t i = 0; i < N; i++){
            if (S.parent[i] == seed) Q.push(i);
        }

        // If Q is not empty add the new cost:
        while (!Q.empty()) {
            int u = Q.front();
            Q.pop();
            // Update cost if it is finite
            if (!std::isinf(S.cost[u])) {
                S.cost[u] += delta;
            }
            S.blocked[u] = std::isinf(S.cost[u]);

            // Push grandchildren
            for (size_t i = 0; i < N; ++i) {
                if (S.parent[i] == u) Q.push(i);
            }
        }
        return;
    }

    // INcase you ned to reompute the cost of all children:
    std::queue<int> Q;
    for (size_t i = 0; i < N; i++){
        if (S.parent[i] == seed) Q.push(i);
    }

    // Check for the childreand and grandchildren to update teh costs:
    while (!Q.empty()){
        int u = Q.front();
        Q.pop();
        int  p = S.parent[u];

        // Check the aprent if its the startidx add a 0 costr as it is already there:
        if (p == -1) { 
            S.cost[u] = 0.0;
            S.blocked[u] = std::isinf(S.cost[u]);
        } else {
            // CHeck if the aprent is blocked:
            if (std::isinf(S.cost[p])) {
                S.cost[u] = std::numeric_limits<double>::infinity();
                S.blocked[u] = true;
            } else {
                
                // Create the 4D state arrays expected by interDubins
                double state_p[4] = {S.V[p](0), S.V[p](1), S.V[p](2), S.V[p](3)};
                double state_u[4] = {S.V[u](0), S.V[u](1), S.V[u](2), S.V[u](3)};
                
                // Pass the arrays to interDubins
                DubinsInterpResult dubins_interp = interDubins(conn, state_p, state_u, 0.0, false);

                // Cckacualte teh cost:
                double rawPsi = std::abs(S.V[p](3) - S.V[u](3));
                double d_heading = std::min(rawPsi, 2.0 * M_PI - rawPsi);
                double headingPenalty = std::pow(S.w1 * d_heading, 2);
                S.cost[u] = S.cost[p] + dubins_interp.totalCost + headingPenalty;
                S.blocked[u] = std::isinf(S.cost[u]);
            }
        }

        // Push the grandchidlren to the queqe:
        for (size_t i = 0; i < N; ++i) {
            if (S.parent[i] == u) Q.push(i);
        }
    }

}



// Function to update/block near dynamic obstacles:
RTFMTPLannerState updateObstructedNodes(FMTPlanner& rt_fmt_planner, FMTBundle& active_obs) {
    // Start by defning the Mask:
    RTFMTPLannerState& S = rt_fmt_planner.ft_fmt_mask;
    // Dubins connector:
    FMTDubinsConnector conn = rt_fmt_planner.dubins_connector;
    size_t N = S.V.size();

    // Build a cosntruct the dynamic obstruction mask:
    std::vector<bool> cur(N, false);
    for (size_t i = 0; i < N; i++){
        for (const auto &o : active_obs.act_obs){
            double distSq = (S.V[i].head(3) - o.obs_pos).squaredNorm();
            if (distSq < o.dm*o.dm){
                cur[i] = true;
                break;
            }
        }
    }
    // Add the actual dynamic obstacle position:
    S.dynamicObstructed = cur;

    // Check the previous Obstructrd nodes and check if size mismatch start it first:
    if (S.dynamicObstructedPrev.size() != N) {
        S.dynamicObstructedPrev.assign(N, false);
    }

    // Compute the transition of the pobstacles:
    std::vector<int> newlyBlockedIdx;
    std::vector<int> newlyUnblockedIdx;
    for (size_t i = 0; i < N; ++i) {
        bool isNewlyBlocked = cur[i] && !S.dynamicObstructedPrev[i];
        bool isNewlyUnblocked = !cur[i] && S.dynamicObstructedPrev[i];

        if (isNewlyBlocked) newlyBlockedIdx.push_back(i);
        if (isNewlyUnblocked) newlyUnblockedIdx.push_back(i);
    }
    // Update teh previous obstructed nodes for the next iterations:
    S.dynamicObstructedPrev = cur;

    // New blcoked nodes costs are infinity:
    for (int t : newlyBlockedIdx) {
        if (!S.blocked[t]){
            S.cost[t] = std::numeric_limits<double>::infinity();
            S.blocked[t] = true;
            recalChildrenCost(t, S, conn);
        }
    }

    // Newly unblocked nodes lets recompute the costs:
    for (int t : newlyUnblockedIdx) {
        // Define the parents;
        int p = S.parent[t];
        
        //Define a zero costs if one of th new unbblcoked nodes is nthe orgin:
        if (p == S.rootIdx) { 
            S.cost[t] = 0.0;
            S.blocked[t] = false;
            recalChildrenCost(t, S, conn);
        } // FOr the other cases clacualte the new costs
        else if (p != -1 && S.cost[p] < std::numeric_limits<double>::infinity()) {
            double state_p[4] = {S.V[p](0), S.V[p](1), S.V[p](2), S.V[p](3)};
            double state_t1[4] = {S.V[t](0), S.V[t](1), S.V[t](2), S.V[t](3)};
            DubinsInterpResult dubins_interp_1 = interDubins(conn, state_p, state_t1, 0.0, false);
            S.cost[t] = S.cost[p] + dubins_interp_1.totalCost;
            S.blocked[t] = false;

            recalChildrenCost(t, S, conn);
        }
    }

    // REtrurnt he nodes which their respective new costs 
    return S;
}



// REverse parent pointes when ther is a new root:
void reversePathToRoot(RTFMTPLannerState& S, int newRoot, int oldRoot) {
    // Defien a varible to check if it has a parent:
    int prev = -1;
    // Defien teh new root:
    int cur = newRoot;
    // Defie the mximum hopes witht he number of aprents:
    size_t maxHops = S.parent.size() + 1;

    // loop between all the nodes and reverse the links:
    for (size_t k = 0; k < maxHops; ++k){
        // Prevent Out-of-Bounds Segmentation Faults
        if (cur < 0 || cur >= static_cast<int>(S.parent.size())) {
            break; 
        }

        int next = S.parent[cur];
        S.parent[cur] = prev;

        // If the new aprent is equal to the oldRoot breturn:
        if (cur == oldRoot){
            break;
        }

        prev = cur;
        cur = next;

        // If the new root is -1 the tree is broken:
        if (cur == -1){
            break;
        }
    }
}



// FUnction that sees id Cnadidate Paretn is rooted in a subtree:
bool isDescendant(int candidateParent, int x, const RTFMTPLannerState& S) {
    bool descendant = false;
    int curr = candidateParent;

    // Walk back in the tree to seach for the parent root:
    while (curr != -1) {
        if (curr == x) {
            descendant = true;
            return descendant;
        }

        curr = S.parent[curr];
    }
    
    return descendant;
}



// FInd neighbors in the space that have a specific state:
std::vector<int> nearStates(int xIdx, const std::vector<int>& stateDes, const RTFMTPLannerState& S) {
    std::vector<int> Y;

    // cALL THE NEAR FUCNTION TO GET THE neighburs:
    auto nearResult = near(S.V, S.V[xIdx], S.rn, S.w1, S.w2);
    const std::vector<int>& Nidx = nearResult.first;

    // FIlter consifering the desired state:
    for (int idx : Nidx) {
        bool isMember = false;
        for (int desiredState : stateDes){
            if (S.state[idx] == desiredState) {
                isMember = true;
                break;
            }
        }

        // If is member add it to the neighbors:
        if (isMember) {
            Y.push_back(idx);
        }
    } 

    return Y;
}



// Check if the interpolated path collides with any dynamic obstacle
bool segmentIntersectsSphere3D(const std::vector<Eigen::VectorXd>& pts, const FMTBundle& dynObs) {
    bool collides = false;

    if (pts.empty() || dynObs.act_obs.empty()) {
        return collides;
    }

    // Loop through every point in the interpolated Dubins path
    for (const auto& pt : pts) {
        Eigen::Vector3d pt3d = pt.head(3);
        
        // Loop through every dynamic obstacle
        for (const auto& obs : dynObs.act_obs) {
            // Squared distance: dx^2 + dy^2 + dz^2
            double dist2 = (pt3d - obs.obs_pos).squaredNorm();
            
            // Compare against safety radius squared
            if (dist2 <= (obs.dm * obs.dm)) {
                collides = true;
                return collides;
            }
        }
    }
    
    return collides;
}



// Function to build and find the best way o joining near nodes:
void rewireFromRoot2(FMTPlanner& rt_fmt_planner, RTFMTPLannerState& S, const FMTBundle& active_obs) {
    FMTDubinsConnector conn = rt_fmt_planner.dubins_connector;

    // Initialize rewire frontier from root if needed
    if (S.rewireRootList.empty()) {
        S.rewireRootList.push_back(S.rootIdx);
        
        // Reset the seen mask
        if (S.rewireRootSeen.size() != S.V.size()) {
            S.rewireRootSeen.assign(S.V.size(), false);
        } else {
            std::fill(S.rewireRootSeen.begin(), S.rewireRootSeen.end(), false);
        }
        S.rewireRootSeen[S.rootIdx] = true;
    }

    // Deifne first node to rewrite:
    int xChild = S.rewireRootList.front();
    S.rewireRootList.pop_front();

    // The candiddate parents are the ope or closed neighnors near the child:
    std::vector<int> targetStates = {1, 3};
    std::vector<int> YParents = nearStates(xChild, targetStates, S);

    // gET EH OLD COST:
    double cost_old = S.cost[xChild];

    // Loop betweeen the parents:
    for (int y : YParents) {
        // Check if the child is obstructed:
        if (S.dynamicObstructed[xChild]) {
            continue;
        }

        // If the endpoint is blockd continue too:
        if (S.dynamicObstructed[y] || S.dynamicObstructed[xChild]) {
            continue;
        }

        // Get the interpolated dubins path:
        double state_y[4] = {S.V[y](0), S.V[y](1), S.V[y](2), S.V[y](3)};
        double state_x[4] = {S.V[xChild](0), S.V[xChild](1), S.V[xChild](2), S.V[xChild](3)};
        DubinsInterpResult dubins_interp = interDubins(conn, state_y, state_x, 0.0, true);

        // GEt the new cost:
        double rawPsi = std::abs(S.V[xChild](3) - S.V[y](3));
        double d_heading = std::min(rawPsi, 2.0 * M_PI - rawPsi);
        double headingPenalty = std::pow(S.w1 * d_heading, 2);
        double cost_new = S.cost[y] + dubins_interp.totalCost + headingPenalty;
        
        // Static obstacle check along the interpolated points:
        bool tf_static = true;
        for (const auto& pt : dubins_interp.intposes) {
            Eigen::Vector3d p3d = pt.head(3);
            if (checkOccupancy(S.map, p3d) >= 0.5) { 
                tf_static = false;
                break;
            }
        }
        if (!tf_static) continue;

        // Cehck if the dynamic obstacles are along the interpolated points:
        if (segmentIntersectsSphere3D(dubins_interp.intposes, active_obs)) {
            continue;
        }

        // If the cost is lower, makjke this node as the new parent:
        if ((cost_new < cost_old) && !isDescendant(y, xChild, S) && (xChild != y)) {
            double oldSeedCost = S.cost[xChild];
            S.parent[xChild] = y;
            S.cost[xChild] = cost_new;
            S.blocked[xChild] = false;
            
            // Recompute children costs with the delta (oldSeedCost)
            recalChildrenCost(xChild, S, conn, oldSeedCost);
            
            cost_old = cost_new; // update best cost
        }

        // Add Y to the queques if y is not seen:
        if (!S.rewireRootSeen[y]) {
            S.rewireRootList.push_back(y);
            S.rewireRootSeen[y] = true;
        }

    }

}
 


// Set the current pose as the root of the tree:
void setCurrentPose(const Eigen::VectorXd& q, FMTPlanner& rt_fmt_planner, FMTBundle& act_obs) {
    RTFMTPLannerState& S = rt_fmt_planner.ft_fmt_mask;

    // If is the first root:
    if (S.lastRootIdx == -1) S.lastRootIdx = S.rootIdx;
    int oldRoot = S.rootIdx;

    // Find the nearest node:
    int nearIdx = -1;
    double dNear = std::numeric_limits<double>::infinity();
    Eigen::Vector4d q_pos = q.head(4);

    // Loop between teh nodes to fiund the nearest vertex:
    for (size_t i = 0; i < S.V.size(); ++i) {
        if (!std::isinf(S.cost[i])) {
            double distSq = (S.V[i].head(3) - q_pos.head(3)).squaredNorm();
            if (distSq < dNear) {
                dNear = distSq;
                nearIdx = i;
            }
        }
    }

    // Definethe nearest distance:
    dNear = std::sqrt(dNear);
    // Define the threshold:
    double insertThresh = S.rn;

    // Insert a new Node id the node is far from the actual vectors:
    if (nearIdx == -1 || dNear > insertThresh){
        int newIdx = S.V.size();

        // Push back to ALL parallel vectors to maintain alignment
        S.V.push_back(q_pos);
        S.parent.push_back(-1); 
        S.cost.push_back(0.0);
        S.state.push_back(1); 
        
        S.blocked.push_back(false);
        S.openNew.push_back(false);
        S.closedToOpen.push_back(false);
        S.dynamicObstructed.push_back(false);
        S.dynamicObstructedPrev.push_back(false);
        S.rewireRootSeen.push_back(false);
        S.isOpen.push_back(false);

        // If the oldROot is not new attach it to the new tree:
        if (oldRoot != -1) {
            S.parent[oldRoot] = newIdx; 
        }

        S.rootIdx = newIdx;
        S.N = newIdx + 1;
    } 
    // If there exists tehn Re-root all the tree to the new origin:
    else {
        int newRoot = nearIdx;
        if (newRoot != oldRoot) {
            reversePathToRoot(S, newRoot, oldRoot);
            S.rootIdx = newRoot;
        }

        // REmember the cots and the partnts are the origin so:
        S.parent[S.rootIdx] = -1; 
        S.cost[S.rootIdx] = 0.0;
        S.state[S.rootIdx] = 1;
    }

    // Deefine the root idx:
    S.z = S.rootIdx;

    // FInd the neighbors:
    S.Nz = near(S.V, S.V[S.z], S.rn, S.w1, S.w2);

    // Recompute the costs:
    FMTDubinsConnector conn = rt_fmt_planner.dubins_connector;
    recalChildrenCost(S.rootIdx, S, conn);

    // Get the best way to join neighbour nodes:
    rewireFromRoot2(rt_fmt_planner, S, act_obs);

    // DEfine the new root IDX:
    S.lastRootIdx = S.rootIdx;

}



// Evaluate candidate parents and check for collisions:
std::tuple<int, double, bool> selectBestParent(int xIdx, std::vector<int>& YNear, 
    FMTPlanner& rt_fmt_planner, RTFMTPLannerState& S, const FMTBundle& act_obs) {
    // Get the best idx:
    int yMin = -1;
    double yCost = std::numeric_limits<double>::infinity();
    bool isCFixed = false;

    // If there is nto tneighbours return:
    if (YNear.empty()) return {yMin, yCost, isCFixed};

    // DEfien the dubnis connector and get the child state:
    FMTDubinsConnector conn = rt_fmt_planner.dubins_connector;
    Eigen::Vector4d childState = S.V[xIdx];

    // Check each near node:
    for (int y : YNear) {
        Eigen::Vector4d parentState = S.V[y];
        
        // Calculate Dubins path and cost
        double state_p[4] = {parentState(0), parentState(1), parentState(2), parentState(3)};
        double state_c[4] = {childState(0), childState(1), childState(2), childState(3)};
        DubinsInterpResult dubins_interp = interDubins(conn, state_p, state_c, 0.0, true);

        double rawPsi = std::abs(childState(3) - parentState(3));
        double d_heading = std::min(rawPsi, 2.0 * M_PI - rawPsi);
        double headingPenalty = std::pow(S.w1 * d_heading, 2);
        
        double totalCost = S.cost[y] + dubins_interp.totalCost + headingPenalty;
        
        // If new cost is best than past code update:
        if (totalCost < yCost) {
            // Check for Static osbtacles:
            bool tf_static = true;
            for (const auto& pt : dubins_interp.intposes) {
                if (checkOccupancy(S.map, pt.head(3)) >= 0.5) {
                    tf_static = false;
                    break;
                }
            }
            if (!tf_static) continue;

            // Check for dynamic obstacles:
            if (S.dynamicObstructed[y] || S.dynamicObstructed[xIdx]) continue;
            if (segmentIntersectsSphere3D(dubins_interp.intposes, act_obs)) continue;

            // Define the min idx parent with the minn cost:
            yMin = y;
            yCost = totalCost;
            isCFixed = true;
        }
    }

    return {yMin, yCost, isCFixed};
}



// Function to re-parent the blocked children
void rewireLocally(FMTPlanner& rt_fmt_planner, RTFMTPLannerState& S, const FMTBundle& act_obs) {
    // Extraxt a list of currrently blocked nodes:
    if (S.rewireLocalList.empty()) {
        for (size_t i = 0; i < S.blocked.size(); ++i){
            if (S.blocked[i]) S.rewireLocalList.push_back(i);
        }
    }
    if (S.rewireLocalList.empty()) return;

    // Get the firt element:
    int x = S.rewireLocalList.front();
    S.rewireLocalList.pop_front();

    // Skip if the node is inside a dynamic obstacle:
    if (S.dynamicObstructed[x]) return;

    // Get the open and close states near:
    std::vector<int> targetStates = {1, 3};
    std::vector<int> YNear = nearStates(x, targetStates, S);

    // Select the best parent:
    auto [yMin, yCost, isCFixed] = selectBestParent(x, YNear, rt_fmt_planner, S, act_obs);

    // If it has a valid parent reconnect it:
    if (yMin != -1 && isCFixed) {
        if (!std::isinf(S.cost[yMin]) && !isDescendant(yMin, x, S) && x != yMin) {
            double oldSeedCost = S.cost[x];
            S.parent[x] = yMin;
            S.cost[x] = yCost;
            
            FMTDubinsConnector conn = rt_fmt_planner.dubins_connector;
            recalChildrenCost(x, S, conn, oldSeedCost);
            S.blocked[x] = false;
        }
    }

    
}



// Helper function to attach a child node to the tree:
void addChild(int child, int parentIdx, double totalCost, RTFMTPLannerState& S) {
    // Remove the child from the unvisited list 
    S.unvis.erase(std::remove(S.unvis.begin(), S.unvis.end(), child), S.unvis.end());
    
    S.parent[child] = parentIdx;
    S.cost[child] = totalCost;
    S.state[child] = 1;       // Open
    S.isOpen[child] = true;
    S.openNew[child] = true;

    // If ir has a path fromt eh child it i aded
    if (!S.checkedPath[child]) {
        int p = parentIdx;
        while (p > -1) { 
            if (S.checkedPath[p]) S.checkedPath[p] = false;
            
            auto it = std::find(S.checkedPathCandidates.begin(), S.checkedPathCandidates.end(), p);
            if (it != S.checkedPathCandidates.end()) {
                S.checkedPathCandidates.erase(it);
            }
            
            p = S.parent[p];

            // If p is the original parentIdx break
            if (p == parentIdx) break;
        }
    }
}



// Function to check if the edge us free to fly there:
bool isEdgeFixedFree(int i, int j, FMTPlanner& rt_fmt_planner, const RTFMTPLannerState& S, const FMTBundle& act_obs) {
    FMTDubinsConnector conn = rt_fmt_planner.dubins_connector;
    
    // Interpolate the path using Dubins
    double state_i[4] = {S.V[i](0), S.V[i](1), S.V[i](2), S.V[i](3)};
    double state_j[4] = {S.V[j](0), S.V[j](1), S.V[j](2), S.V[j](3)};
    
    // interp_pos must be true to get the intermediate points
    DubinsInterpResult dubins_interp = interDubins(conn, state_i, state_j, 0.0, true);

    //Static obstacle check
    for (const auto& pt : dubins_interp.intposes) {
        if (checkOccupancy(S.map, pt.head(3)) >= 0.5) { // Assuming 0.5 is your FreeThreshold
            return false;
        }
    }

    //  Dynamic obstacle checks
    if (act_obs.act_obs.empty()) {
        return true;
    }

    // If either endpoint is already flagged as dynamically obstructed, block edge
    if (S.dynamicObstructed[i] || S.dynamicObstructed[j]) {
        return false;
    }

    // Check if the interpolated path hits any moving obstacles
    if (segmentIntersectsSphere3D(dubins_interp.intposes, act_obs)) {
        return false;
    }

    return true;
}



// FUnction to expand the rt-fmt TREE:
void expandTree(FMTPlanner& rt_fmt_planner, RTFMTPLannerState& S, const FMTBundle& act_obs) {
    // REfresh if the unvisited candidate nodes are not in XNear:
    if (!S.unvis.empty() && S.XNear.empty() && S.z != -1) {
        auto N_z_pair = near(S.V, S.V[S.z], S.rn, S.w1, S.w2);
        std::vector<int> Nz = N_z_pair.first;
        
        // Intersect Nz and unvis (unvisited nodes)
        for (int nz_val : Nz) {
            if (std::find(S.unvis.begin(), S.unvis.end(), nz_val) != S.unvis.end()) {
                S.XNear.push_back(nz_val);
            }
        }
    }

    // COnnect one X from XNear:
    if (!S.XNear.empty()){
        int x = S.XNear.back();
        S.XNear.pop_back();

        // Gte the near states to x form teh OPen Nodes {1}
        std::vector<int> targetStates = {1}; 
        std::vector<int> YNear = nearStates(x, targetStates, S);

        // Get the best parent
        auto [yMin, yCost, isCFixed] = selectBestParent(x, YNear, rt_fmt_planner, S, act_obs);

        if (yMin != -1 && isCFixed) {
            bool isCNodeOK = !S.dynamicObstructed[x];
            if (!std::isinf(S.cost[yMin]) && isCNodeOK && !isDescendant(yMin, x, S) && x != yMin) {
                // Add A child if the cost is not infinity
                addChild(x, yMin, yCost, S);
            }
        }
    }

    // Transition State when XNear is empty:
    if (S.XNear.empty() && S.z != -1) {
        for (size_t i = 0; i < S.openNew.size(); ++i) {
            if (S.openNew[i]) {
                S.isOpen[i] = true;
                S.state[i] = 1;
                S.openNew[i] = false;
            }
        }

        // Close the current node
        int zClosed = S.z;
        S.isOpen[zClosed] = false;
        S.state[zClosed] = 3;

        // Cehck unvisited neighbours:
        auto zNbrPair = near(S.V, S.V[zClosed], S.rn, S.w1, S.w2);
        bool zHasCF = false;
        for (int k : zNbrPair.first) {
            // Check if k is unvisited
            if (std::find(S.unvis.begin(), S.unvis.end(), k) != S.unvis.end()) {
                if (isEdgeFixedFree(zClosed, k, rt_fmt_planner, S, act_obs)) { 
                    zHasCF = true;
                    break;
                }
            }
        }
        if (zHasCF) {
            S.closedToOpen[zClosed] = true;
        }


        // FInd the new z by getting the lowest cost to expand the tree:
        S.z = -1;
        double min_cost = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < S.isOpen.size(); ++i) {
            if (S.isOpen[i] && S.cost[i] < min_cost) {
                min_cost = S.cost[i];
                S.z = i;
            }
        }
    }

    // IF THE ACTUal position is not find restart the tree:
    if (S.z == -1) {
        bool reopenedAny = false;
        for (size_t i = 0; i < S.closedToOpen.size(); ++i) {
            if (S.closedToOpen[i]) {
                S.state[i] = 1;          
                S.isOpen[i] = true;    
                S.closedToOpen[i] = false;
                reopenedAny = true;
            }
        }

        if (reopenedAny) {
            S.z = -1;
            double min_cost = std::numeric_limits<double>::infinity();
            for (size_t i = 0; i < S.isOpen.size(); ++i) {
                if (S.isOpen[i] && S.cost[i] < min_cost) {
                    min_cost = S.cost[i];
                    S.z = i;
                }
            }
            S.XNear.clear(); 
        }
    }
}



// Function to check if a nde in the goal set i=s connected to teh tree:
std::tuple<bool, int> isInTree(const std::vector<int>& goalSet, const RTFMTPLannerState& S) {
    int best_node = -1;
    double min_cost = std::numeric_limits<double>::infinity();
    bool reached = false;

    // WItht eh gaol set find hte best node idx:
    for (int g: goalSet) {
        if (S.cost[g] < std::numeric_limits<double>::infinity() && (S.parent[g] != -1 || g == S.rootIdx)) {
            reached = true;
            if (S.cost[g] < min_cost) {
                min_cost = S.cost[g];
                best_node = g;
            }
        }
    }

    return {reached, best_node};
}



//FUntio to get the distance from a node to the closest node:
double endToGoalRegionDist(int endIdx, const RTFMTPLannerState& S) {
    double min_dist = std::numeric_limits<double>::infinity();
    Eigen::Vector3d end_pos = S.V[endIdx].head(3);

    // Get the distance between the best node and the goal
    for (int g : S.goalRegionIdx) {
        double d = (S.V[g].head(3) - end_pos).norm();
        if (d < min_dist) min_dist = d;
    }
    return min_dist;
}



// Fucntion to obtain the parents from the goal and back to the root/:
std::vector<int> generatePathBackward(int g, const RTFMTPLannerState& S) {
    std::vector<int> path;
    int curr = g;

    // Go back in the tree of the current tree
    while (curr != -1){
        path.push_back(curr);
        curr = S.parent[curr];
    }

    // Reverse the list so it goes to the goa, and not from the goal:
    std::reverse(path.begin(), path.end());
    return path;
}



// FInd the most promissing child from the tree:
int findBestChild(int c, const RTFMTPLannerState& S){
    int best_child = -1;
    double min_f = std::numeric_limits<double>::infinity();
    Eigen::Vector3d goal_pos = S.V[S.goalIdx].head(3);

    // Do a foorloop t6o find a promissing chiold form the parents list using the eucledian distnace:
    for (size_t i = 0; i < S.parent.size(); i++){
        if (S.parent[i] == c && S.cost[i] < std::numeric_limits<double>::infinity() && !S.checkedPath[i]) {
            double h = (S.V[i].head(3) - goal_pos).norm();
            double f = S.cost[i] + h;
            
            if (f < min_f) {
                min_f = f;
                best_child = i;
            }
        }
    }
    return best_child;
}



// Fucntion to build a forward path from root using "best child" selection.
std::vector<int> generatePathForward(int root, RTFMTPLannerState& S) {
    std::vector<int> candidatePath;
    int c = root;
    candidatePath.push_back(c);

    //Funciton to erify that the node and ial regions are not the same as goalIdx
    auto isGoal = [&](int node) {
        return std::find(S.goalRegionIdx.begin(), S.goalRegionIdx.end(), node) != S.goalRegionIdx.end();
    };
    bool nodeGoal = isGoal(c);

    // Go formaward int he path:
    while (!nodeGoal){
        int y = findBestChild(c, S);
        if (y != -1){
            c = y;
            candidatePath.push_back(c);
            nodeGoal = isGoal(c);
        }
        // if it get to the father save it:
        else {
            S.checkedPath[c] = true;
            S.checkedPathCandidates.push_back(c);
            break;
        }
    }

    return candidatePath;
}



// FUntion to generate a path from the tree:
PathResult generatePath(RTFMTPLannerState& S){
    PathResult result;

    // Chek if goal is attached to a tree:
    auto [goalReached, idx] = isInTree(S.goalRegionIdx, S);

    // If Goal is reache create the path:
    if (goalReached){
        S.generatedPathIdx = generatePathBackward(idx, S);
        result.goal_found = true;
        result.path_found = true;
        
        for (int i : S.generatedPathIdx) {
            result.waypoints.push_back(S.V[i]);
        }
        return result;
    }

    // If goal is not reached, search for a candidate:
    std::vector<int> candidateIdx = generatePathForward(S.rootIdx, S);
    int newEnd = candidateIdx.back();
    // If there is not generated apth pass it
    if (S.generatedPathIdx.empty()) {
        S.generatedPathIdx = candidateIdx;
        result.path_found = true;
    } else {
        // If uit exist one replace it if the new path is closer to the goal regoin:
        int oldEnd = S.generatedPathIdx.back();
        if (endToGoalRegionDist(newEnd, S) < endToGoalRegionDist(oldEnd, S)) {
            S.generatedPathIdx = candidateIdx;
            result.path_found = true;
        } else {
            result.path_found = true;
        }
    }

    // If the ecnd part happen the goa is not reached pass the genrated apth and a false bool:
    result.goal_found = false;
    for (int i : S.generatedPathIdx) {
        result.waypoints.push_back(S.V[i]);
    }
    
    return result;
} 



// FUnction that works every iteration:
RTFMTPLannerState tick(FMTPlanner& rt_fmt_planner, FMTBundle& act_obs, const Eigen::VectorXd& currPose) {
    // Start by updating the Obstacles to block or unblock nodes:
    RTFMTPLannerState S = updateObstructedNodes(rt_fmt_planner, act_obs);
    rt_fmt_planner.ft_fmt_mask = S;

    //  Update the root of the tree to the current UAV pose
    if (currPose.size() > 0) {
        setCurrentPose(currPose, rt_fmt_planner, act_obs);
    }
    S = rt_fmt_planner.ft_fmt_mask;

    // Run the expansion loop based on expandTreeRate:
    int nSteps =  static_cast<int>(S.expandTreeRate);
    for (int ii = 0; ii < nSteps; ++ii){
        rewireLocally(rt_fmt_planner, S, act_obs);
        expandTree(rt_fmt_planner, S, act_obs);
        rewireFromRoot2(rt_fmt_planner, S, act_obs);
    }

    // sAVE THE EDGE LIST:
    S.E.clear();
    for (size_t i = 0; i < S.parent.size(); ++i) {
        if (S.parent[i] != -1) {
            S.E.push_back({S.parent[i], static_cast<int>(i)});
        }
    }

    // Save state back to planner
    rt_fmt_planner.ft_fmt_mask = S;
    return S;
}




// Start by defining a searching function of obstacles:
void computerFMTAvoidance(const uav_dynamics::msg::AvoidanceStates& moving_obstacles, const gnss_multipath_plugin::msg::AdsbInfo& own_state,
    double min_radius, double crit_time, FMTNavigationState& nav_state, FMTPlanner& rt_fmt_planner){
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