#include "uav_dynamics/geometric_guidance.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>


// Generate rotattion matrix in the x axis in degrees:
inline static Eigen::Matrix3d Rx_deg(double deg) {
    const double r = deg * M_PI / 180.0;
    const double c = std::cos(r), s = std::sin(r);
    Eigen::Matrix3d R;
    R << 1, 0, 0,
        0, c,-s,
        0, s, c;
    return R;
}



// Fast Detect Avoidance (FCA) Detection outputs:
struct FCADetect{
    // Index of the intruder:
    std::string aiplane_id;
    // relaive position and oeintation 
    Eigen::Vector3d rel_pos_enu;
    Eigen::Vector3d rel_vel_enu;
    // Avoidance characteristics:
    // relative angle between speed and location:
    double theta_to;
    // MInimum radius in future:
    double r_pca;
    // Minum avoidance radius:
    double dm;
};

// Structre to save the active intruders characteristics and number
struct FCABundle {
    // Add a number of FCA detection resume:
    std::vector<FCADetect> act_intruders;

    // Number of conflicts start with zero:
    double conflicts = 0;
};

// Functions used for the Geomtric AVoidance Guidance maneuver:
// ENU -> NED conversion
inline static Eigen::Vector3d ENU_to_NED(const Eigen::Vector3d &enu) {
    return Eigen::Vector3d(enu.y(), enu.x(), -enu.z()); // (N,E,-U)
}



// FUnction to find the acitve intruders for the GEomtric AVoidance maneuver:
FCABundle FCA_Detect(const uav_dynamics::msg::AvoidanceStates &intruders,double own_e, double own_n, double own_u,double own_ve, double own_vn, double own_vu){
    // Add a structure to save all teh conflicting intruders:
    FCABundle fca_bundle;
    // Obtain the number of intruders on the message:
    const size_t n = intruders.intruder_states.size();


    // Loop in each of the intruders:
    for (size_t i = 0; i<n; i++){
        // Own velocity norm:
        const double own_vel_a = std::sqrt(own_vn*own_vn+own_ve*own_ve+own_vu*own_vu);
        // Open the data of each intruder:
        const auto &intr = intruders.intruder_states[i];
        const double int_velocity_a = std::sqrt(intr.v_north*intr.v_north + intr.v_east*intr.v_east + intr.v_up*intr.v_up);
        const double dm = 1.5*(int_velocity_a*1.0+125+1.5*6+1.0*own_vel_a);
        const std::string &id = intruders.obstacles_id[i];

        // relative position and velocity:
        Eigen::Vector3d rel_pos(intr.east - own_e,
                                intr.north - own_n,
                                intr.up - own_u);

        Eigen::Vector3d rel_vel(-intr.v_east + own_ve,
                                -intr.v_north + own_vn,
                                -intr.v_up + own_vu);

        // get the relative position adn velocity norms:
        const double r_rel = rel_pos.norm();
        double v_rel = rel_vel.norm(); 

        // Get te relative angle between the relative position and velcity;
        if (v_rel < 1e-6) v_rel = 1.0;
        const double theta_to = std::acos(std::clamp(rel_pos.dot(rel_vel) / (r_rel * v_rel), -1.0, 1.0));

        // Obtian the closest future relative radius:
        const double r_pca = r_rel * std::sin(theta_to);
        // Analize if hte intruder is an active obstacle:
        if (r_pca < dm){
            ++fca_bundle.conflicts;

            FCADetect det;
            det.aiplane_id = id;
            det.rel_pos_enu = rel_pos;
            det.rel_vel_enu = rel_vel;
            det.theta_to = theta_to;
            det.r_pca = r_pca;
            det.dm = dm;

            fca_bundle.act_intruders.push_back(det);
        }

    }

    // return the dtructure witht he inforamtion of all the active obstacles:
    return fca_bundle;
}



// Estimate the targets next position:
std::vector<Eigen::Vector3d> Estimate_target_next_position(const FCABundle &intruders, const Eigen::Vector3d &act_position,
    const Eigen::Vector3d &act_vel, double crit_time){
    // Varible where the future target's positions are saved:
    std::vector<Eigen::Vector3d> future_target_pos;
    future_target_pos.reserve(intruders.act_intruders.size());

    for (const auto &o : intruders.act_intruders) {
        // Use the critical minimum time to see the estiamte the future position fo the obstacle
        future_target_pos.emplace_back(o.rel_pos_enu - crit_time * (o.rel_vel_enu-act_vel) + act_position);
    }
    return future_target_pos;

}



// Check for overlapping:
FCABundle Check_for_overalping(const FCABundle &active_obstacles, double own_e, double own_n, 
    double own_u,double own_ve, double own_vn, double own_vu){
    // Start the structure to save the overalping obstacles;
    FCABundle overlaped_obs;

    // Ownship velocity and position information:
    const Eigen::Vector3d act_pos(own_e,own_n,own_u);
    const Eigen::Vector3d act_vel(own_ve,own_vn,own_vu);
    
    // Open the active encounters:
    const auto &obstacles = active_obstacles.act_intruders;
    // Find the number of active obstacles:
    const size_t n = obstacles.size();
    if (n == 0) { overlaped_obs.conflicts = 0; return overlaped_obs; }

    // Loop between the active obstacles:
    for (size_t i = 0; i < n ; ++i){
        // Save the information of the initial position, velocity, and avoidance raius in a variable in case it changes with the overlaping:
        Eigen::Vector3d act_rel_pos = obstacles[i].rel_pos_enu;
        Eigen::Vector3d act_rel_vel = obstacles[i].rel_vel_enu;
        double act_dme = obstacles[i].dm;

        // Loop inside each of the other obstacles:
        for (size_t j=0; j < n; ++j){
            // Jump in case is hte same obstacle as the principal:
            if (j == i) continue;

            // Projected distance between the obstacle j and the actual relative position of the obstacle i
            double proj_j =  obstacles[j].rel_pos_enu.dot(act_rel_pos.normalized());
            // Vector between the the actual osbtacle i to the analized osbtacle j:
            Eigen::Vector3d rel_pos_ij =  act_rel_pos - obstacles[j].rel_pos_enu;

            // Identify if the obstacle j is in the avoidance cone of i:
            double delta_dm = obstacles[j].rel_pos_enu.norm() * std::sin(std::acos(proj_j / obstacles[j].rel_pos_enu.norm())) - proj_j * (act_dme / act_rel_pos.norm());
            bool overlap = false;
            if (proj_j <= act_rel_pos.norm()){
                overlap = (delta_dm < obstacles[j].dm);
            }else {
                overlap = (act_dme + obstacles[j].dm > rel_pos_ij.norm());
            }

            // In case they overlap combine them and update the actual aoidance zone:
            if (overlap){
                act_dme = (act_dme + rel_pos_ij.norm() + obstacles[j].dm) * 0.5;
                act_rel_pos = obstacles[j].rel_pos_enu + (rel_pos_ij.normalized()) * (act_dme - obstacles[j].dm);

                // Add the effect of the aded intruder to the obstacle in the velocity:
                double rel_ij_norm = rel_pos_ij.norm();
                if (rel_ij_norm > 1e-6) {  // or whatever epsilon you like
                    act_rel_vel += 0.2 * obstacles[j].rel_vel_enu;
                }
            }
        }

        // Change the characteristics if the avoidancw with respect to the ownship (v_rel, theta_to, r_pca, and nearest crash):
        double theta_to = std::acos(act_rel_pos.dot(act_rel_vel)/(act_rel_pos.norm()*act_rel_vel.norm()));
        double r_pca = act_rel_pos.norm() * std::sin(theta_to);

        // Save the information in the output structure after is overlaped:
        ++overlaped_obs.conflicts;
        FCADetect det;
        det.aiplane_id = obstacles[i].aiplane_id;
        det.rel_pos_enu = act_rel_pos;
        det.rel_vel_enu = act_rel_vel;
        det.theta_to = theta_to;
        det.r_pca = r_pca;
        det.dm = act_dme;

        overlaped_obs.act_intruders.push_back(det);

    }

    return overlaped_obs;
}



// Identify the risker obstacle using the critical time of avoidance and generate teh avoidance waypoints:
    void avoidance_waypoints(const FCABundle &overall_obstacles, double own_e, double own_n, double own_u,
        double own_ve, double own_vn, double own_vu, double own_course, double own_fpa,
        const std::vector<Eigen::Vector3d> &future_target_pos,double min_radius, double crit_time, NavigationState& nav_state){
    // Ownship velocity and position information:
    const Eigen::Vector3d act_pos(own_e,own_n,own_u);
    const Eigen::Vector3d act_vel(own_ve,own_vn,own_vu);
    
    // Open the active encounters:
    const auto &act_obstacles = overall_obstacles.act_intruders;
    // Find the number of active obstacles:
    const size_t n = act_obstacles.size();
    if (n == 0) { return; }

    // Variable to save the minimum critical time:
    double min_crit_time = 0;
    int count_time = 0;
    // Iidentify the critical target
    int crit_target_idx = 0;

    for (size_t i = 0; i < n ; ++i){
        // Caclulate the critical time of each target:
        double act_crit_time = (act_obstacles[i].rel_pos_enu.norm()*(std::cos(act_obstacles[i].theta_to)+std::sin(act_obstacles[i].theta_to))-min_radius-act_obstacles[i].dm)/(act_obstacles[i].rel_vel_enu.norm());
        // check if one of the airplanes that was already avoided is giving problems by staying in a negative avoidance zone:
        if (act_crit_time < -1.5){continue;}
        if (count_time == 0 || act_crit_time < min_crit_time){
            count_time = 1;
            min_crit_time = act_crit_time; 
            crit_target_idx = i;
        }
    }
    

    // Add the avoidance waypoints in case the critical time is small:
    // return in case the minimum critical time is not enough
    if (nav_state.start_the_avoidance) {
        return;
    }                   
    if (!std::isfinite(min_crit_time)||count_time==0) return;
    if (min_crit_time > crit_time) return;

    // Start the avoidance in case it is lower than the critical time:
    nav_state.start_the_avoidance = true;
    nav_state.transition_radius = 200;
    nav_state.look_ahead_distance = 150;

    // Modify the waypoints:
    // Identify the actual avoidance obstacle characteristics:
    Eigen::Vector3d avoidance_center = act_obstacles[crit_target_idx].rel_pos_enu + act_pos;
    Eigen::Vector3d crit_next_pos = future_target_pos[crit_target_idx];
    double dm_avoid = act_obstacles[crit_target_idx].dm;
    // Use the ownship inforamtion to generate the avoiance zone:
    const double own_yaw = M_PI / 2 - own_course;
    const double own_pitch = own_fpa;

    // Generate the avoidance zone:
    // Deine the normal avoidance circle normal to the velcoity of the ownship:
    int N = 50;
    // Rotation matrices:
    Eigen::Matrix3d Rz, Ry;
    Rz << std::cos(own_yaw), -std::sin(own_yaw), 0,
        std::sin(own_yaw),  std::cos(own_yaw), 0,
                0 ,        0 , 1;
    Ry << std::cos(-own_pitch), 0, std::sin(-own_pitch),
        0,                 1, 0,
        -std::sin(-own_pitch), 0, std::cos(-own_pitch);
    Eigen::Matrix3d Rotation_matrix = Rz * Ry;
    // Generate the circle around the intruder:
    std::vector<Eigen::Vector3d> circle;
    circle.reserve(N);
    for (int i = 0; i < N; i++)
    {
        double theta = 2.0 * M_PI * i / N;
        double y = std::cos(theta);
        double z = std::sin(theta);
        Eigen::Vector3d point(0.0, y, z);
        Eigen::Vector3d rotated_point = dm_avoid * Rotation_matrix * point + avoidance_center;
        circle.push_back(rotated_point);
    }



    // Identify which Waypoint is going to be at the end of the avoidance
    nav_state.avoidance_last_point_enu = avoidance_center + act_vel.normalized() * dm_avoid;
    // pass the avoidance to point from ENU to NED:
    Eigen::Vector3d avoidance_last_point_ned=  ENU_to_NED(nav_state.avoidance_last_point_enu.value());
    // Find the nearest waypoint:
    if (nav_state.waypoints.empty()) {return;}
    // Index of the nearest waypoint and actual distance to the las avoidance position:
    size_t near_wp_idx = nav_state.current_idx;
    double best_d2 = (nav_state.waypoints[near_wp_idx] - avoidance_last_point_ned).squaredNorm();

    if (near_wp_idx + 1 < nav_state.waypoints.size()) {
        for (size_t i = near_wp_idx + 1; i < nav_state.waypoints.size(); ++i) {
            const double d2 = (nav_state.waypoints[i] - avoidance_last_point_ned).squaredNorm();
            if (d2 < best_d2) {
                best_d2 = d2;
                near_wp_idx = i;
            }
        }
    }
    // Insert avoidance waypoints:
    const size_t insert_at_wp = std::min(nav_state.current_idx + 1, nav_state.waypoints.size());

    // Use a cost fucntion to identify the rigth trajectory in the avoidance zone:
    double min_cost = -1.0;
    size_t best_avoid_idx = 0;
    for (int i = 0; i < N; ++i)
    {
        double d_alt = circle[i].z() - act_pos.z();
        double d_north = circle[i].y() - act_pos.y();
        double d_east = circle[i].x() - act_pos.x();
        double horiz = std::max(std::sqrt(d_north*d_north + d_east*d_east), 1e-6);
        double d_pitch = std::atan2(d_alt, horiz);
        double d_yaw   = std::atan2(d_east, std::max(d_north, 1e-9)); // yaw-from-East

        // Get a ratio that depends on the next posiiton orientation respct to teh pincipal avoidance circle position:
        Eigen::Vector3d r_avo_fut = crit_next_pos - avoidance_center;
        Eigen::Vector3d r_avo_circ = circle[i]  - avoidance_center;
        double value_1 = std::abs(std::acos(r_avo_fut.dot(r_avo_circ)/(r_avo_fut.norm()*r_avo_circ.norm())));
        

        double cost;
        if ((d_pitch - own_pitch) < 0.0) {
            cost = ((1.0/(value_1+0.1)) + 2.4) * std::abs(d_pitch)
                + (1.0/(value_1+0.1)) * std::abs(d_yaw);
        } else {
            cost = ((1.0/(value_1+0.1)) + 1.2) * std::abs(d_pitch)
                + (1.0/(value_1+0.1)) * std::abs(d_yaw);
        }

        if (min_cost < 0.0 || cost < min_cost) {
            min_cost = cost;
            best_avoid_idx = i;
        }
    }

    // Generate the avoidance trajectory:
    Eigen::Vector3d principal_avoidance_point_enu = (circle[best_avoid_idx]-avoidance_center).normalized();
    Eigen::Vector3d own_vel_norm = act_vel.normalized();
    Eigen::Vector3d ort_norm = own_vel_norm.cross(principal_avoidance_point_enu);
    if (ort_norm.norm() < 1e-6)
    {
        Eigen::Vector3d tmp(1,0,0);
        if (std::abs(act_vel.dot(tmp)) > 0.9) tmp = Eigen::Vector3d(0,1,0);
        ort_norm = act_vel.cross(tmp);
    }
    ort_norm.normalize();
    // Regenerate principal_avoidance_point_enu in case of almost parallel rotationl axis:
    Eigen::Vector3d principal_avoidance_point_enu_new = ort_norm.cross(own_vel_norm).normalized();
    // Rotational matrix where the principal point rotates:
    Eigen::Matrix3d P;
    P.col(0) = ort_norm;
    P.col(1) = own_vel_norm;
    P.col(2) = principal_avoidance_point_enu_new;
    // Anilize if the rotation needs to go from +/- 90 deg using the closests avoidance point:
    const Eigen::Vector3d cand_p  = avoidance_center + dm_avoid * (P * Rx_deg( +90.0) * P.transpose() * principal_avoidance_point_enu_new);
    const Eigen::Vector3d cand_m  = avoidance_center + dm_avoid * (P * Rx_deg( -90.0) * P.transpose() * principal_avoidance_point_enu_new);
    const double d1 = (nav_state.avoidance_last_point_enu.value() - cand_p).squaredNorm();
    const double d2 = (nav_state.avoidance_last_point_enu.value() - cand_m).squaredNorm();
    // Generate the avodiance trajectory using the proper direction:
    std::array<double,7> phi = (d1<d2)
        ? std::array<double,7>{-60.0,-30.0,-15.0,0.0,15.0,30.0,60.0}
        : std::array<double,7>{60.0,30.0,15.0,0.0,-15.0,-30.0,-60.0};
    // Vector to save the avoidance waypoint path:
    std::vector<Eigen::Vector3d> avoidance_waypoints; avoidance_waypoints.reserve(7);
    for (double angle : phi)
    {
        Eigen::Vector3d avoid_point = dm_avoid * (P * Rx_deg(angle) * P.transpose() * principal_avoidance_point_enu_new) + avoidance_center;
        avoidance_waypoints.push_back(ENU_to_NED(avoid_point));
    }

    // Insert the 7 waypoints in the waypoint list:
    nav_state.waypoints.insert(nav_state.waypoints.begin() + static_cast<long>(insert_at_wp), avoidance_waypoints.begin(), avoidance_waypoints.end());
    // Add velocities as the akst velocity multiplied:
    const double keep_speed = (insert_at_wp < nav_state.cmd_vel.size()) ? nav_state.cmd_vel[insert_at_wp] : 0.0;
    nav_state.cmd_vel.insert(nav_state.cmd_vel.begin() + static_cast<long>(insert_at_wp), 7, keep_speed);
    const size_t arc_len = avoidance_waypoints.size(); // 7
    nav_state.end_of_arc = insert_at_wp + arc_len - 1;
    // eliminate the middle waypoints in the avoidance path;
    size_t rejoin_idx_new = near_wp_idx;
    if (near_wp_idx >= insert_at_wp) {
        rejoin_idx_new += arc_len;
    }
    if (rejoin_idx_new > nav_state.end_of_arc.value() + 1 && rejoin_idx_new <= nav_state.waypoints.size()) {
        auto wp_erase_begin = nav_state.waypoints.begin() + static_cast<long>(nav_state.end_of_arc.value() + 1);
        auto wp_erase_end   = nav_state.waypoints.begin() + static_cast<long>(rejoin_idx_new);
        nav_state.waypoints.erase(wp_erase_begin, wp_erase_end);

        // keep cmd_vel_ in sync
        if (nav_state.cmd_vel.size() >= rejoin_idx_new) {
            auto sp_erase_begin = nav_state.cmd_vel.begin() + static_cast<long>(nav_state.end_of_arc.value() + 1);
            auto sp_erase_end   = nav_state.cmd_vel.begin() + static_cast<long>(rejoin_idx_new);
            nav_state.cmd_vel.erase(sp_erase_begin, sp_erase_end);
        }
    }

    // Update the current Idx:
    nav_state.current_idx = insert_at_wp;

}



// RElaize the Geomtric AVoidance algorithm:
void computeGeometricAvoidance(const uav_dynamics::msg::AvoidanceStates& obstacles, const gnss_multipath_plugin::msg::AdsbInfo& own_state,
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

        // Save its actual position and velocit of the ownship:
        Eigen::Vector3d actual_pose;
        actual_pose << own_e, own_n, own_u;
        Eigen::Vector3d actual_vel;
        actual_vel << own_ve, own_vn, own_vu;

        // Identify the conflicts:
        FCABundle active_conflicts = FCA_Detect(obstacles, own_e, own_n, own_u, own_ve, own_vn, own_vu);

        // If there nor conflict return:
        if (active_conflicts.conflicts == 0){
            return;
        }

        // Obtain the future obstacles position:
        std::vector<Eigen::Vector3d> Target_Next_pos = Estimate_target_next_position(active_conflicts,actual_pose,actual_vel, crit_time);

        //Use a variable the save the overall active obstacles:
        FCABundle overall_obstacles;

        // Overlap obstacle that near each other or inside the minimum radius threshold
        if (active_conflicts.conflicts > 1){
            overall_obstacles = Check_for_overalping(active_conflicts, own_e, own_n, own_u, own_ve, own_vn, own_vu);
            Target_Next_pos = Estimate_target_next_position(overall_obstacles,actual_pose,actual_vel, crit_time);
        } else {
            overall_obstacles = active_conflicts;
        }

        // Calculate their critical avoidance time to generate the avoiance waypoints:
        avoidance_waypoints(overall_obstacles, own_e, own_n, own_u, own_ve, own_vn, own_vu, own_course, own_fpa, 
            Target_Next_pos, min_radius, crit_time, nav_state);

}
