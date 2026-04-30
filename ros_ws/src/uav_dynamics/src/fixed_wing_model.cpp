// Add ROS2 library
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
// C++ libraries: 
#include <string>
#include <vector>
#include <array>
#include <sstream>
#include <utility>
#include <cmath>
#include <algorithm> 
#include <numeric>
#include <optional>
#include <Eigen/Dense> 
// ROS2 Messages:
#include "uav_dynamics/msg/avoidance_states.hpp"
#include "uav_dynamics/msg/intruders_status.hpp"
#include "uav_dynamics/msg/intruder_flag.hpp"
#include "gnss_multipath_plugin/msg/states_info.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include "std_msgs/msg/int32.hpp"
// Add the library of the Fast geometric avoidance ssytem:
#include "uav_dynamics/geometric_guidance.hpp"
// Add the library of the RT-FMT guidance system:
#include "uav_dynamics/rt_fmt.hpp" 


// Start the class of the fixed-wing Dynamics;
class FixedWingDynamics : public rclcpp::Node{
    public:
        // Do the starting function:
        FixedWingDynamics(const std::string &avoider_name, const std::string &waypoints_str, 
            const std::string guidance_system, const bool active_avoidance, const bool avoidance_analysis
            ): Node(("dynamics_" + avoider_name).c_str()), avoider_name_(avoider_name), guidance_system_(guidance_system), active_avoidance_(active_avoidance), avoidance_analysis_(avoidance_analysis){

                // Qualioty of serviuce for the states messages:
                auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable).durability(rclcpp::DurabilityPolicy::Volatile);

                // Parse the avoidance waypoints:
                parse_waypoints(waypoints_str); // ---> It should have the N,E,D,vel; structure in each segment of the waypoints.

                // Define the topics:
                // Topic where the avoider publish its states using the ADS-B
                std::string avoider_topic = avoider_name_ + "/adsb_info";
                // Topic to publish teh velcoities to the movement pruggin:
                std::string cmd_vel_topic = avoider_name_ + "/cmd_vel";
                // Topic to say that the trajectory is completed.
                std::string complete_topic = "/traj_complete";
                // Topiuic wher ethe obtacles are published for each uav:
                std::string obstacles_topic = avoider_name_ + "/obstacles_adsb";
                // Topc to see when the simulation is restatrded
                std::string mission_start_topc = "/mission_starts";

                // Topic for the publishers of when the avoidance occurs, and if inside an avodiacne region:
                std::string is_inside_avoidance_topic = avoider_name_ + "/is_inside_avo";
                std::string is_avoiding_topic = avoider_name_ + "/is_avoiding";

                // Specify the vearibles used for the differnt Guidance ALgorithms:
                if (active_avoidance_ && guidance_system_ == "GEOMETRIC") {
                    avoidance_vars_geom_ = GeometricAvoidanceVars{
                        180, // min_radius of turn (m)
                        0.0000 // critical avoidance time (s)
                    };

                    RCLCPP_INFO(this->get_logger(), "Geometric avoidance system created.");
                } else if (active_avoidance_ && guidance_system_ == "FMT"){
                    // Define hte characteristis of the rt_fmt:
                    rt_fmt_planner_ = FMTPlanner();
                    // NUmber of nodes cacualted with a distance we wnat them to be separed average:
                    rt_fmt_planner_->rt_fmt_opts.N = 1000;
                    // Weights of the cost functio w1 (dist) and w2 (heading):
                    rt_fmt_planner_->rt_fmt_opts.w1 = 0.25;
                    rt_fmt_planner_->rt_fmt_opts.w2 = 50.0;
                    // Exapansion rate:
                    rt_fmt_planner_->rt_fmt_opts.expandTreeRate = 10.0;
                    // Gaol radius that it needs to be into:
                    rt_fmt_planner_->rt_fmt_opts.goal_radius = 50.0;
                    // SAFE radius to be from the obstacle:
                    rt_fmt_planner_->rt_fmt_opts.safeRadiusDObstacle = 20;

                    // If the avdoiance maneuver is the FMT system start by defining the configruation variables:
                    limits_ = get_trajectory_limits(waypoints_);
                    // Map of the world, for the moment is not used:
                    std::vector<int> map; 
                    // Get teh starting and goal points from waypoints:
                    auto [start_, goal_] = GetCharacteristicPoints(waypoints_);
                    // Deifne the airspeed from teh mean airspeed fo the trajctory:
                    mean_air_speed_ = get_mean(cmd_vel_);
                    // Searching radius:
                    double rn = mean_distance_nodes(limits_, rt_fmt_planner_->rt_fmt_opts.N);
                    rn = rn * 5.0;
                    
                    // Generate the RT-FMT planner system:
                    start_rt_fmt(map, limits_, start_, goal_, rn, rt_fmt_planner_.value(), max_roll_, mean_air_speed_, fpa_limits_);
                    
                    RCLCPP_INFO(this->get_logger(), "FMT avoidance system created.");
                }

                // Subscriber to the states of the avoider:
                avoider_states_sub_ = this->create_subscription<gnss_multipath_plugin::msg::StatesInfo>(
                    avoider_topic, qos, std::bind(&FixedWingDynamics::avoider_states_callback, this, std::placeholders::_1));
                
                // SUbscribe to the mission start to start all tehe avoidance:
                auto qos_miss = rclcpp::QoS(rclcpp::KeepLast(1)).reliability(rclcpp::ReliabilityPolicy::Reliable).durability(rclcpp::DurabilityPolicy::TransientLocal);
                mission_start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                    mission_start_topc, qos_miss, std::bind(&FixedWingDynamics::mission_start_callback, this, std::placeholders::_1)
                );
                
                if (active_avoidance_){
                    // Subscribe to the obstacles states:
                    obstacles_states_sub_ = this->create_subscription<uav_dynamics::msg::AvoidanceStates>(
                        obstacles_topic, qos, std::bind(&FixedWingDynamics::obstacle_states_callback, this, std::placeholders::_1));

                    // If the avoidance analysis is necesary add publishers of when the avoidance occurs, and is the avoidance goes against any intruder:
                    if (avoidance_analysis_){
                        // Publisher of the int vector showing if is inside an avodiance zone:
                        is_inside_avo_pub_ = this->create_publisher<uav_dynamics::msg::IntrudersStatus>(is_inside_avoidance_topic, qos);
                        // Publish if the sytem is in an avoiding maneuver:
                        is_avoding_pub_ = this->create_publisher<std_msgs::msg::Int32>(is_avoiding_topic, qos);
                    }
                }
                // Publisher to the command velcoity and use the plugin movement:
                cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, qos);
                // Publisher to define if the trajecotry of any of the UAVs is completed:
                complete_encounter_pub_ = this->create_publisher<std_msgs::msg::Bool>(complete_topic, qos);
                // Subscriber to define if any other UAV complete its trajecotry:
                complete_encounter_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                    complete_topic, qos, std::bind(&FixedWingDynamics::completeEncounterCallback, this, std::placeholders::_1)
                );
        }

        // Function to say when goal is reached:
        bool is_goal_reached() const { return goal_reached_; }
    
    private:
        // Define the avoider_name of the model:
        std::string avoider_name_;
        // DEfine if the avoidance is active and the guidance technique:
        std::string guidance_system_;
        bool active_avoidance_;
        bool avoidance_analysis_;


        // Define the wripoints vector adn the last waypoint:
        std::vector<Eigen::Vector3d> waypoints_;
        Eigen::Vector3d last_waypoint_;
        // Save the velocity that the waypoints want to input in a vector:
        std::vector<double> cmd_vel_;

        // Define the vraibles of the Waypoint follower to see the next position:
        double transition_radius_;
        double look_ahead_distance_;

        // Define the avoider current state:
        gnss_multipath_plugin::msg::StatesInfo avoider_current_state_;

        // Re-initialize the waypoint index changing the MIT encounter set:
        size_t current_idx = 0;
        // Iteration counter:
        int count = 0;
        
        // Generate a boolean to start the following process with the service:
        bool start_following = false;

        // Deifine the ROS2 subscribers and publishers:
        rclcpp::Subscription<gnss_multipath_plugin::msg::StatesInfo>::SharedPtr avoider_states_sub_;
        rclcpp::Subscription<uav_dynamics::msg::AvoidanceStates>::SharedPtr obstacles_states_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr complete_encounter_pub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr complete_encounter_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_start_sub_;
        rclcpp::Publisher<uav_dynamics::msg::IntrudersStatus>::SharedPtr is_inside_avo_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr is_avoding_pub_;

        // Variable to see when the UAV reaches the last waypoint:
        bool goal_reached_ = false;
        bool last_published_goal_reached_ = false;

        // Define the Characteristis of the Guidance systems in structs:
        // Define if the avoidance is started:
        bool start_the_avoidance = false;
        // Define the avoidance last point: 
        std::optional<Eigen::Vector3d> avoidance_last_point_enu;
        // ENd of avoidance arc made for avoidance:
        std::optional<size_t> end_of_arc_ = 0; 
        // UAV chracteristics
        double min_radius_;
        double max_roll_ = 35.0/57.3;
        // FGAC characteristics:
        struct GeometricAvoidanceVars {
            double min_radius;
            double crit_time;
        };
        std::optional<GeometricAvoidanceVars> avoidance_vars_geom_;


        // Optional variable that the needed to cerate teh rt-fmt planner:
        // Lmits of the planner:
        std::vector<std::vector<double>> limits_;
        // Objective points:
        std::vector<double> start_;
        std::vector<double> goal_;
        // RT-FMT Planner:
        std::optional<FMTPlanner> rt_fmt_planner_;
        // Uav parameters (that are not defined alrady):
        double fpa_limits_[2] = {-0.3, 0.3};
        // Mean airspeed:
        double mean_air_speed_;

        // Start the vector where is goign to be defined if enter inside the avoidance zone:
        std::vector<std::pair<std::string, int>> is_inside_avoidance_zone_;

        



        ///////////////////////// FUNCTIONS //////////////////////////////
        // Parse the waypoints from a string to matrices of doubles to forloop in them:
        void parse_waypoints(const std::string &waypoints_str){
            std::stringstream ss(waypoints_str);
            std::string segment;

            // loop to separe teh waypoints:
            while (std::getline(ss, segment, ';')){
                std::stringstream coords(segment);
                std::string val;

                // place to save the waypoints and commanded speed
                Eigen::Vector3d wp;
                double vel = 0.0;
                int i = 0;
                while (std::getline(coords, val, ',')){
                    double num = std::stod(val);
                    if (i < 3) {
                        wp(i) = num;
                    } else if (i == 3) {
                        vel = num;
                    }
                    i++;
                }
                // Store it in the matrices:
                waypoints_.push_back(wp);
                cmd_vel_.push_back(vel);
            }

            // Save the last waypoint
            if (!waypoints_.empty()){
                last_waypoint_ = waypoints_.back();
            }
        }



        // Defien the limits from the current waypoints:
        std::vector<std::vector<double>> get_trajectory_limits(const std::vector<Eigen::Vector3d> &Waypoints){
            std::vector<std::vector<double>> limits_way;
            if (Waypoints.empty()) return limits_way;

            // FInd the maximum and minimum N, E and D limits and add some in istance in the vertical axis:
            auto n_lim = std::minmax_element(Waypoints.begin(), Waypoints.end(),
                [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) { return a.x() < b.x(); });
            auto e_lim = std::minmax_element(Waypoints.begin(), Waypoints.end(),
                [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) { return a.y() < b.y(); });
            auto d_lim = std::minmax_element(Waypoints.begin(), Waypoints.end(),
                [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) { return a.z() < b.z(); });

            limits_way = {{n_lim.first->x()-400, n_lim.second->x()+400}, {e_lim.first->y()-400, e_lim.second->y()+400}, {d_lim.first->z()-20, d_lim.second->z()+20}};

            return limits_way;
        }



        // Function to get teh strat and the goal from the waypoints:
        std::pair<std::vector<double>, std::vector<double>> GetCharacteristicPoints(const std::vector<Eigen::Vector3d> &Waypoints){
            // DEfine the vectors:
            std::vector<double> start_pt(4, 0.0);
            std::vector<double> goal_pt(4, 0.0);

            // Safety check:
            if (Waypoints.empty()) {
                return {start_pt, goal_pt};
            }
            if (Waypoints.size() == 1) {
                start_pt = {Waypoints[0].x(), Waypoints[0].y(), Waypoints[0].z(), 0.0};
                return {start_pt, start_pt};
            }

            // Calcualte the headings:
            for (size_t i = 0; i < Waypoints.size(); i++){
                // For teh starting and final waypoint:
                if (i == 0){
                    Eigen::Vector3d next_wp = Waypoints[i+1];
                    Eigen::Vector3d curr_wp = Waypoints[i];
                    // Get the yaw values:
                    double dx = next_wp.x() - curr_wp.x();
                    double dy = next_wp.y() - curr_wp.y();
                    double yaw = std::atan2(dy, dx);
                    // Save teh starting point:
                    start_pt = {curr_wp.x(), curr_wp.y(), curr_wp.z(), yaw};

                if (yaw < 0) yaw += 2.0 * M_PI;
                } else if (i ==  Waypoints.size() - 1 ){
                    Eigen::Vector3d next_wp = Waypoints[i];
                    Eigen::Vector3d curr_wp = Waypoints[i-1];
                    // Get the yaw values:
                    double dx = next_wp.x() - curr_wp.x();
                    double dy = next_wp.y() - curr_wp.y();
                    double yaw = std::atan2(dy, dx);
                    if (yaw < 0) yaw += 2.0 * M_PI;
                    // Save teh starting point:
                    goal_pt = {next_wp.x(), next_wp.y(), next_wp.z(), yaw};
                } else {
                    continue;
                }   
            }

            // Return the pair of vectors cleanly
            return {start_pt, goal_pt};
        }



        // Function to get the mean od a vector:
        double get_mean(std::vector<double>& vec_i){
            if (vec_i.empty()){
                return 0.0;
            }

            double sum = std::accumulate(vec_i.begin(), vec_i.end(), 0.0);
            double mean = sum / vec_i.size();
            return mean;
        }



        // Funvtion to get the average ditance radius of separation between nodes using Newton-Raphson:
        double mean_distance_nodes(std::vector<std::vector<double>> limits, double N){
            // Defoine the lengths of the limits:
            double l1 = limits[0][1]-limits[0][0];
            double l2 = limits[1][1]-limits[1][0];
            double l3 = limits[2][1]-limits[2][0];

            // Newton rhapson variables:
            double volume = l1 * l2 * l3;
            double x0 = std::cbrt(volume / N); 
            double x1 = 0.0;
            int iter = 0;

            // DO Newton rahson:
            while (iter < 20){
                double f_x = (l1 + x0) * (l2 + x0) * (l3 + x0) - N * std::pow(x0, 3);
                double df_x = (l1 * l2 + l1 * l3 + l2 * l3) 
                              + 2.0 * x0 * (l1 + l2 + l3) 
                              + 3.0 * std::pow(x0, 2) 
                              - 3.0 * N * std::pow(x0, 2);

                // Break if the difference si small:
                if (std::abs(df_x) < 1e-9) break;

                // Update:
                x1 = x0 - (f_x / df_x);
                if (std::abs(x0-x1) < 1e-4) break;
                x0 = x1;
                iter ++;
            }
            return x1;
        }



        // Define the NextGoalInformation
        struct NextGoalInformation{
            double course_cmd;
            Eigen::Vector3d look_ahead_pos;
            int act_idx;
        };



        // STrat the avoidance manuver or the movement:
        void mission_start_callback(const std_msgs::msg::Bool::SharedPtr msg){
            if (msg->data){
                start_following = true;
            } else {
                return;
            }
        }



        // Define the Waypoint Follower using the HYperplane logic:
        NextGoalInformation WaypointFollower(const Eigen::Vector3d &pose, const std::vector<Eigen::Vector3d> &Waypoints, double look_ahead_distance,
                                    int init_idx, double transition_radius){
            // Pass the position to be in the NED axes:
            Eigen::Vector3d mod_pose = pose;
            mod_pose(2) = -mod_pose(2);

            // Find the Actual idx in which the Airplane is flying using teh Waypoint Hyperplane ondition:
            int max_idx = static_cast<int>(Waypoints.size()) - 2;
            int act_idx = init_idx;
            for (int i = init_idx; i < max_idx; i++)
            {
                if ((mod_pose - Waypoints[i]).dot(Waypoints[i + 1] - Waypoints[i]) < 0){
                    act_idx = i;
                    if (act_idx >= max_idx)
                        act_idx = max_idx;
                    break;
                }
            }
            // Check if the position is at near the next waypoint using the transition radius:
            if (act_idx < max_idx && (mod_pose - Waypoints[act_idx + 1]).norm() <= transition_radius)
            {
                act_idx++;
            }

            // Obtain the unitary vector that defines the direction from the pass waypoin tot hte next waypoint:
            Eigen::Vector3d past_goal = Waypoints[act_idx];
            Eigen::Vector3d goal = Waypoints[act_idx + 1];
            Eigen::Vector3d path_vec = goal - past_goal;
            const double seg_len = path_vec.norm();
            if (seg_len < 1e-6){
                path_vec = Eigen::Vector3d(1.0, 0.0, 0.0);
            } else{
                path_vec /= seg_len;
            }

            // Generate the projected pose on the path_vec trjectory and adding the look ahead distance:
            Eigen::Vector3d past_2_pos = mod_pose - past_goal;
            double proj_dist = past_2_pos.dot(path_vec);
            Eigen::Vector3d proj_pose = past_goal + path_vec * proj_dist;
            Eigen::Vector3d look_ahead_pos = proj_pose + look_ahead_distance * path_vec;

            // Identify the commanded course using the North and East poistion of the look ahead position and the actual position:
            Eigen::Vector3d delta = look_ahead_pos - mod_pose;
            double course_cmd = std::atan2(delta(1),delta(0));

            return {course_cmd, look_ahead_pos, act_idx};
        }


        
        // Define the small UAV-fixed wing autopilot model:
        Eigen::VectorXd FixedWingLogic(const Eigen::VectorXd &state, double vel_cmd, double course_cmd, double alt_cmd){
            // Define the constant values
            const double g = 9.81;   // gravity
            const double kp_V = 0.6; // proportional gain of the velocity
            const double kp_roll = 1.1; //proportional gain of the roll
            const double kd_roll = 1.8; //derivatice gain of the roll
            const double kp_Y = 0.5; // proportional gain of the course
            const double kp_h = 0.4; // proportional gain of the velocity
            const double kp_heading = 0.4; // proportional gain of the heading
            
            // Start teh derivative vector:
            Eigen::VectorXd dstate(8);

            // Define the velocity:
            double V = state(5);
            // Check the velocity and add aminimum vsalue in case is zero:
            if (!std::isfinite(V) || V < 1.0) {
                V = 1.0;
            }

            // Obtain the commanded roll:
            double roll_cmd = atan2(kp_heading * angdiff(course_cmd,state(3)) * V, g);
            // Limit the commanded roll:
            roll_cmd = std::clamp(roll_cmd, -max_roll_, max_roll_);

            // Obtian the FPA command:
            double alt_diff = kp_h * (alt_cmd - state(2));
            alt_diff = std::clamp(alt_diff, -V, V);
            double Y_cmd = std::clamp(std::asin((1.0 / V) * alt_diff), -25.0 * (M_PI / 180.0), 25.0 * (M_PI / 180.0));

            // Genate the derivative vector:
            dstate(0) = V * cos(state(3)) * cos(state(4));
            dstate(1) = V * sin(state(3)) * cos(state(4));
            dstate(2) = V * sin(state(4));
            dstate(3) = (g * tan(state(6))) / V;
            dstate(4) = kp_Y * (Y_cmd - state(4));
            dstate(5) = kp_V * (vel_cmd - V);
            dstate(7) = kp_roll * (roll_cmd - state(6)) - kd_roll * state(7);
            dstate(6) = 0.1 * dstate(7) + state(7);\

            // Establish some limits:
            dstate(4) = std::clamp(dstate(4), -3.0 * (M_PI / 180.0), 3.0 * (M_PI / 180.0));  // pitch rate
            dstate(3) = std::clamp(dstate(3), -6.0 * (M_PI / 180.0), 6.0 * (M_PI / 180.0));  // heading rate
            dstate(6) = std::clamp(dstate(6), -30.0 * (M_PI / 180.0), 30.0 * (M_PI / 180.0)); // roll rate

            return dstate;
        }



        // ENU -> NED conversion
        inline static Eigen::Vector3d ENU_to_NED(const Eigen::Vector3d &enu) {
            return Eigen::Vector3d(enu.y(), enu.x(), -enu.z()); // (N,E,-U)
        }



        // Avoider ads-b message callback:
        void avoider_states_callback(const gnss_multipath_plugin::msg::StatesInfo::SharedPtr msg){
            // Save the actual avoider state:
            avoider_current_state_ = *msg;

            // Only start if the Path follower its started:
            if (!start_following){
                return;
            }

            // Apply teh intial veclocity if the mdoel is nto moving:
            const double velocity_a = std::sqrt(msg->v_north * msg->v_north + msg->v_east * msg->v_east + msg->v_up * msg->v_up);
            if (velocity_a < 5.0){
                geometry_msgs::msg::Twist v0;
                const double v_init = std::max(cmd_vel_.empty() ? 5.0 : cmd_vel_.front(), 5.0);
                v0.linear.x = v_init;
                v0.angular.x = 0.0; v0.angular.y = 0.0; v0.angular.z = 0.0;
                cmd_vel_pub_->publish(v0);
                return;
            }

            // Change the order of the states information:
            // Todefine the actual position
            Eigen::Vector3d actual_pose;
            actual_pose << msg->north, msg->east, msg->up;

            // Define a vector as the actual state:
            Eigen::VectorXd actual_state(8);
            actual_state << msg->north, msg->east, msg->up, msg->course, msg-> fpa, velocity_a, msg-> roll, msg->p;

            // Calacualte the lookahed aditsnte and the transitionr adius:
            double current_speed = std::max(velocity_a, 1.0);
            double current_min_radius = 2.0 * (current_speed * current_speed) / (std::tan(max_roll_) * 9.81);
            if (!start_the_avoidance) {
                transition_radius_ = std::max(100.0, current_min_radius * 2.0); 
                look_ahead_distance_ = std::max(20.0, current_speed * 3.0); 
            }

            // USe the Waypoint follower to obtain teh next waypoint to follow:
            NextGoalInformation next_goal = WaypointFollower(actual_pose, waypoints_, look_ahead_distance_, current_idx, transition_radius_);
            // Update the index:
            current_idx = next_goal.act_idx;

            // Define the important information from the goal position:
            double vel_cmd = cmd_vel_[current_idx];
            double course_cmd = next_goal.course_cmd;
            double altitude_cmd = -next_goal.look_ahead_pos(2);

            // Use the Small UAV dynamics:
            Eigen::VectorXd actual_dstates = FixedWingLogic(actual_state, vel_cmd, course_cmd, altitude_cmd);

            // Add more value to teh counter:
            count = count +1;

            // Publish the velocity:
            geometry_msgs::msg::Twist cmd_velocity;
            
            // Check if it gets near the goal:
            Eigen::Vector3d act_mod_pose = actual_pose;
            act_mod_pose(2) = -act_mod_pose(2); 

            // Identify if the avoidance maneuver is compelted and restard the following varibales in case is required:
            if (active_avoidance_){
                // Calculate the minimum radius using the UAV characteristisc:
                min_radius_ = velocity_a * velocity_a / (std::tan(max_roll_)  * 9.81);

                if (guidance_system_ == "GEOMETRIC"){
                    // Add the minimum raduis to the avodance parameters.
                    avoidance_vars_geom_->min_radius = min_radius_;
                    if (start_the_avoidance) {
                        const Eigen::Vector3d avoidance_last_point_ned = ENU_to_NED(avoidance_last_point_enu.value()); // (N,E,-U)

                        if (((act_mod_pose - avoidance_last_point_ned).norm() <= 100.0) || current_idx >= end_of_arc_) {
                            start_the_avoidance = false;

                            // restore thresholds/params      
                            transition_radius_ = std::max(100.0, current_min_radius * 2.0);    
                            look_ahead_distance_ = std::max(20.0, current_speed * 0.5);

                            RCLCPP_INFO(this->get_logger(), "Avoidance complete; resuming nominal path.");
                        }
                    }
                }

                // Use teh FMT:
                if (guidance_system_ == "FMT"){
                    // reduce the lookahead radius and the trnasition radius to make it more maneuverable
                    transition_radius_ = std::max(100.0, current_min_radius * 2.0);    
                    look_ahead_distance_ = std::max(20.0, current_speed * 0.5);

                    // NOTE: Because teh RT-FMT qorks based on teh goal after it starts working it is not freaseble to ave a last point of avoidance.
                }
            }

            // Do stop system to stop if is near teh last waypoint:
            if ((act_mod_pose - last_waypoint_).norm() <= 300){

                // If teh avoider is near the goal make it stop:
                if (active_avoidance_) {
                    goal_reached_ = true;
                    // send a zero speed before starting a new trajectory
                    RCLCPP_INFO(this->get_logger(), "GOAL REACHED !!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                    cmd_velocity.linear.x = 0;
                    cmd_velocity.angular.x = 0;
                    cmd_velocity.angular.y = 0;
                    cmd_velocity.angular.z = 0;
                    // publish it:
                    for (int i = 0; i < 3; ++i) {
                        cmd_vel_pub_->publish(cmd_velocity);
                        rclcpp::sleep_for(std::chrono::milliseconds(100));
                    }

                    // Publish to the other UAVs that the encounter is completed:
                    if (goal_reached_ != last_published_goal_reached_) {
                        std_msgs::msg::Bool finish_follow;
                        finish_follow.data = goal_reached_;
                        complete_encounter_pub_->publish(finish_follow);
                        last_published_goal_reached_ = goal_reached_;
                    }
                } else {
                    // For intruders continue flying:
                    goal_reached_ = false;
                    // Define cruise conditions:
                    double cruise_speed = cmd_vel_.back();
                    double course_cmd_cruise = actual_state(3); 
                    double alt_cmd_cruise = actual_state(2);
                    Eigen::VectorXd cruise_dstates = FixedWingLogic(actual_state, cruise_speed, course_cmd_cruise, alt_cmd_cruise);

                    cmd_velocity.linear.x = cruise_speed;
                    cmd_velocity.angular.x = cruise_dstates(6);
                    cmd_velocity.angular.y = -cruise_dstates(4);
                    cmd_velocity.angular.z = -cruise_dstates(3);

                    cmd_vel_pub_->publish(cmd_velocity);
                }
            } else {
                goal_reached_ = false;
                // Send the required velcoity command:
                double com_linear_vel = std::clamp(vel_cmd, 5.0, 420.0);
                cmd_velocity.linear.x = com_linear_vel;
                // Pass the angular velcoites in teh Earth RF to body angular veloicites commands:
                double phi   = actual_state(6); 
                double theta = actual_state(4);
                double dot_phi   = actual_dstates(6);
                double dot_theta = actual_dstates(4); 
                double dot_psi   = actual_dstates(3);
                double p = dot_phi - dot_psi * std::sin(theta);
                double q = dot_theta * std::cos(phi) + dot_psi * std::cos(theta) * std::sin(phi);
                double r = -dot_theta * std::sin(phi) + dot_psi * std::cos(theta) * std::cos(phi);
                cmd_velocity.angular.x = p;
                cmd_velocity.angular.y = -q; 
                cmd_velocity.angular.z = -r;
                // publish it:
                cmd_vel_pub_->publish(cmd_velocity);
            }

        }



        // Callback that is called when a Obstacle message is recieved:
        void obstacle_states_callback(const uav_dynamics::msg::AvoidanceStates::SharedPtr msg){
            // Always start it:
            is_inside_avoidance_zone_.clear();
            for (const auto& id : msg->obstacles_id) {
                is_inside_avoidance_zone_.push_back({id, 0});
            }

            // Call the guidance logic:
            if (guidance_system_ == "GEOMETRIC" && avoidance_vars_geom_.has_value()) {
                NavigationState nav_state{
                    waypoints_, 
                    cmd_vel_, 
                    current_idx, 
                    transition_radius_,
                    look_ahead_distance_, 
                    start_the_avoidance,
                    avoidance_last_point_enu, 
                    end_of_arc_,
                    is_inside_avoidance_zone_
                };

                computeGeometricAvoidance(
                    *msg, 
                    avoider_current_state_, 
                    avoidance_vars_geom_->min_radius, 
                    avoidance_vars_geom_->crit_time, 
                    nav_state
                );
            } else if (guidance_system_ == "FMT" ) {
                FMTNavigationState nav_state_fmt{
                    waypoints_,
                    cmd_vel_,
                    current_idx,
                    transition_radius_,
                    look_ahead_distance_, 
                    start_the_avoidance,
                    is_inside_avoidance_zone_
                };

                computerFMTAvoidance(
                    *msg,
                    avoider_current_state_, 
                    min_radius_, 
                    0.3, 
                    nav_state_fmt,
                    rt_fmt_planner_.value()
                );
            }

            // Publish the avoidance analysis if its required:
            if (avoidance_analysis_) {
                // Create the individual messgae for all obstacle:
                uav_dynamics::msg::IntrudersStatus inside_msg;
                // Loop inside each obtacle:
                for (const auto& entry : is_inside_avoidance_zone_) {
                    // Create if or each obstacle
                    uav_dynamics::msg::IntruderFlag flag_msg;
                    flag_msg.obstacle_id = entry.first;
                    flag_msg.is_inside = entry.second;
                    inside_msg.states.push_back(flag_msg); 
                }

                // Publish the oevrall measge for al teh obstacle:
                is_inside_avo_pub_->publish(inside_msg);

                // Check when is avoiding:
                std_msgs::msg::Int32 avoiding_msg;
                avoiding_msg.data = start_the_avoidance ? 1 : 0;
                is_avoding_pub_->publish(avoiding_msg);
            }

        }



        // Callback to stop the simulation in case other airplane comeplete the encounter;
        void completeEncounterCallback(const std_msgs::msg::Bool::SharedPtr msg)
        {
            // Check your own airplane already reaches the goal:
            if (!msg->data || goal_reached_) return;
            RCLCPP_INFO(this->get_logger(), "Another airplane completed the encounter!");
            geometry_msgs::msg::Twist cmd_velocity;
            RCLCPP_INFO(this->get_logger(), "GOAL REACHED !!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            for (int i = 0; i < 3; ++i) {
                cmd_vel_pub_->publish(cmd_velocity);
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
            // Optional: stop this follower
            goal_reached_ = true;
            std_msgs::msg::Bool finish_follow;
            finish_follow.data = goal_reached_;
        } 



        // generate an absolute angle difference calcualtion:
        inline double angdiff(double a, double b) {
            // define values of and b in teh range 0 to 2pi
            double abs_a = a;
            while (abs_a > 2.0 * M_PI) abs_a -= 2.0 * M_PI;
            while (abs_a < 0.0) abs_a += 2.0 * M_PI;
            double abs_b = b;
            while (abs_b > 2.0 * M_PI) abs_b -= 2.0 * M_PI;
            while (abs_b < 0.0) abs_b += 2.0 * M_PI;
            // obtain the difference
            double diff = abs_a - abs_b;
            if (diff > M_PI)  diff -= 2.0 * M_PI;
            if (diff < -M_PI) diff += 2.0 * M_PI;
            return diff;
        }
};




// Main logic of the Executable:
int main(int argc, char **argv){
    // Start the ROS 2 node:
    rclcpp::init(argc, argv);

    // Safety check that all the arguments are being inputted:
    if (argc < 6)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                     "Usage: ros2 run <package_name> <executable_name> <avoider_name> <waypoints_string> <guidance_system> <active_avoidance (0 or 1)> <avoidance_analysis (0 or 1)>");
        return 1;
    }

    // Obtain the arguments from the command line:
    std::string avoider_name = argv[1];
    std::string waypoints_str = argv[2];
    std::string guidance_system = argv[3];
    bool active_avoidance = (std::stoi(argv[4]) != 0); 
    bool avoidance_analysis = (std::stoi(argv[5]) != 0); 

    // Start the node with teh class:
    auto node = std::make_shared<FixedWingDynamics>(avoider_name, waypoints_str, guidance_system, active_avoidance, avoidance_analysis);

    // SPin the node:
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    while (rclcpp::ok() && !node->is_goal_reached()){
        exec.spin_once();
    }
    rclcpp::sleep_for(std::chrono::milliseconds(200));

    // Shutdown ROS 2:
    rclcpp::shutdown();
    return 0;
}