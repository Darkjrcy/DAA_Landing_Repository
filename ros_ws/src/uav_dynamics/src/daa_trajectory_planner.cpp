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
#include <mutex>
#include <algorithm> 
#include <numeric>
#include <optional>
#include <Eigen/Dense> 
// ROS2 Messages:
#include <std_msgs/msg/bool.hpp>
#include "std_msgs/msg/int32.hpp"
// Messages to be used to generate the DAA logic:
#include "uav_dynamics/msg/avoidance_states.hpp"
#include "uav_dynamics/msg/intruders_status.hpp"
#include "uav_dynamics/msg/intruders_status.hpp"
#include "uav_dynamics/msg/intruder_flag.hpp"
#include "uav_dynamics/msg/waypoint.hpp"
#include "uav_dynamics/msg/avoidance_path.hpp"
// Add the library of the Fast geometric avoidance ssytem:
#include "uav_dynamics/geometric_guidance.hpp"
// Add the library of the RT-FMT guidance system:
#include "uav_dynamics/rt_fmt.hpp" 



// Start the class of the DAA_trajecotry_planner:
class DAATrajectoryPlanner : public rclcpp::Node{
    public:
        // Create the starting function:
        DAATrajectoryPlanner(const std::string &avoider_name, const std::string &waypoints_str, 
         const std::string guidance_system, const bool avoidance_analysis
        ) : Node(("daa_trajectory_planner_" + avoider_name).c_str()), avoider_name_(avoider_name), guidance_system_(guidance_system), avoidance_analysis_(avoidance_analysis){

            // Qualioty of serviuce for the states messages:
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable).durability(rclcpp::DurabilityPolicy::Volatile);

            // Pass the waypoint string to a vector:
            parse_waypoints(waypoints_str); 

            // Define the topics:
            // Topic where the avoider publish its states using the ADS-B
            std::string avoider_topic = avoider_name_ + "/adsb_info";
            // Topiuic wher ethe obtacles are published for each uav:
            std::string obstacles_topic = avoider_name_ + "/obstacles_adsb";
            // Topc to see when the simulation is restatrded
            std::string mission_start_topc = "/mission_starts";
            // Topic to publish the waypoitns to the kinematics:
            std::string waypoints_topic = avoider_name_ + "/daa_waypoints";
            // Define the topic for the current idx:
            std::string current_idx_topic = avoider_name_ + "/current_idx";

            // sTART THE Idx counter:
            current_idx = 0;

            // Define the characteristics for the different Guidence Algotihms:
            if (guidance_system_ == "GEOMETRIC") {
                crit_time_ = 0.0;
                current_min_radius_ = 180;

                RCLCPP_INFO(this->get_logger(), "Geometric avoidance system created.");
            } else if (guidance_system_ == "FMT"){
                // Define hte characteristis of the rt_fmt:
                rt_fmt_planner_ = FMTPlanner();
                // NUmber of nodes cacualted with a distance we wnat them to be separed average:
                rt_fmt_planner_->rt_fmt_opts.N = 10000;
                // Weights of the cost functio w1 (dist) and w2 (heading):
                rt_fmt_planner_->rt_fmt_opts.w1 = 0.25;
                rt_fmt_planner_->rt_fmt_opts.w2 = 50.0;
                // Exapansion rate:
                rt_fmt_planner_->rt_fmt_opts.expandTreeRate = 100.0;
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

            // Subscribe to the states of the ownship:
            ownship_states_sub_ = this->create_subscription<gnss_multipath_plugin::msg::StatesInfo>(
                    avoider_topic, qos, std::bind(&DAATrajectoryPlanner::ownship_states_callback, this, std::placeholders::_1));

            // Subscribe to the mission start to start all tehe avoidance:
            auto qos_miss = rclcpp::QoS(rclcpp::KeepLast(1)).reliability(rclcpp::ReliabilityPolicy::Reliable).durability(rclcpp::DurabilityPolicy::TransientLocal);
            mission_start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                mission_start_topc, qos_miss, std::bind(&DAATrajectoryPlanner::start_mission_callback, this, std::placeholders::_1)
            );

            // Subscribe to the osbtacles:
            obstacles_states_sub_ = this->create_subscription<uav_dynamics::msg::AvoidanceStates>(
                obstacles_topic, qos, std::bind(&DAATrajectoryPlanner::obstacle_states_callback, this, std::placeholders::_1));

            // Subscribe to teh current_idx topic:
            current_idx_sub_ = this->create_subscription<std_msgs::msg::Int32>(
                current_idx_topic, qos, std::bind(&DAATrajectoryPlanner::current_idx_callback, this, std::placeholders::_1));

            // Pubvlsiher of the waypoitns:
            waypoints_pub_ = this->create_publisher<uav_dynamics::msg::AvoidancePath>(waypoints_topic, qos);

            // If the avoidance analysis is necesary add publishers of when the avoidance occurs, and is the avoidance goes against any intruder:
            if (avoidance_analysis_){
                // Topic for the publishers of when the avoidance occurs, and if inside an avodiacne region:
                std::string is_inside_avoidance_topic = avoider_name_ + "/is_inside_avo";
                std::string is_avoiding_topic = avoider_name_ + "/is_avoiding";

                // Publisher of the int vector showing if is inside an avodiance zone:
                is_inside_avo_pub_ = this->create_publisher<uav_dynamics::msg::IntrudersStatus>(is_inside_avoidance_topic, qos);
                // Publish if the sytem is in an avoiding maneuver:
                is_avoiding_pub_ = this->create_publisher<std_msgs::msg::Int32>(is_avoiding_topic, qos);
            }

        }          

    private:
        // Define the avoider_name of the model:
        std::string avoider_name_;
        // DEfine if the avoidance is active and the guidance technique:
        std::string guidance_system_;
        // Define if you are realizing the avoidance analysis:
        bool avoidance_analysis_;

        // Define the wripoints vector adn the last waypoint:
        std::vector<Eigen::Vector3d> waypoints_;
        Eigen::Vector3d last_waypoint_;
        // Save the velocity that the waypoints want to input in a vector:
        std::vector<double> cmd_vel_;

        // Generate a boolean to start the following process with the service:
        bool start_following = false;

        // Re-initialize the waypoint index changing the MIT encounter set:
        size_t current_idx = 1;
        // Iteration counter:
        int count = 0;

        // Define teh different characteristics for the DAA systems:
        // GEOMTRIC:
        double crit_time_ = 0.0;
        double current_min_radius_;
        // FMT:
        // Lmits of the planner:
        std::vector<std::vector<double>> limits_;
        // Objective points:
        std::vector<double> start_;
        std::vector<double> goal_;
        // RT-FMT Planner:
        std::optional<FMTPlanner> rt_fmt_planner_;
        // COunter of ticks:
        int tick_counter_ = 0;
        // Uav parameters (that are not defined alrady):
        double fpa_limits_[2] = {-0.3, 0.3};
        // Mean airspeed:
        double mean_air_speed_;

        // Define the actual states of the ownship:
        double current_speed_;
        double max_roll_ = 35.0/57.3;

        // Define the avoider current state:
        gnss_multipath_plugin::msg::StatesInfo avoider_current_state_;

        // Start the vector where is goign to be defined if enter inside the avoidance zone:
        std::vector<std::pair<std::string, int>> is_inside_avoidance_zone_;

        // Define if the avoidance is started:
        bool start_the_avoidance = false;
        // End of avoidance arc made for avoidance:
        std::optional<size_t> end_of_arc_ = 0; 

        // Define the current_idx subscriber:
        std::mutex current_idx_mutex_;

        // Deifine the ROS2 subscribers and publishers:
        rclcpp::Subscription<gnss_multipath_plugin::msg::StatesInfo>::SharedPtr ownship_states_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_start_sub_;
        rclcpp::Subscription<uav_dynamics::msg::AvoidanceStates>::SharedPtr obstacles_states_sub_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr current_idx_sub_;
        rclcpp::Publisher<uav_dynamics::msg::IntrudersStatus>::SharedPtr is_inside_avo_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr is_avoiding_pub_;
        rclcpp::Publisher<uav_dynamics::msg::AvoidancePath>::SharedPtr waypoints_pub_; 
        // Define the avoidance last point: 
        std::optional<Eigen::Vector3d> avoidance_last_point_enu;
        



        ///////////////////////// FUNCTIONS //////////////////////////////
        // Parse the waypoints from a string to matrices of doubles to forloop in them:
        inline void parse_waypoints(const std::string &waypoints_str){
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


        // ENU -> NED conversion
        inline static Eigen::Vector3d ENU_to_NED(const Eigen::Vector3d &enu) {
            return Eigen::Vector3d(enu.y(), enu.x(), -enu.z()); // (N,E,-U)
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



        // Ownship avaodacne callabck:
        void ownship_states_callback(const gnss_multipath_plugin::msg::StatesInfo::SharedPtr msg){
            // Save the actual avoider state:
            avoider_current_state_ = *msg;

            // Only start if the Path follower its started:
            if (!start_following){
                return;
            }

            // Get teh intial velcoity:
            const double velocity_a = std::sqrt(msg->v_north * msg->v_north + msg->v_east * msg->v_east + msg->v_up * msg->v_up);

            // Change the order of the states information:
            // Todefine the actual position
            Eigen::Vector3d actual_pose;
            actual_pose << msg->north, msg->east, msg->up;

            // Define a vector as the actual state:
            Eigen::VectorXd actual_state(8);
            actual_state << msg->north, msg->east, msg->up, msg->course, msg-> fpa, velocity_a, msg-> roll, msg->p;

            // Calacualte the lookahed aditsnte and the transitionr adius:
            current_speed_ = std::max(velocity_a, 1.0);
            current_min_radius_ = 2.0 * (current_speed_ * current_speed_) / (std::tan(max_roll_) * 9.81);

            // Check if it gets near the goal:
            Eigen::Vector3d act_mod_pose = actual_pose;
            act_mod_pose(2) = -act_mod_pose(2); 

            // Identify if the avoidance is finisehd:
            if (start_the_avoidance) {
                if (guidance_system_ == "GEOMETRIC"){
                    // Add the minimum raduis to the avodance parameters.
                    if (start_the_avoidance) {
                        const Eigen::Vector3d avoidance_last_point_ned = ENU_to_NED(avoidance_last_point_enu.value()); // (N,E,-U)

                        if (((act_mod_pose - avoidance_last_point_ned).norm() <= 100.0) || current_idx >= end_of_arc_) {
                            start_the_avoidance = false;
                            RCLCPP_INFO(this->get_logger(), "Avoidance complete; resuming nominal path.");
                        }
                    }
                }
            }
        }


        // STrat the avoidance manuver or the movement:
        void start_mission_callback(const std_msgs::msg::Bool::SharedPtr msg){
            if (msg->data){
                start_following = true;
            } else {
                return;
            }
        }



        // FUnction to publisht eh waypoitns:
        void publish_avoidance_path() {
            uav_dynamics::msg::AvoidancePath msg;
            
            // Safely grab the current index using the mutex
            int local_idx;
            {
                std::lock_guard<std::mutex> lock(current_idx_mutex_);
                local_idx = current_idx;
            }
            msg.current_idx = local_idx;

            // Load all waypoints into the message
            for (size_t i = 0; i < waypoints_.size(); ++i) {
                uav_dynamics::msg::Waypoint wp;
                wp.north = waypoints_[i].x();
                wp.east = waypoints_[i].y();
                wp.down = waypoints_[i].z();
                wp.velocity = cmd_vel_[i];
                msg.waypoints.push_back(wp);
            }

            // Publish the message!
            waypoints_pub_->publish(msg);
        }



        // Callback that is called when a Obstacle message is recieved:
        void obstacle_states_callback(const uav_dynamics::msg::AvoidanceStates::SharedPtr msg){
            // Always start it:
            is_inside_avoidance_zone_.clear();
            for (const auto& id : msg->obstacles_id) {
                is_inside_avoidance_zone_.push_back({id, 0});
            }

            // Call the guidance logic:
            if (guidance_system_ == "GEOMETRIC") {
                NavigationState nav_state{
                    waypoints_, 
                    cmd_vel_, 
                    current_idx, 
                    start_the_avoidance,
                    avoidance_last_point_enu, 
                    end_of_arc_,
                    is_inside_avoidance_zone_
                };

                computeGeometricAvoidance(
                    *msg, 
                    avoider_current_state_, 
                    current_min_radius_, 
                    crit_time_, 
                    nav_state
                );
                // Publsih the waypoints:
                publish_avoidance_path();

            } else if (guidance_system_ == "FMT" ) {
                FMTNavigationState nav_state_fmt{
                    waypoints_,
                    cmd_vel_,
                    current_idx,
                    start_the_avoidance,
                    is_inside_avoidance_zone_,
                    tick_counter_
                };

                computerFMTAvoidance(
                    *msg,
                    avoider_current_state_, 
                    current_min_radius_, 
                    0.3, 
                    nav_state_fmt,
                    rt_fmt_planner_.value()
                );

                // ADD THIS: Advance the FMT clock!
                tick_counter_++;
                
                // Publsih the waypoints:
                if (tick_counter_ % 10) {
                    publish_avoidance_path();
                }
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
                is_avoiding_pub_->publish(avoiding_msg);
            }

        }



        void current_idx_callback(const std_msgs::msg::Int32::SharedPtr msg) {
            // Automatically lock the mutex for the duration of this scope
            std::lock_guard<std::mutex> lock(current_idx_mutex_);
            
            // Safely update the variable
            current_idx = msg->data;
        }
};



// Main logic of the Executable:
int main(int argc, char **argv){
    // Start the ROS 2 node:
    rclcpp::init(argc, argv);

    // Safety check that all the arguments are being inputted:
    if (argc < 5)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                     "Usage: ros2 run <package_name> <executable_name> <avoider_name> <waypoints_string> <guidance_system> <avoidance_analysis (0 or 1)>");
        return 1;
    }

    // Obtain the arguments from the command line:
    std::string avoider_name = argv[1];
    std::string waypoints_str = argv[2];
    std::string guidance_system = argv[3];
    bool avoidance_analysis = (std::stoi(argv[4]) != 0); 

    // Start the node with teh class:
    auto node = std::make_shared<DAATrajectoryPlanner>(avoider_name, waypoints_str, guidance_system, avoidance_analysis);

    // SPin the node:
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    while (rclcpp::ok()){
        exec.spin_once();
    }
    rclcpp::sleep_for(std::chrono::milliseconds(200));

    // Shutdown ROS 2:
    rclcpp::shutdown();
    return 0;
}