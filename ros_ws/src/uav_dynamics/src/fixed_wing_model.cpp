// Add ROS2 library
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
// C++ libraries: 
#include <string>
#include <vector>
#include <array>
#include <sstream>
#include <cmath>
#include <algorithm> 
#include <optional>
#include <Eigen/Dense> 
// ROS2 Messages:
#include "uav_dynamics/msg/avoidance_states.hpp"
#include "gnss_multipath_plugin/msg/adsb_info.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
// Add the library 
#include "uav_dynamics/geometric_guidance.hpp"


// Start the class of the fixed-wing Dynamics;
class FixedWingDynamics : public rclcpp::Node{
    public:
        // Do the starting function:
        FixedWingDynamics(const std::string &avoider_name, const std::string &waypoints_str, 
            const std::string guidance_system, const bool active_avoidance, const std::string guidance_type
            ): Node(("dynamics_" + avoider_name).c_str()), avoider_name_(avoider_name), guidance_type_(guidance_type), active_avoidance_(active_avoidance){

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

                // Specify the vearibles used for the differnt Guidance ALgorithms:
                if (active_avoidance_ && guidance_type_ == "GEOMETRIC") {
                    avoidance_vars_geom_ = GeometricAvoidanceVars{
                        180, // min_radius of turn (m)
                        0.1 // critical avoidance time (s)
                    };
                }

                // Subscriber to the states of the avoider:
                avoider_states_sub_ = this->create_subscription<gnss_multipath_plugin::msg::AdsbInfo>(
                    avoider_topic, qos, std::bind(&FixedWingDynamics::avoider_states_callback, this, std::placeholders::_1));
                
                // Subscribe to the obstacles states:
                if (active_avoidance_){
                    obstacles_states_sub_ = this->create_subscription<uav_dynamics::msg::AvoidanceStates>(
                        obstacles_topic, qos, std::bind(&FixedWingDynamics::obstacle_states_callback, this, std::placeholders::_1));
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
        std::string guidance_type_;
        bool active_avoidance_;


        // Define the wripoints vector adn the last waypoint:
        std::vector<Eigen::Vector3d> waypoints_;
        Eigen::Vector3d last_waypoint_;
        // Save the velocity that the waypoints want to input in a vector:
        std::vector<double> cmd_vel_;

        // Define the vraibles of the Waypoint follower to see the next position:
        double transition_radius_ = 420;
        double look_ahead_distance_ = 250;

        // Define the avoider current state:
        gnss_multipath_plugin::msg::AdsbInfo avoider_current_state_;

        // Re-initialize the waypoint index changing the MIT encounter set:
        size_t current_idx = 0;
        // Iteration counter:
        int count = 0;
        
        // Generate a boolean to start the following process with the service:
        bool start_following = false;

        // Deifine the ROS2 subscribers and publishers:
        rclcpp::Subscription<gnss_multipath_plugin::msg::AdsbInfo>::SharedPtr avoider_states_sub_;
        rclcpp::Subscription<uav_dynamics::msg::AvoidanceStates>::SharedPtr obstacles_states_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr complete_encounter_pub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr complete_encounter_sub_;

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
        // FGAC characteristics:
        struct GeometricAvoidanceVars {
            double min_radius;
            double crit_time;
        };
        std::optional<GeometricAvoidanceVars> avoidance_vars_geom_;

        



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



        // Define the NextGoalInformation
        struct NextGoalInformation{
            double course_cmd;
            Eigen::Vector3d look_ahead_pos;
            int act_idx;
        };



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
            const double kp_V = 6.7; // proportional gain of the velocity
            const double kp_roll = 2.5; //proportional gain of the roll
            const double kd_roll = 0.8; //derivatice gain of the roll
            const double kp_Y = 0.25; // proportional gain of the course
            const double kp_h = 0.4; // proportional gain of the velocity
            const double kp_heading = 0.8; // proportional gain of the heading

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
            roll_cmd = std::clamp(roll_cmd, -35.0/57.3, 35.0/57.3);

            // Obtian the FPA command:
            double alt_diff = kp_h * (alt_cmd - state(2));
            alt_diff = std::clamp(alt_diff, -V, V);
            double Y_cmd = asin((1.0 / V) * alt_diff);

            // Genate the derivative vector:
            dstate(0) = V * cos(state(3)) * cos(state(4));
            dstate(1) = V * sin(state(3)) * cos(state(4));
            dstate(2) = V * sin(state(4));
            dstate(3) = (g * tan(state(6))) / V;
            dstate(4) = kp_Y * (Y_cmd - state(4));
            dstate(5) = kp_V * (vel_cmd - V);
            dstate(6) = state(7);
            dstate(7) = kp_roll * (roll_cmd - state(6)) - kd_roll * state(7);
            return dstate;
        }



        // ENU -> NED conversion
        inline static Eigen::Vector3d ENU_to_NED(const Eigen::Vector3d &enu) {
            return Eigen::Vector3d(enu.y(), enu.x(), -enu.z()); // (N,E,-U)
        }



        // Avoider ads-b message callback:
        void avoider_states_callback(const gnss_multipath_plugin::msg::AdsbInfo::SharedPtr msg){
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
                if (guidance_type_ == "GEOMETRIC"){
                    if (start_the_avoidance) {
                        const Eigen::Vector3d avoidance_last_point_ned = ENU_to_NED(avoidance_last_point_enu.value()); // (N,E,-U)

                        if (((act_mod_pose - avoidance_last_point_ned).norm() <= 100.0) || current_idx >= end_of_arc_) {
                            start_the_avoidance = false;

                            // restore thresholds/params      
                            transition_radius_ = 420;   
                            look_ahead_distance_ = 250;

                            RCLCPP_INFO(this->get_logger(), "Avoidance complete; resuming nominal path.");
                        }
                    }
                }
            }

            // Do stop system to stop if is near teh last waypoint:
            if ((act_mod_pose - last_waypoint_).norm() <= 50){
                goal_reached_ = true;
                std_msgs::msg::Bool finish_follow;
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
            } else {
                goal_reached_ = false;
                // Send the required velcoity command:
                double com_linear_vel = std::sqrt(std::max(0.0,
                    actual_dstates(0)*actual_dstates(0)+
                    actual_dstates(1)*actual_dstates(1)+
                    actual_dstates(2)*actual_dstates(2)));
                com_linear_vel = std::clamp(com_linear_vel, 5.0, 420.0);
                cmd_velocity.linear.x = com_linear_vel;
                cmd_velocity.angular.x = actual_dstates(7);
                cmd_velocity.angular.y = -actual_dstates(4);
                cmd_velocity.angular.z = -actual_dstates(3);
                // publish it:
                cmd_vel_pub_->publish(cmd_velocity);
            }
            

            // Publish to the other UAVs that the goal is completed:
            if (goal_reached_ != last_published_goal_reached_) {
                std_msgs::msg::Bool finish_follow;
                finish_follow.data = goal_reached_;
                complete_encounter_pub_->publish(finish_follow);
                last_published_goal_reached_ = goal_reached_;
            }

        }



        // Callback that is called when a Obstacle message is recieved:
        void obstacle_states_callback(const uav_dynamics::msg::AvoidanceStates::SharedPtr msg){
            // Only start if the Path follower its started:
            if (!start_following){
                return;
            }

            // Call the guidance logic:
            if (guidance_type_ == "GEOMETRIC" && avoidance_vars_geom_.has_value()) {
                NavigationState nav_state{
                    waypoints_, 
                    cmd_vel_, 
                    current_idx, 
                    transition_radius_,
                    look_ahead_distance_, 
                    start_the_avoidance,
                    avoidance_last_point_enu, 
                    end_of_arc_
                };

                computeGeometricAvoidance(
                    *msg, 
                    avoider_current_state_, 
                    avoidance_vars_geom_->min_radius, 
                    avoidance_vars_geom_->crit_time, 
                    nav_state
                );
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
                     "Usage: ros2 run <package_name> <executable_name> <avoider_name> <waypoints_string> <guidance_system> <active_avoidance (0 or 1)> <guidance_type>");
        return 1;
    }

    // Obtain the arguments from the command line:
    std::string avoider_name = argv[1];
    std::string waypoints_str = argv[2];
    std::string guidance_system = argv[3];
    bool active_avoidance = (std::stoi(argv[4]) != 0); 
    std::string guidance_type = argv[5];

    // Start the node with teh class:
    auto node = std::make_shared<FixedWingDynamics>(avoider_name, waypoints_str, guidance_system, active_avoidance, guidance_type);

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