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
#include "gnss_multipath_plugin/msg/states_info.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include "std_msgs/msg/int32.hpp"
// Add the ROS2 message to recieve the waypoints:
#include "uav_dynamics/msg/avoidance_path.hpp"
#include "uav_dynamics/msg/waypoint.hpp"

// Start the class of the fixed-wing Dynamics;
class FixedWingDynamics : public rclcpp::Node{
    public:
        // Do the starting function:
        FixedWingDynamics(const std::string &avoider_name, const std::string &waypoints_str, const bool active_avoidance
            ): Node(("dynamics_" + avoider_name).c_str()), avoider_name_(avoider_name), active_avoidance_(active_avoidance){
            
            // Qualioty of serviuce for the states messages:
                auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliability(rclcpp::ReliabilityPolicy::Reliable).durability(rclcpp::DurabilityPolicy::Volatile);

                // Parse the avoidance waypoints:
                parse_waypoints(waypoints_str);

                // Define the topics:
                // Topic where the avoider publish its states using the ADS-B
                std::string avoider_topic = avoider_name_ + "/adsb_info";
                // Topic to publish teh velcoities to the movement pruggin:
                std::string cmd_vel_topic = avoider_name_ + "/cmd_vel";
                // Topic to say that the trajectory is completed.
                std::string complete_topic = "/traj_complete";
                // Topc to see when the simulation is restatrded
                std::string mission_start_topc = "/mission_starts";

                // Subscriber to the states of teh avoider:
                avoider_states_sub_ = this->create_subscription<gnss_multipath_plugin::msg::StatesInfo>(
                    avoider_topic, qos, std::bind(&FixedWingDynamics::avoider_states_callback, this, std::placeholders::_1));

                // Subscribe to the mission:
                auto qos_miss = rclcpp::QoS(rclcpp::KeepLast(1)).reliability(rclcpp::ReliabilityPolicy::Reliable).durability(rclcpp::DurabilityPolicy::TransientLocal);
                mission_start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                    mission_start_topc, qos_miss, std::bind(&FixedWingDynamics::mission_start_callback, this, std::placeholders::_1)
                );

                // Publisher to the command velcoity and use the plugin movement:
                cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, qos);
                // Publisher to define if the trajecotry of any of the UAVs is completed:
                complete_encounter_pub_ = this->create_publisher<std_msgs::msg::Bool>(complete_topic, qos);
                // Subscriber to define if any other UAV complete its trajecotry:
                complete_encounter_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                    complete_topic, qos, std::bind(&FixedWingDynamics::completeEncounterCallback, this, std::placeholders::_1)
                );

                // If the system has an avoidance flag subscribe to teh waypoint avodiance genrator in other node
                if (active_avoidance_){
                    // Define the topic of DAA trajectory
                    std::string waypoints_topic = avoider_name_ + "/daa_waypoints";
                    // Define the topic for the current idx:
                    std::string current_idx_topic = avoider_name_ + "/current_idx";

                    // Create the subscriber to the Waypoints from teh DAA trajecotry:
                    waypoints_from_daa_sub_ = this->create_subscription<uav_dynamics::msg::AvoidancePath>(
                        waypoints_topic, qos, std::bind(&FixedWingDynamics::waypoints_callback, this, std::placeholders::_1)
                    );

                    // Publish the current idx:
                    current_idx_pub_ = this->create_publisher<std_msgs::msg::Int32>(current_idx_topic, qos);
                }

        }

        // Function to say when goal is reached:
        bool is_goal_reached() const { return goal_reached_; }
    
    private:
        // Define the avoider_name of the model:
        std::string avoider_name_;
        // DEfine if the avoidance is active:
        bool active_avoidance_;

        // Define the wripoints vector adn the last waypoint:
        std::vector<Eigen::Vector3d> waypoints_;
        Eigen::Vector3d last_waypoint_;
        // Save the velocity that the waypoints want to input in a vector:
        std::vector<double> cmd_vel_;

        // Define the avoider current state:
        gnss_multipath_plugin::msg::StatesInfo avoider_current_state_;

        // Generate a boolean to start the following process with the service:
        bool start_following = false;

        // Define the vraibles of the Waypoint follower to see the next position:
        double transition_radius_;
        double look_ahead_distance_;

        // Re-initialize the waypoint index changing the MIT encounter set:
        size_t current_idx = 0;
        // Iteration counter:
        int count = 0;

        // Variable to see when the UAV reaches the last waypoint:
        bool goal_reached_ = false;
        bool last_published_goal_reached_ = false;

        // UAV chracteristics
        double min_radius_;
        double max_roll_ = 35.0/57.3;

        // New waypoints that come from teh DAA system:
        std::vector<Eigen::Vector3d> new_waypoints_;
        std::vector<double> new_cmd_vel_; 

        // Deifine the ROS2 subscribers and publishers:
        rclcpp::Subscription<gnss_multipath_plugin::msg::StatesInfo>::SharedPtr avoider_states_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_start_sub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr complete_encounter_pub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr complete_encounter_sub_;
        rclcpp::Subscription<uav_dynamics::msg::AvoidancePath>::SharedPtr waypoints_from_daa_sub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr current_idx_pub_;
        


        ////////////////////////////////// FUNCTIONS //////////////////////////////////
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
            const double kp_V = 0.6; // proportional gain of the velocity
            const double kp_roll = 0.6; //proportional gain of the roll
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
            transition_radius_ = std::max(100.0, current_min_radius * 2.0); 
            look_ahead_distance_ = std::max(20.0, current_speed * 3.0); 

            // USe the Waypoint follower to obtain teh next waypoint to follow:
            NextGoalInformation next_goal = WaypointFollower(actual_pose, waypoints_, look_ahead_distance_, current_idx, transition_radius_);
            // Update the index:
            current_idx = next_goal.act_idx;
            if (active_avoidance_){
                std_msgs::msg::Int32 idx_msg;
                idx_msg.data = current_idx;
                current_idx_pub_->publish(idx_msg);
            }

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



        // STrat the avoidance manuver or the movement:
        void mission_start_callback(const std_msgs::msg::Bool::SharedPtr msg){
            if (msg->data){
                start_following = true;
            } else {
                return;
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



        // Callback every time waypoitns have been generated
        void waypoints_callback(const uav_dynamics::msg::AvoidancePath::SharedPtr msg){
            // Clear the old waypoints obtained from past DAA generation:
            new_waypoints_.clear();
            new_cmd_vel_.clear(); 

            // Loop through the waypoints to 
            for (const auto& wp : msg->waypoints) {
                // Assuming your Waypoint.msg has fields: north, east, down, velocity
                new_waypoints_.push_back(Eigen::Vector3d(wp.north, wp.east, wp.down));
                new_cmd_vel_.push_back(wp.velocity);
            }

            // Save the last waypoint for the "is_goal_reached" stopping logic
            if (!new_waypoints_.empty()) {
                last_waypoint_ = new_waypoints_.back();
            }

            // Get the actual_idx:
            current_idx = msg->current_idx;

            // Update the waypoints and cms_vel:
            waypoints_ = new_waypoints_;
            cmd_vel_ = new_cmd_vel_;
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
    if (argc < 4)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), 
                     "Usage: ros2 run <package_name> <executable_name> <avoider_name> <waypoints_string> <active_avoidance (0 or 1)>");
        return 1;
    }

    // Obtain the arguments from the command line:
    std::string avoider_name = argv[1];
    std::string waypoints_str = argv[2];
    bool active_avoidance = (std::stoi(argv[3]) != 0);  

    // Start the node with teh class:
    auto node = std::make_shared<FixedWingDynamics>(avoider_name, waypoints_str, active_avoidance);

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