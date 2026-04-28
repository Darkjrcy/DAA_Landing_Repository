// ROS2 Library:
#include <rclcpp/rclcpp.hpp>
// Include ROS2 message:
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include "gnss_multipath_plugin/msg/states_info.hpp"
#include "uav_dynamics/msg/intruders_status.hpp"
#include "uav_dynamics/msg/intruder_flag.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
// C++ libraries:
#include <iostream>
#include <string>
#include <optional>
#include <vector>
#include <mutex>
#include <chrono>
#include <fstream> 
#include <cmath>
// Gazebo libaries used to interact with quaternions and odometry message:
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>



// Create a structure to save the Odometry messgaes:
struct Odom_unit {
    double sim_time;
    double real_time;
    
    // Position
    double pos_east_real, pos_north_real, pos_up_real;
    // Orientation (Quaternion)
    double course_real, fpa_real, roll_real;
    // Linear Velocity (Body Frame Nose-rigth-down)
    double vel_north_real, vel_east_real, vel_up_real;
    // Angular Velocity (Body frame)
    double p, q, r;
};

// Create a structure to save the ADS-B inforamtion:
struct Adsb_unit {
    double sim_time;
    double real_time;

    // Position
    double pos_north, pos_east, pos_up;
    // Velocity in the Earth RF
    double v_north, v_east, v_up;
    // Orientation from the Earth RF:
    double course, fpa, roll;
    // Angular Velocity (Body frame)
    double p, q, r;

};

// Create a structure to save the infroamtion ralted to the avoidance:
struct avoidance_info_unit
{
    double sim_time;
    double real_time;

    // The intruders names wiht the boolean saying if they are insie the avoidance radius
    std::vector<uav_dynamics::msg::IntruderFlag> states;
    
    // Is the system avoidance:
    int is_avoiding;
};




// Generate the class of the executable:
class SaveUAVInfo : public rclcpp::Node{
    public:
        SaveUAVInfo(const std::vector<std::string> &model_names, const std::map<std::string, bool> &is_avoider_map, const std::string &save_dir, const bool avoidance_analysis) 
        : rclcpp::Node("save_info_node"), model_names_(model_names), save_dir_(save_dir), is_avoider_map_(is_avoider_map), avoidance_analysis_(avoidance_analysis){
            // Declare the topic parameters:
            std::string mission_start_topic = "/mission_starts";
            std::string traj_complete_topic = "/traj_complete";
            // Declare the saving directory:
            save_dir_ = save_dir;

            // Declare a saving rate parameter:
            update_rate_ = this->declare_parameter<double>("update_rate", 10.0);

            // Subscribe to the clock to oabtin the simulation time:
            clock_cub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
                "/clock", 10, std::bind(&SaveUAVInfo::clock_callback, this, std::placeholders::_1));
            
            // SUbscribe to the the tracjectory complete node:
            rclcpp::QoS qos_profile_reliable(10);
            qos_profile_reliable.reliable();
            trajectory_complete_sub_ = this->create_subscription<std_msgs::msg::Bool>( traj_complete_topic, qos_profile_reliable,
                std::bind(&SaveUAVInfo::complete_callback, this, std::placeholders::_1));
            
            // Subscribe to the start mission topic to start saving the data:
            rclcpp::QoS qos_profile_transient(1);
            qos_profile_transient.transient_local();
            qos_profile_transient.reliable();
            start_recording_sub_ = this->create_subscription<std_msgs::msg::Bool>( mission_start_topic, qos_profile_transient,
                std::bind(&SaveUAVInfo::start_recording_callback, this, std::placeholders::_1));

            // Dynamically creasete subscibers to the adsb_info and states fo teh UAVs:
            for (const auto& name : model_names_) {
                // Define the topic names:
                std::string states_topic = name + "/states";
                std::string adsb_info_topic = name + "/adsb_info";

                // Start empty histories:
                odom_histories_[name] = std::vector<Odom_unit>();
                adsb_histories_[name] = std::vector<Adsb_unit>();

                // Create the sates subscribers for each model name:
                states_subs_[name] =  this->create_subscription<nav_msgs::msg::Odometry>(
                    states_topic, 10,
                    [this, name](const nav_msgs::msg::Odometry::SharedPtr msg) {
                        std::lock_guard<std::mutex> lock(mutex_);
                        last_uav_states_[name] = *msg;
                    }
                );

                // Create the ADS-B subscribers
                adsb_subs_[name] = this->create_subscription<gnss_multipath_plugin::msg::StatesInfo>(
                    adsb_info_topic, 10,
                    [this, name](const gnss_multipath_plugin::msg::StatesInfo::SharedPtr msg) {
                        std::lock_guard<std::mutex> lock(mutex_);
                        last_adsb_states_[name] = *msg;
                    }
                );

                // First lets see if the user require and avoidance analysis:
                if (avoidance_analysis_){
                    // Identify the avoiders:
                    if (is_avoider_map_[name]) {
                        // oBTAIN THE TOPIC with teh inforamtin showing if it is in the avoidance zone:
                        std::string intruder_avoidance_topic = name + "/is_inside_avo";
                        std::string is_avoiding_topic = name + "/is_avoiding";

                        // Create histories for every avoider:
                        avoidance_histories_[name] = std::vector<avoidance_info_unit>();

                        // Create Intruder Status subscriber
                        intruders_info_subs_[name] = this->create_subscription<uav_dynamics::msg::IntrudersStatus>(
                            intruder_avoidance_topic, 10,
                            [this, name](const uav_dynamics::msg::IntrudersStatus::SharedPtr msg) {
                                std::lock_guard<std::mutex> lock(mutex_);
                                last_intruders_states_[name] = *msg;
                            }
                        );

                        // Create the subscriber ot see if the avoider is active:
                        avoiding_subs_[name] = this->create_subscription<std_msgs::msg::Int32>(
                            is_avoiding_topic, 10,
                            [this, name](const std_msgs::msg::Int32::SharedPtr msg) {
                                std::lock_guard<std::mutex> lock(mutex_);
                                last_avoiding_states_[name] = *msg;
                            }
                        );

                    }
                }
            }

            // Create the timer to save the information:
            auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_));
            timer_ = this->create_wall_timer(timer_period, std::bind(&SaveUAVInfo::timer_callback, this));
        }
    
    private:
        // Update saving rate:
        double update_rate_;
        // Name of the UAV model in Gazebo:
        std::vector<std::string> model_names_;
        // Saving directory path:
        std::string save_dir_;
        // Simualtion time fom gz sim clock:
        double sim_time_ = 0.0;
        // Initial simualtion time:
        double sim_time_0_ = 0.0;
        // Intiial real time:
        double real_time_0_ = 0.0;

        // Boolean variable ot see if we need to estart:
        bool is_recording_ = false;

        // Mutex for thread safety
        std::mutex mutex_;

        // Last sates of the UAVs:
        std::map<std::string, std::optional<nav_msgs::msg::Odometry>> last_uav_states_;
        // Last ADS-Bs states:
        std::map<std::string, std::optional<gnss_multipath_plugin::msg::StatesInfo>> last_adsb_states_;
        // Last states fo teh avoidance:
        std::map<std::string, std::optional<uav_dynamics::msg::IntrudersStatus>> last_intruders_states_;
        std::map<std::string, std::optional<std_msgs::msg::Int32>> last_avoiding_states_;

        // Maps to hold the historical data for each UAV
        std::map<std::string, std::vector<Odom_unit>> odom_histories_;
        std::map<std::string, std::vector<Adsb_unit>> adsb_histories_;
        std::map<std::string, std::vector<avoidance_info_unit>> avoidance_histories_;


        // Define the subscribers in ROS2:
        std::map<std::string, rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> states_subs_;
        std::map<std::string, rclcpp::Subscription<gnss_multipath_plugin::msg::StatesInfo>::SharedPtr> adsb_subs_;
        std::map<std::string, rclcpp::Subscription<uav_dynamics::msg::IntrudersStatus>::SharedPtr> intruders_info_subs_;
        std::map<std::string, rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr> avoiding_subs_;
        rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_cub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_recording_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr trajectory_complete_sub_;
        // Define the saving timer:
        rclcpp::TimerBase::SharedPtr timer_;

        // Variable to see if the airplanes are avoiders:
        std::map<std::string, bool> is_avoider_map_;
        // Permit the avoidance anylsis saving files:
        bool avoidance_analysis_;



        ///////////////////////////////// FUNCTIONS ///////////////////////////////////
        // Callback to teh clock to save the time:
        void clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg) {
            // The message contains a 'clock' field with 'sec' and 'nanosec'
            int32_t seconds = msg->clock.sec;
            uint32_t nanoseconds = msg->clock.nanosec;
            
            // Convert to a double for easy reading/logging
            sim_time_ = seconds + (nanoseconds / 1e9);
        }



        // Callback to restart the recording of data:
        void start_recording_callback(const std_msgs::msg::Bool::SharedPtr msg) {
            // Start th recording only if its true:
            if (msg->data) {
                is_recording_ = true;
                sim_time_0_ = sim_time_;
                real_time_0_ = this->get_clock()->now().seconds();
            }
        }


        // Callback to see when the trajectory is completed:
        void complete_callback(const std_msgs::msg::Bool::SharedPtr msg) {
            // Start th recording only if its true:
            if (msg->data) {
                save_to_csv();
                // Clear the maps with the states and the adsb data:
                for (const auto& name : model_names_) {
                    odom_histories_[name].clear();
                    adsb_histories_[name].clear();
                    avoidance_histories_[name].clear();
                }
            }
        }



        // Timer callback tosave the inforamtion in vectors:
        void timer_callback() {
            std::lock_guard<std::mutex> lock(mutex_);

            // Check if the system needs to start recording:
            if (!is_recording_){
                return;
            }

            // Get teh actual real time:
            double current_real_time = this->get_clock()->now().seconds();

            // Save the odometry:
            for (const auto& name : model_names_) {
                // Save the odometry:
                if (last_uav_states_[name].has_value()){
                    auto state = last_uav_states_[name].value();
                    // Tansofrm the quaternion in course, fpa, and roll
                    gz::math::Quaterniond q(state.pose.pose.orientation.w, state.pose.pose.orientation.x,
                        state.pose.pose.orientation.y, state.pose.pose.orientation.z);
                    // Pass it to roll, pitch and yaw:
                    gz::math::Vector3d euler = q.Euler();
                    double roll_odom = euler.X();
                    double pitch_odom = euler.Y();
                    double yaw_odom = euler.Z();

                    // Define the course, fpa, and roll;
                    double course_odom = M_PI/2 - yaw_odom;
                    double fpa_odom = -pitch_odom;

                    // Define the linear velocity in the Earth RF:
                    gz::math::Vector3d body_vel(state.twist.twist.linear.x, state.twist.twist.linear.y * -1,
                        state.twist.twist.linear.z * -1);
                    gz::math::Vector3d world_vel = q.RotateVector(body_vel);

                    // Save in the vector:
                    Odom_unit odom_unity;
                    odom_unity.sim_time = sim_time_ - sim_time_0_;
                    odom_unity.real_time = current_real_time - real_time_0_;
                    odom_unity.pos_east_real = state.pose.pose.position.x;
                    odom_unity.pos_north_real = state.pose.pose.position.y;
                    odom_unity.pos_up_real = state.pose.pose.position.z;
                    odom_unity.course_real = course_odom;
                    odom_unity.fpa_real = fpa_odom;
                    odom_unity.roll_real = roll_odom;
                    odom_unity.vel_east_real = world_vel.X();
                    odom_unity.vel_north_real = world_vel.Y();
                    odom_unity.vel_up_real = world_vel.Z();
                    odom_unity.p = state.twist.twist.angular.x;
                    odom_unity.q = state.twist.twist.angular.y;
                    odom_unity.r = state.twist.twist.angular.z;   
                    odom_histories_[name].push_back(odom_unity);
                }

                // Save the ADS-B data:
                if (last_adsb_states_[name].has_value()) {
                    auto adsb = last_adsb_states_[name].value();
                    Adsb_unit adsb_unity;
                    adsb_unity.sim_time = sim_time_ - sim_time_0_;
                    adsb_unity.real_time = current_real_time - real_time_0_;
                    adsb_unity.pos_north = adsb.north;
                    adsb_unity.pos_east = adsb.east;
                    adsb_unity.pos_up = adsb.up;
                    adsb_unity.v_north = adsb.v_north;
                    adsb_unity.v_east = adsb.v_east;
                    adsb_unity.v_up = adsb.v_up;
                    adsb_unity.course = adsb.course;
                    adsb_unity.fpa = adsb.fpa;
                    adsb_unity.roll = adsb.roll;
                    adsb_unity.p = adsb.p;
                    adsb_unity.q = adsb.q;
                    adsb_unity.r = adsb.r;
                    adsb_histories_[name].push_back(adsb_unity);
                }

                // Save teh avoiders info in teh history:
                if (avoidance_analysis_ && is_avoider_map_[name]){
                    if (last_intruders_states_[name].has_value() && last_avoiding_states_[name].has_value()) {
                        auto intruders_msg = last_intruders_states_[name].value();
                        auto avoiding_msg = last_avoiding_states_[name].value();

                        // Create one avoidance info structure:
                        avoidance_info_unit avo_unit;
                        avo_unit.sim_time = sim_time_ - sim_time_0_;
                        avo_unit.real_time = current_real_time - real_time_0_;
                        avo_unit.states = intruders_msg.states;
                        avo_unit.is_avoiding = avoiding_msg.data; 
                        avoidance_histories_[name].push_back(avo_unit);
                    }
                }

            }
        }



        // Function to save the data
        void save_to_csv(){
            // Create  a mutex to have a complete saving system:
            std::lock_guard<std::mutex> lock(mutex_);

            // FOrloop between all the models to start saving:
            for (const auto& name : model_names_) {
                // Check if the specifc data for each model exitsts:
                if (odom_histories_[name].empty() || adsb_histories_[name].empty()){
                    continue;
                }

                // Define the file record name:
                std::string save_filename = save_dir_ + "/" + name + "_data.csv";
                std::ofstream save_file(save_filename);

                // Define the number of dat is collected per fligth
                size_t num_records = std::min(odom_histories_[name].size(), adsb_histories_[name].size());

                // wRITE TEH CSV FILE:
                if (save_file.is_open()){
                    // Strart with the titles:
                    save_file << "sim_time,real_time,pos_east_real,pos_north_real,pos_up_real,"
                              << "course_real,fpa_real,roll_real,vel_east_real,vel_north_real,vel_up_real,"
                              << "p_real,q_real,r_real,pos_east_est,pos_north_est,pos_up_est,"
                              << "course_est,fpa_est,roll_est,vel_east_est,vel_north_est,vel_up_est,"
                              << "p_est,q_est,r_est\n";

                    // Now record the data using a forloop:
                    for (size_t i = 0; i < num_records; ++i) {
                        const auto& data_odom = odom_histories_[name][i];
                        const auto& data_adsb = adsb_histories_[name][i];
                        save_file << data_odom.sim_time << "," << data_odom.real_time << ","
                                  << data_odom.pos_east_real << "," << data_odom.pos_north_real << "," << data_odom.pos_up_real << ","
                                  << data_odom.course_real << "," << data_odom.fpa_real << "," << data_odom.roll_real << ","
                                  << data_odom.vel_east_real << "," << data_odom.vel_north_real << "," << data_odom.vel_up_real << ","
                                  << data_odom.p << "," << data_odom.q << "," << data_odom.r << ","
                                  << data_adsb.pos_east << "," << data_adsb.pos_north << "," << data_adsb.pos_up << ","
                                  << data_adsb.course << "," << data_adsb.fpa << "," << data_adsb.roll << ","
                                  << data_adsb.v_east << "," << data_adsb.v_north << "," << data_adsb.v_up << ","
                                  << data_adsb.p << "," << data_adsb.q << "," << data_adsb.r << "\n";
                    }
                    // cLOSE THE FILE AND SAVE IT:
                    save_file.close();
                    RCLCPP_INFO(this->get_logger(), "Successfully saved combined data for %s", name.c_str());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to open file %s for writing.", save_filename.c_str());
                }

                // Save teh avoidance analysis if its necesary:
                if (avoidance_analysis_ && is_avoider_map_[name]){
                    // Check if there is avoidance history to save:
                    if (!avoidance_histories_[name].empty()) {
                        // FIle name pf the avoidance info:
                        std::string save_avo_filename = save_dir_ + "/" + name + "_avoidance_info.csv";
                        std::ofstream avo_file(save_avo_filename);

                        // Record the data inside the csv file:
                        if (avo_file.is_open()) {
                            // Create teh headers
                            avo_file  << "sim_time,real_time";
                            // CReate each of the obstacles:
                            const auto& first_record = avoidance_histories_[name][0];
                            for (size_t j = 0; j < first_record.states.size(); ++j) {
                                avo_file << "," << first_record.states[j].obstacle_id;
                            }
                            // Header with teh boolean to se if the avoider is active:
                            avo_file << ",avoidance_started\n";

                            // Write teh avoidance info data:
                            for (const auto& avo_data : avoidance_histories_[name]) {
                                avo_file << avo_data.sim_time << "," << avo_data.real_time;
                                for (const auto& intruder : avo_data.states) {
                                    avo_file << "," << (intruder.is_inside ? 1 : 0); 
                                }
                                avo_file << "," << avo_data.is_avoiding << "\n";
                            }
                            
                            // Close the file:
                            avo_file.close();
                            RCLCPP_INFO(this->get_logger(), "Successfully saved avoidance data for %s", name.c_str());
                        } else {
                            RCLCPP_ERROR(this->get_logger(), "Failed to open avoidance file %s for writing.", save_avo_filename.c_str());
                        }
                    }
                }
            }
        }
};




//starts the node:
int main(int argc, char **argv) {
    // Start the ROS2 node:
    rclcpp::init(argc, argv);
    // Obtain the arguments:
    std::vector<std::string> model_names;
    std::map<std::string, bool> is_avoider_map;
    bool avoidance_analysis = false;
    std::string save_dir = "/home/jorge/DAA_Landing_Repository/ros_ws/src/uav_bringup/saving_data";
    if (argc > 2) {
        save_dir = argv[1];
        avoidance_analysis = (std::stoi(argv[2]) != 0); 
        model_names.clear(); // Clear the default
        for (int i = 3; i < argc; i+=2) {
            std::string name = argv[i];
            model_names.push_back(name);

            // Default boolean:
            bool is_avoider = false;
            if (i + 1 < argc) {
                std::string bool_str = argv[i+1];
                if (bool_str == "true" || bool_str == "True" || bool_str == "1") {
                    is_avoider = true;
                }
            }
            is_avoider_map[name] = is_avoider;
        }
    } else if (argc == 2) {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Only 1 argument provided. Usage: <save_dir> <model_1> <model_2> ...");
        save_dir = argv[1];
    } else {
        model_names.push_back("airplane_1");
        is_avoider_map["airplane_1"] = true;
    }
    // Create the node class:
    auto node = std::make_shared<SaveUAVInfo>(model_names, is_avoider_map, save_dir, avoidance_analysis);
    
    RCLCPP_INFO(node->get_logger(), "Centralized recording node started. Tracking %zu models.", model_names.size());
    
    // Spin teh NOde and close it when Ctrl+C is pressed
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}