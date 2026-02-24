// ROS2 Library:
#include <rclcpp/rclcpp.hpp>
// Include ROS2 message:
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include "gnss_multipath_plugin/msg/adsb_info.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
// C++ libraries:
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



// Generate the class of the executable:
class SaveUAVInfo : public rclcpp::Node{
    public:
        SaveUAVInfo(const std::string &model_name) : rclcpp::Node(("save_info_" + model_name).c_str()), model_name_(model_name){
            // Declare the topic parameters:
            std::string states_topic = model_name_ + "/states";
            std::string adsb_info_topic = model_name_ + "/adsb_info";
            std::string start_save_trigger_topic = "/start_saving";

            // Declare a saving rate parameter:
            update_rate_ = this->declare_parameter<double>("update_rate", 10.0);

            // Define the subscriptions to the states and adsb_info:
            states_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(states_topic, 10,
                            [this](const nav_msgs::msg::Odometry &msg) {std::lock_guard<std::mutex> lock(mutex_);last_usv_state_ = msg;});
            adsb_sub_ = this->create_subscription<gnss_multipath_plugin::msg::AdsbInfo>(states_topic, 10,
                            [this](const gnss_multipath_plugin::msg::AdsbInfo &msg) {std::lock_guard<std::mutex> lock(mutex_);last_adsb_ = msg;});
            // Sunscribe to the clock to oabtin the simulation time:
            clock_cub_ = this->create_subscription<rosgraph_msgs::msg::Clock>("/clock", 10, std::bind(&SaveUAVInfo::clock_callback, this, std::placeholders::_1));
            // Topic to restart the recording or start it:
            start_recording_sub_ = this->create_subscription<std_msgs::msg::Bool>( start_save_trigger_topic, 10,
                std::bind(&SaveUAVInfo::start_recording_callback, this, std::placeholders::_1));

            // TImer to save the information in a specfici upfate rate:
            auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_));
            timer_ = this->create_wall_timer(timer_period, td::bind(&SaveUAVInfo::timer_callback, this));
        }
    
    private:
        // Update saving rate:
        double update_rate_;
        // Name of the UAV model in Gazebo:
        std::string model_name_;
        // Simualtion time fom gz sim clock:
        double sim_time_ = 0.0;

        // Boolean variable ot see if we need to estart:
        bool is_recording_ = false;
        // Coutner of the number of simualtions it have been done continuoslly:
        int count_sim_ = 0;

        // Mutex for thread safety
        std::mutex mutex_;

        // Last sates of the UAV:
        std::optional<nav_msgs::msg::Odometry> last_uav_state_;
        // Last ADS-B state:
        std::optional<gnss_multipath_plugin::msg::AdsbInfo> last_adsb_;

        // Vectors to hold the historical data
        std::vector<Odom_unit> odom_history_;
        std::vector<Adsb_unit> adsb_history_;

        // Define the subscribers in ROS2:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr states_sub_;
        rclcpp::Subscription<gnss_multipath_plugin::msg::AdsbInfo>::SharedPtr adsb_sub_;
        rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_cub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_recording_sub_;
        // Define the saving timer:
        rclcpp::TimerBase::SharedPtr timer_;



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
            if (msg.data && count_sim_==0) {
                is_recording_ = true;
                count_sim_ += 1;
            } else if (msg.data && count_sim > 0){
                // Add function to save the data HERE!!!!!
                is_recording_ = true;
                count_sim_ += 1;
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
            if (last_uav_state_.has_value()){
                // Tansofrm the quaternion in course, fpa, and roll
                gz::math::Quaterniond q(last_uav_state_->pose.pose.orientation.w, last_uav_state_->pose.pose.orientation.x,
                    last_uav_state_->pose.pose.orientation.y, last_uav_state_->pose.pose.orientation.z);
                // Pass it to roll, pitch and yaw:
                gz::math::Vector3d euler = q.Euler();
                double roll_odom = euler.X();
                double pitch_odom = euler.Y();
                double yaw_odom = euler.Z();
                // Define the course, fpa, and roll;
                double course_odom = M_PI/2 - yaw_odom;
                double fpa_odom = -pitch_odom;

                // Pass thelinear veloicyt from the body frame to the world frame (Remmeber the body frame of the odometry is in NEU)
                // that is true for the angular velcoity too.
                gz::math::Vector3d body_vel(last_uav_state_->twist.twist.linear.x, last_uav_state_->twist.twist.linear.y * -1,
                    last_uav_state_->twist.twist.linear.z * -1);
                gz::math::Vector3d world_vel = q.RotateVector(body_vel);

                // Save in the vector:
                Odom_unit odom_unity;
                odom_unity.sim_time = sim_time_;
                odom_unity.real_time = current_real_time;
                odom_unity.pos_east_real = last_uav_state_->pose.pose.position.x;
                odom_unity.pos_north_real = last_uav_state_->pose.pose.position.y;
                odom_unity.pos_up_real = last_uav_state_->pose.pose.position.z;
                odom_unity.course_real = course_odom;
                odom_unity.fpa_real = fpa_odom;
                odom_unity.roll_real = roll_odom;
                odom_unity.vel_east_real = world_vel.X();
                odom_unity.vel_north_real = world_vel.Y();
                odom_unity.vel_up_real = world_vel.Z();
                odom_unity.p = last_uav_state_->twist.twist.angular.x;
                odom_unity.q = last_uav_state_->twist.twist.angular.y;
                odom_unity.r = last_uav_state_->twist.twist.angular.z;   
                odom_history_.push_back(odom_unity);
            }

            // Save the ADS-B:
            if (last_adsb_.has_value()) {
            Adsb_unit adsb_unity;
            adsb_unity.sim_time = sim_time_;
            adsb_unity.real_time = current_real_time;
            adsb_unity.north = last_adsb_->north;
            adsb_unity.east = last_adsb_->east;
            adsb_unity.up = last_adsb_->up;
            adsb_unity.v_north = last_adsb_->v_north;
            adsb_unity.v_east = last_adsb_->v_east;
            adsb_unity.v_up = last_adsb_->v_up;
            adsb_unity.course = last_adsb_->course;
            adsb_unity.fpa = last_adsb_->fpa;
            adsb_unity.roll = last_adsb_->roll;
            adsb_unity.p = last_adsb_->p;
            adsb_unity.q = last_adsb_->q;
            adsb_unity.r = last_adsb_->r;
            adsb_history_.push_back(adsb_unity);
            }
        }
}