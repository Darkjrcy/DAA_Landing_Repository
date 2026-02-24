// ROS2 Library:
#include <rclcpp/rclcpp.hpp>
#include "movement_plugin/movement_3d_plugin.hpp"
// ROS2 messages:
#include <geometry_msgs/msg/twist.hpp>
// TF Transforms:
#include <tf2_ros/transform_broadcaster.h>
// Gazebo Harmonic plugin integration:
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Pose.hh> 
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
// Protect the ROS2 velocity publishing:
#include <mutex> 
// Add the string information type:
#include <string>
// math library:
#include <iostream>
#include <cmath> 




// Start the 3D movement plugin:
namespace movement_3d_plugin{
    // Create teh private class with all the components:
    class Movement3DPluginPrivate{
        public:
            // Topics used by ROS2:
            rclcpp::Node::SharedPtr ros_node_;
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
            std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

            // Messages to save velocity and protects the information oversubscription:
            geometry_msgs::msg::Twist target_cmd_vel_;
            std::mutex cmd_mutex_;

            // Strat the default configuration of the plugin:
            // Model name
            std::string states_frame_ = "airplane1";
            // Robot base link where the velocity is applied:
            std::string robot_base_frame_ = "base_link";

            // Obtain the Gazebo model from the plugin when its applied in sdf:
            gz::sim::Entity model_entity_{gz::sim::kNullEntity};
            gz::sim::Model model_{gz::sim::kNullEntity};

            // Functions used during the publish of the command to publish the states of the airplane:
            void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg);
    };



    // Constructor definition:
    Movement3DPlugin::Movement3DPlugin()
    : impl_(std::make_unique<Movement3DPluginPrivate>())
    {
    }



    // Function that starts the plugin when its configured:
    void Movement3DPlugin::Configure(const gz::sim::Entity &_entity,const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,gz::sim::EventManager &){
        (void)_ecm;
        // Store the encity and the model:
        this->impl_->model_entity_ = _entity;
        this->impl_->model_ = gz::sim::Model(_entity);

        // check if ros2 is working if not initilize it:
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }

        // Get the states frame:
        if (_sdf->HasElement("statesFrame")) {
            impl_->states_frame_ = _sdf->Get<std::string>("statesFrame");
        }
        // Get the robot base frame
        if (_sdf->HasElement("robotBaseFrame")){
            impl_->robot_base_frame_ = _sdf->Get<std::string>("robotBaseFrame");
        }

        // Create a ROS 2 node with name based on the state_frame:
        rclcpp::NodeOptions options;
        options.arguments({ "--ros-args", "-r", "__ns:=/" + impl_->states_frame_ });
        impl_->ros_node_ = std::make_shared<rclcpp::Node>("movement_3d_node", options);
        RCLCPP_WARN(impl_->ros_node_->get_logger(), "ROS 2 node movemendt initialized.");
        std::thread([node = impl_->ros_node_]() {
            rclcpp::executors::SingleThreadedExecutor exec;
            exec.add_node(node);
            exec.spin();
        }).detach();

        // Create the Transfer Function BroadCaster
        impl_->transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);

        // Define the topic that is subscribed to obtain the cmd_vel:
        std::string cmd_vel_topic = "cmd_vel";
        if (_sdf->HasElement("cmdVelTopic")){
            cmd_vel_topic = _sdf->Get<std::string>("cmdVelTopic");
        }

        // Start the subscriber to command the velocity:
        impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
            cmd_vel_topic,rclcpp::SystemDefaultsQoS(),std::bind(&Movement3DPluginPrivate::OnCmdVel,impl_.get(), std::placeholders::_1));
        RCLCPP_INFO(impl_->ros_node_->get_logger(), "Subscribed to [%s]", cmd_vel_topic.c_str());

        // Print the characteristics to check them in the command window:
        RCLCPP_INFO(impl_->ros_node_->get_logger(),"Publishing states transforms between [%s] and [%s]", impl_->states_frame_.c_str(),impl_->robot_base_frame_.c_str());
    }



    // Read the velocity command message
    void Movement3DPluginPrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
    {
        std::lock_guard<std::mutex> lock(this->cmd_mutex_);
        this->target_cmd_vel_ = *_msg;
    }



    // PreUpdate Process before the inforamtion is updated in the gazebo plugin:
    void Movement3DPlugin::PreUpdate( const gz::sim::UpdateInfo &_info,gz::sim::EntityComponentManager &_ecm){
        (void)_info;
        // Stop if the model doesn't exists:
        if (impl_->model_entity_ == gz::sim::kNullEntity){
            return;
        }

         // Read the comanded speed:
        geometry_msgs::msg::Twist cmd_vel;
        {
            std::lock_guard<std::mutex> lock(impl_->cmd_mutex_);
            cmd_vel = impl_->target_cmd_vel_;
        }

        // Obtain the actural rotational angles in the ZYX Euler Rotation
        const auto pose_comp = _ecm.Component<gz::sim::components::Pose>(impl_->model_entity_);
        // Return in case the pose is not avaliable:
        if (!pose_comp){
            return;
        }
        // Rotate local linear velocity in the world frame:
        gz::math::Vector3d local_linear_vel(cmd_vel.linear.x,cmd_vel.linear.y,cmd_vel.linear.z);
        gz::math::Vector3d local_angular_vel(cmd_vel.angular.x,cmd_vel.angular.y,cmd_vel.angular.z);

        // Publish the linear velocities:
        if (_ecm.Component<gz::sim::components::LinearVelocityCmd>(impl_->model_entity_)!= nullptr)
        {
            _ecm.SetComponentData<gz::sim::components::LinearVelocityCmd>(impl_->model_entity_, local_linear_vel);
        }
        else
        {
            _ecm.CreateComponent(impl_->model_entity_,gz::sim::components::LinearVelocityCmd(local_linear_vel));
        }
        // Publish the angular velocities:
        if (_ecm.Component<gz::sim::components::AngularVelocityCmd>(impl_->model_entity_)!= nullptr)
        {
            _ecm.SetComponentData<gz::sim::components::AngularVelocityCmd>(impl_->model_entity_, local_angular_vel);
        }
        else
        {
            _ecm.CreateComponent(impl_->model_entity_,gz::sim::components::AngularVelocityCmd(local_angular_vel));
        }

    }

}



// Create teh plugin:
GZ_ADD_PLUGIN(
  movement_3d_plugin::Movement3DPlugin,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)

// if you really want an alias, it has to be a string:
GZ_ADD_PLUGIN_ALIAS(
  movement_3d_plugin::Movement3DPlugin,
  "movement_3d_plugin")