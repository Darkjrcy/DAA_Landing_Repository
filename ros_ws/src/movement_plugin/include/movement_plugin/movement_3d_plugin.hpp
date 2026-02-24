#ifndef MOVEMENT_3D_PLUGIN_HPP_
#define MOVEMENT_3D_PLUGIN_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>
#include <mutex>
#include <string>

namespace movement_3d_plugin
{
  class Movement3DPluginPrivate;

  class Movement3DPlugin:public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate, public gz::sim::ISystemPostUpdate
  {
    public:
    //Constructor:
    Movement3DPlugin();
    //Destructor:
    ~Movement3DPlugin() override = default;
    // Start the plugin
    void Configure(const gz::sim::Entity &entity,const std::shared_ptr<const sdf::Element> &sdf,gz::sim::EntityComponentManager &ecm,gz::sim::EventManager &eventMgr) override;
    //Update the velocity:
    void PreUpdate(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &ecm) override;
    // Publisht eh odometry:
    void PostUpdate(const gz::sim::UpdateInfo &_info,const gz::sim::EntityComponentManager &_ecm) override;

  private:
    std::unique_ptr<Movement3DPluginPrivate> impl_;
  };
}

#endif  // MOVEMENT_3D_PLUGIN_HPP_
