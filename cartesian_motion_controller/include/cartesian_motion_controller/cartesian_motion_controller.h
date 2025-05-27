#ifndef CARTESIAN_MOTION_CONTROLLER_H_INCLUDED
#define CARTESIAN_MOTION_CONTROLLER_H_INCLUDED

#include <cartesian_controller_base/ROS2VersionConfig.h>
#include <cartesian_controller_base/cartesian_controller_base.h>

#include <controller_interface/controller_interface.hpp>

#include <std_msgs/msg/float64_multi_array.hpp>

namespace cartesian_motion_controller

{

class CartesianMotionController : public virtual cartesian_controller_base::CartesianControllerBase
{
public:
  CartesianMotionController() = default;
  
  virtual ~CartesianMotionController() = default;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(const rclcpp::Time & time,

                                           const rclcpp::Duration & period) override;

private:

  void decoderCommandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_decoder_sub_;
  std::array<double, 7> m_latest_command{};
  std::mutex m_command_mutex;

};

}  // namespace cartesian_motion_controller

#endif
 