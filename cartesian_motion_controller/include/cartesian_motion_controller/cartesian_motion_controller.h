#ifndef CARTESIAN_MOTION_CONTROLLER_H
#define CARTESIAN_MOTION_CONTROLLER_H


#include <array>
#include <mutex>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "cartesian_controller_base/cartesian_controller_base.h"

namespace cartesian_motion_controller
{

class CartesianMotionController : public cartesian_controller_base::CartesianControllerBase
{
public:
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init() override;
  
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void decoderCommandCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  std::array<double, 7> m_latest_command{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::mutex m_command_mutex;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr m_decoder_sub_;
};

}  // namespace cartesian_motion_controller
#endif  // CARTESIAN_MOTION_CONTROLLER_H