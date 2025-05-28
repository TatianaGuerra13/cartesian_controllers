// cartesian_velocity_controller.cpp
#include <cartesian_motion_controller/cartesian_velocity_controller.hpp>
 
#include <algorithm>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <std_msgs/msg/float64.hpp>
 
namespace cartesian_motion_controller
{
 
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianMotionController::on_init()
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
 
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianMotionController::on_configure(const rclcpp_lifecycle::State &)
{
  m_decoder_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/decoder_output", 10,
    std::bind(&CartesianMotionController::decoderCommandCallback, this, std::placeholders::_1));
 
  m_grasp_pub_ = get_node()->create_publisher<std_msgs::msg::Float64>(
    "/grasp_command", 10);
 
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
 
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianMotionController::on_activate(const rclcpp_lifecycle::State &)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
 
rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianMotionController::on_deactivate(const rclcpp_lifecycle::State &)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}
 
void CartesianMotionController::decoderCommandCallback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (!this->isActive() || msg->data.size() < 6) return;
  std::lock_guard<std::mutex> lock(m_command_mutex);
  std::copy_n(msg->data.begin(), std::min(msg->data.size(), size_t(7)), m_latest_command.begin());
}
 
controller_interface::return_type CartesianMotionController::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & period)
{
  Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_state_pos_handles);
 
  std::array<double, 7> cmd;
  {
    std::lock_guard<std::mutex> lock(m_command_mutex);
    cmd = m_latest_command;
  }
 
  KDL::Twist desired_twist(
    KDL::Vector(cmd[0], cmd[1], cmd[2]),
    KDL::Vector(cmd[3], cmd[4], cmd[5]));
 
  KDL::JntArray joint_velocities;
  if (!Base::m_ik_solver->CartToJntVel(desired_twist, joint_velocities))
  {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                         "Failed IK velocity solve.");
    return controller_interface::return_type::OK;
  }
 
  for (size_t i = 0; i < joint_velocities.rows(); ++i)
  {
    Base::m_joint_command_handles[i].setCommand(joint_velocities(i));
  }
 
  // Publish grasp command if available
  // The cmd[6] is the grasp command
  
  if (m_grasp_pub_ && cmd.size() >= 7)
  {
    std_msgs::msg::Float64 grasp_msg;
    grasp_msg.data = cmd[6];
    m_grasp_pub_->publish(grasp_msg);
  }
 
  return controller_interface::return_type::OK;
}
 
}  // namespace cartesian_motion_controller
 
PLUGINLIB_EXPORT_CLASS(cartesian_motion_controller::CartesianMotionController,
                       controller_interface::ControllerInterface)