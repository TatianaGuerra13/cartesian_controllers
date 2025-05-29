// Velocity controller for Cartesian motion using IK solver

#include <cartesian_motion_controller/cartesian_motion_controller.h>
#include <algorithm>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/float64.hpp>
#include "cartesian_controller_base/Utility.h"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/clock.hpp"

namespace cartesian_motion_controller
{

using Base = cartesian_controller_base::CartesianControllerBase;

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianMotionController::on_init()
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianMotionController::on_configure(const rclcpp_lifecycle::State &)
{
  // Creazione del subscriber per i comandi del decoder
  m_decoder_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/decoder_output", 10,
    std::bind(&CartesianMotionController::decoderCommandCallback, this, std::placeholders::_1));

  // Creazione del publisher per il grasp
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
  // Aggiorna le posizioni correnti dei giunti
  Base::m_ik_solver->synchronizeJointPositions(Base::m_joint_state_pos_handles);

  // Acquisizione thread-safe dell'ultimo comando dal decoder
  std::array<double, 7> cmd;
  {
    std::lock_guard<std::mutex> lock(m_command_mutex);
    cmd = m_latest_command;
  }

  // Costruisce il Twist con le prime 6 componenti (linear + angular)
  KDL::Twist twist_cmd(
    KDL::Vector(cmd[0], cmd[1], cmd[2]),
    KDL::Vector(cmd[3], cmd[4], cmd[5]));

  // Conversione KDL::Twist in ctrl::Vector6D (Eigen)
  ctrl::Vector6D motion_error;
  motion_error << twist_cmd.vel.x(), twist_cmd.vel.y(), twist_cmd.vel.z(),
                  twist_cmd.rot.x(), twist_cmd.rot.y(), twist_cmd.rot.z();

  // Esegue il controllo per generare velocità articolari
  Base::computeJointControlCmds(motion_error, period);

  // Pubblica comando grasp (cmd[6])
  if (m_grasp_pub_ && cmd.size() >= 7)
  {
    std_msgs::msg::Float64 grasp_msg;
    grasp_msg.data = cmd[6];
    m_grasp_pub_->publish(grasp_msg);
  }

  return controller_interface::return_type::OK;
}

}  // namespace cartesian_motion_controller

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cartesian_motion_controller::CartesianMotionController,
                       controller_interface::ControllerInterface)
