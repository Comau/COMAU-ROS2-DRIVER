/**
 * @file comau_hw_interface.hpp
 * @author Comau Robotics S.p.A.
 * @brief The ROS2 node that publishes hardware interface
 * @version 1.0
 * @date 02/07/2024
 *
 * @copyright (c) Comau Robotics S.p.A.
 *
 */

#ifndef ROS2_COMAU_HARDWARE_INTERFACE__ROBOT_HARDWARE_HPP_
#define ROS2_COMAU_HARDWARE_INTERFACE__ROBOT_HARDWARE_HPP_

#include "string"
#include "unordered_map"
#include "vector"

#include "rclcpp/rclcpp.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

using hardware_interface::return_type;

namespace comau_hardware_interface
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HARDWARE_INTERFACE_PUBLIC ComauHardwareInterface : public hardware_interface::SystemInterface
{
  public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

protected:
  /// The size of this vector is (standard_interfaces_.size() x nr_joints)
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocities_command_;
  std::vector<double> joint_position_;
  std::vector<double> joint_velocities_;
  std::vector<double> ft_states_;
  std::vector<double> ft_command_;

  std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {
    {"position", {}}, {"velocity", {}}};
};

}  

#endif
