/**
 * @file execute_joint_trajectory_handler.h
 * @author Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 * @brief The ROS node that publishes the robot information
 * @version 0.1
 * @date 25-02-2020
 *
 * @copyright (c) 2020 Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 *
 */

#pragma once

#include <boost/scoped_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <limits>
#include <functional>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include <utility>

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "nav2_util/simple_action_server.hpp"

//#include "hardware_interface/robot_hw.hpp"
// ros2_control hardware_interface
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/visibility_control.h"

#include <joint_limits/joint_limits.hpp>
#include <joint_limits/joint_limits_rosparam.hpp>
//#include <joint_limits_interface/joint_limits_urdf.hpp>
#include <urdf/model.h>

// msgs
#include "comau_msgs/msg/action_result_status_constants.hpp"
#include "comau_msgs/action/execute_joint_trajectory.hpp"
#include <std_msgs/msg/bool.hpp>

#include "comau_driver/comau_driver.hpp"

using namespace comau_tcp_interface;
using namespace comau_tcp_interface::utils;

namespace comau_action_handlers {
//typedef actionlib::SimpleActionServer<comau_msgs::ExecuteJointTrajectoryAction> ExecuteJointTrajectoryActionServer;
//typedef rclcpp_action::Server<comau_msgs::action::ExecuteJointTrajectory> ExecuteJointTrajectoryActionServer;

/**
 * @brief This handler accepts as goal a joint space trajectory and send it to
 * Robot with RobotClient
 *
 */
class ExecuteJointTrajectoryHandler {
public:

  using ExecuteJointTrajectory = comau_msgs::action::ExecuteJointTrajectory;
  using GoalHandleExecuteJointTrajectory = rclcpp_action::ServerGoalHandle<ExecuteJointTrajectory>;

  /**
   * @brief Construct a new MoveJointsServer object
   *
   * @param nh
   * @param nh_local
   * @param robot_ptr
   * @param name
   */
  ExecuteJointTrajectoryHandler(const rclcpp::Node::SharedPtr& nh, const rclcpp::Node::SharedPtr& nh_local, const std::string &name,
                                const boost::shared_ptr<comau_driver::ComauRobot> &robot_ptr);

  /**
   * @brief Destroy the Move Joints Handler object
   *
   */
  ~ExecuteJointTrajectoryHandler();
  /**
   * @brief Initialization function
   *
   * @param use_state_server
   * @param use_motion_server
   *
   * @return true
   * @return false if something went wrong at the initialization
   */
  bool initialize(bool use_state_server, bool use_robot_server, bool use_arm1_server);
  void set_status(char &status);
  void set_allow_async(const bool &allow_async);

private:
  rclcpp_action::Server<ExecuteJointTrajectory>::SharedPtr ExecuteJointTrajectoryActionServer;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const ExecuteJointTrajectory::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleExecuteJointTrajectory> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleExecuteJointTrajectory> goal_handle);

  /**
   * @brief Get the URDF XML from the parameter server
   *
   * @param nh
   * @param robot_descr_param_name
   * @return true
   * @return false
   */
  bool loadJointLimits(const rclcpp::Node::SharedPtr& nh, std::string robot_descr_param_name);
  /**
   * @brief A function that check whether the goal command is complete or out of limits
   *
   * @return true if goal is valid
   * @return false if invalid
   */
  bool validateJointLimits(vector10f_t joint_values_array);
  /**
   * @brief Conversion function ROS trajectory_msgs/JointTrajectoryPoint[]
   * message to custom utils::joint_trajectoryf_t
   *
   * @param trajectory
   */ 
  joint_trajectoryf_t parseJointTrajectoryGoal(std::shared_ptr<const comau_msgs::action::ExecuteJointTrajectory::Goal> goal);

  /**
   * @brief A callback function that handles the incoming goal
   *
   * @param goal see comau_msgs::ExecuteJointTrajectoryGoal
   */
  void executeCallback(const std::shared_ptr<GoalHandleExecuteJointTrajectory> goal);
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Node::SharedPtr nh_local_;
  std::string action_name_;

  bool action_active_;
  bool error_;
  bool valid_goal_;
  char robot_status_ = 'R'; // init only for dummy
  joint_trajectoryf_t goal_joint_trajectory_;

  /*ExecuteJointTrajectory::Feedback feedback_;
  ExecuteJointTrajectory::Result result_;*/
  std::shared_ptr<ExecuteJointTrajectory::Feedback> feedback_;
  std::shared_ptr<ExecuteJointTrajectory::Result> result_;

  //boost::shared_ptr<ExecuteJointTrajectoryActionServer> as_ptr_;
  std::shared_ptr<GoalHandleExecuteJointTrajectory> goal_handle_;

  boost::shared_ptr<urdf::Model> urdf_model_ptr_;
  std::vector<double> joint_position_lower_limits_;
  std::vector<double> joint_position_upper_limits_;
  bool use_state_server_, use_robot_server_, use_arm1_server_;
  bool allow_async_ = false;
  // Robot Driver Pointer
  boost::shared_ptr<comau_driver::ComauRobot> robot_ptr_;
public:
  size_t urdf_number_of_joints_;
  std::vector<std::string> joint_names_;
};

} // namespace comau_action_handlers
