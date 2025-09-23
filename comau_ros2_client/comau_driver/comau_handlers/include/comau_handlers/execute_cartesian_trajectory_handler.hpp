/**
 * @file execute_cartesian_trajectory_handler.hpp
 * @author Comau Robotics S.p.A.
 * @brief The ROS2 node that publishes the cartesian trajectory action
 * @version 1.0
 * @date 02/07/2024
 *
 * @copyright (c) Comau Robotics S.p.A.
 *
 */

#pragma once

#include <boost/scoped_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <limits>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include <utility>
#include <chrono>
#include <cstdlib>

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "hardware_interface/system_interface.hpp"
// msgs
#include "comau_msgs/msg/action_result_status_constants.hpp"
#include "comau_msgs/action/execute_cartesian_trajectory.hpp"
#include <comau_msgs/msg/cartesian_pose_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
// tf
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "comau_driver/comau_driver.hpp"

using namespace comau_tcp_interface;
using namespace comau_tcp_interface::utils;

namespace comau_action_handlers {

typedef rclcpp_action::Server<comau_msgs::action::ExecuteCartesianTrajectory> ExecuteCartesianTrajectoryActionServer;
//typedef actionlib::SimpleActionServer<comau_msgs::ExecuteCartesianTrajectoryAction> ExecuteCartesianTrajectoryActionServer;

/**
 * @brief This handler accepts as goal a cartesian space trajectory and send it to
 * Robot with RobotClient
 *
 */
class ExecuteCartesianTrajectoryHandler {
public:
  /**
   * @brief Construct a new MoveCartesianServer object
   *
   * @param nh
   * @param nh_local
   * @param robot_ptr
   * @param name
   */
  ExecuteCartesianTrajectoryHandler(const rclcpp::Node::SharedPtr &nh, const rclcpp::Node::SharedPtr &nh_local, const std::string &name,
                                    const boost::shared_ptr<comau_driver::ComauRobot> &robot_ptr);
  /**
   * @brief Destroy the Move Cartesian Handler object
   *
   */
  ~ExecuteCartesianTrajectoryHandler();
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
  /**
   * @brief Conversion function ROS geometry_msgs/PoseStamped[]
   * message to custom utils::trajectoryf_t
   *
   * @param cartesian_trajectory
   */
  trajectoryf_t parseCartesianTrajectoryGoal(const std::shared_ptr<const comau_msgs::action::ExecuteCartesianTrajectory::Goal> goal);

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const comau_msgs::action::ExecuteCartesianTrajectory::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<comau_msgs::action::ExecuteCartesianTrajectory>> &goal_handle);

  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<comau_msgs::action::ExecuteCartesianTrajectory>> &goal_handle);
  /**
   * @brief A callback function that handles the incoming goal
   *
   * @param goal see comau_msgs::ExecuteCartesianTrajectoryGoal
   */
  void executeCallback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<comau_msgs::action::ExecuteCartesianTrajectory>> &goal_handle);
  /**
   * @brief A function that transforms the pose goal relative to another frame
   *
   */
  geometry_msgs::msg::PoseStamped changePoseFrame(const std::string &target_frame, const geometry_msgs::msg::PoseStamped &goal_pose);

  rclcpp::Node::SharedPtr nh_;
  rclcpp::Node::SharedPtr nh_local_;
  std::string action_name_;

  bool action_active_;
  bool error_;

  /*comau_msgs::ExecuteCartesianTrajectoryFeedback feedback_;
  comau_msgs::ExecuteCartesianTrajectoryResult result_;*/
  std::shared_ptr<comau_msgs::action::ExecuteCartesianTrajectory::Feedback> feedback_;
  std::shared_ptr<comau_msgs::action::ExecuteCartesianTrajectory::Result> result_;
  rclcpp_action::Server<comau_msgs::action::ExecuteCartesianTrajectory>::SharedPtr action_server_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<comau_msgs::action::ExecuteCartesianTrajectory>> goal_handle_;

  char robot_status_ = 'R'; // only initialized for dummy

  //boost::shared_ptr<ExecuteCartesianTrajectoryActionServer> as_ptr_;
  trajectoryf_t goal_cartesian_trajectory_;
  bool use_state_server_, use_robot_server_, use_arm1_server_;
  bool allow_async_ = false;
  // Robot Driver Pointer
  boost::shared_ptr<comau_driver::ComauRobot> robot_ptr_;
};

} // namespace comau_action_handlers
