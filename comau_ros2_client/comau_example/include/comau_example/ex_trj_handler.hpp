/**
 * @file ex_trj_handler.hpp
 * @author Comau Robotics S.p.A.
 * @brief The ROS2 node that publishes example of trajectories
 * @version 1.0
 * @date 02/07/2024
 *
 * @copyright (c) Comau Robotics S.p.A.
 *
 */

#include <csignal>
#include <boost/scoped_ptr.hpp>
#include <comau_driver/comau_driver.hpp>
#include "comau_handlers/execute_joint_trajectory_handler.hpp"
#include "comau_handlers/execute_cartesian_trajectory_handler.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.h"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include <comau_msgs/msg/comau_robot_status.hpp>
#include <comau_msgs/msg/comau_server_error.hpp>
#include <comau_msgs/msg/comau_operation_mode.hpp>
#include <comau_msgs/msg/digital.hpp>
#include <comau_msgs/msg/io_states.hpp>
//#include <dynamic_reconfigure/server.h>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <comau_msgs/srv/open_connection.hpp>
#include <comau_msgs/srv/set_move_fly_params.hpp>
#include <comau_msgs/srv/set_arm_state.hpp>
#include <comau_msgs/srv/set_io.hpp>
#include <comau_msgs/srv/set_sns_trk_params.hpp>
#include <std_msgs/msg/int32.hpp>
#include <time.h>
#include <chrono>

namespace ex_trj_handler {
using JntTraj           = comau_msgs::action::ExecuteJointTrajectory;
using GoalHandleJntTraj = rclcpp_action::ClientGoalHandle<JntTraj>;
using CartTraj           = comau_msgs::action::ExecuteCartesianTrajectory;
using GoalHandleCartTraj = rclcpp_action::ClientGoalHandle<CartTraj>;

class TrajectoryHandler {
public:

  TrajectoryHandler(rclcpp::Node::SharedPtr &nh);

  virtual ~TrajectoryHandler() = default;

  virtual bool init();

  void sendCartTraj();

  void send_goal();

  //void cancel_goal(); /* ANDY */

  void goal_response_callback(const GoalHandleCartTraj::SharedPtr & goal_handle);

  void feedback_callback(GoalHandleCartTraj::SharedPtr, const std::shared_ptr<const CartTraj::Feedback> feedback);

  void result_callback(const GoalHandleCartTraj::WrappedResult & result);

  void sendJntTraj();

  void send_jnt_goal();

  void jnt_goal_response_callback(const GoalHandleJntTraj::SharedPtr & goal_handle);

  void jnt_feedback_callback(GoalHandleJntTraj::SharedPtr, const std::shared_ptr<const JntTraj::Feedback> feedback);

  void jnt_result_callback(const GoalHandleJntTraj::WrappedResult & result);

  rclcpp_action::Client<comau_msgs::action::ExecuteCartesianTrajectory>::SharedPtr client_ptr_;
  rclcpp_action::Client<comau_msgs::action::ExecuteJointTrajectory>::SharedPtr client_jnt_ptr_;

  GoalHandleCartTraj::WrappedResult result_;

protected:

  std::string name_;                             
  rclcpp::Node::SharedPtr nh_, nh_priv_;

  std::vector<double> urdf_command_;
};

} // namespace comau_hardware_interface
