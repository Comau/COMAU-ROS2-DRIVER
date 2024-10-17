/**
 * @file trajectory_handler.hpp
 * @author Comau Robotics S.p.A.
 * @brief The ROS2 node that publishes the trajectory action
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

/* Server error value - server coding */
#define KI_ERR_STATE_ACCEPT       0x00001 /* State  Server  error 15470 : Address already in use */
#define KI_ERR_ROBOT_ACCEPT       0x00002 /* Robot  Server  error 15470 : Address already in use */
#define KI_ERR_HANDLER_ACCEPT     0x00004 /* Motion Handler error 15470 : Address already in use */
#define KI_ERR_STATE_WRITE        0x00008 /* State  Server  error 40033 : Error 15474 in write   */
#define KI_ERR_ROBOT_READ_NCON    0x00010 /* Robot  Server  error 39990 : Error 15467 in read    */
#define KI_ERR_HANDLER_READ_NCON  0x00020 /* Motion Handler error 39990 : Error 15467 in read    */
#define KI_ERR_ROBOT_READ_DCON    0x00040 /* Robot  Server  error 39990 : Error 15468 in read    */
#define KI_ERR_HANDLER_READ_DCON  0x00080 /* Motion Handler error 39990 : Error 15468 in read    */
#define KI_ERR_ROBOT_READ_CANC    0x00100 /* Robot  Server  error 39991 */
#define KI_ERR_HANDLER_READ_CANC  0x00200 /* Motion Handler error 39991 */
#define KI_ERR_ROBOT_READ_TOUT    0x00400 /* Robot  Server  error 39992 */
#define KI_ERR_HANDLER_READ_TOUT  0x00800 /* Motion Handler error 39992 */
#define KI_ERR_STATE_DISCONNECT   0x01000 /* State  Server  error 30767 */
#define KI_ERR_ROBOT_DISCONNECT   0x02000 /* Robot  Server  error 30767 */
#define KI_ERR_HANDLER_DISCONNECT 0x04000 /* Motion Handler error 30767 */
#define KI_ERR_STATE_SAFETY_GATE  0x08000 /* Safety Gate / External Emergency Stop */
#define KI_ERR_STATE_WRONG_MOTION 0x10000 /* Program Execution Errors (36864-37191) */
#define KI_ERR_ROBOT_ALARM        0x20000 /*  */
#define KI_ERR_MOTION_DRIVEOFF    0x40000 /*  */
#define KI_ERR_STATE_MAX          0x80000 /* Max error value - increase it if necessary */

namespace trajectory_handler {
using JntTraj           = comau_msgs::action::ExecuteJointTrajectory;
using GoalHandleJntTraj = rclcpp_action::ClientGoalHandle<JntTraj>;
using CartTraj           = comau_msgs::action::ExecuteCartesianTrajectory;
using GoalHandleCartTraj = rclcpp_action::ClientGoalHandle<CartTraj>;

class TrajectoryHandler {
public:

  TrajectoryHandler(rclcpp::Node::SharedPtr &nh);

  virtual ~TrajectoryHandler() = default;

  virtual bool init();
  
  void publishRobotStatus();

  void errorParser(uint32_t error_value);

  void publishErrorValue();

  void publishEndEffectorPose();

  void publishIOPins();

  void publishOperationMode();

  void async_enable_routine();

  void copyVector(const std::vector<double> &src, std::vector<double> &dest);

  void closeComauDriver();
  
  void read(double time, double period);

  void write(double time, double period);

  bool holdConnection();
  
  void printVector(const std::vector<double> &vec);

  void update_urdf();

  void update();

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
  
  std::unique_ptr<comau_action_handlers::ExecuteJointTrajectoryHandler>
      execute_joints_handler_ptr;

  boost::shared_ptr<comau_driver::ComauRobot> robot_ptr_; /**< Robot driver object pointer */

  std::unique_ptr<comau_action_handlers::ExecuteCartesianTrajectoryHandler>
      execute_cartesian_handler_ptr; /**< Object for asynchronous cartesian trajectory action server */

  rclcpp_action::Client<comau_msgs::action::ExecuteCartesianTrajectory>::SharedPtr client_ptr_;
  rclcpp_action::Client<comau_msgs::action::ExecuteJointTrajectory>::SharedPtr client_jnt_ptr_;

  GoalHandleCartTraj::WrappedResult result_;
  
  double desired_update_period_;
  double elapsed_time_;
  uint64_t loop_hz_;
  double cycle_time_error_threshold_;
  struct timespec last_time_;
  struct timespec current_time_;

protected:

  std::string name_;                             
  rclcpp::Node::SharedPtr nh_, nh_priv_;

  // Configuration
  bool position_controller_running_;
  bool velocity_controller_running_;
  bool sensor_tracking_controller_running_;
  bool controllers_initialized_;
  bool robot_program_running_ = true; /* TODO */
  // States
  std::vector<double> joint_position_;
  std::vector<double> joint_velocity_;
  std::vector<double> joint_effort_;
  std::vector<double> ee_position_;
  std::vector<int> pins_in_;
  std::vector<int64_t> pins_state_in_;
  std::vector<int> pins_out_;
  std::vector<int64_t> pins_state_out_;
  // Commands
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_effort_command_;
  //std::vector<double> sensor_tracking_command_;
  std::vector<double> urdf_command_; 
  // private
  uint32_t num_joints_;
  uint32_t num_robot_joints_;
  bool initialization_; 
  std::vector<std::string> joint_names_;
  std::vector<int> jnt_type_;
  uint32_t stsSelector_;
  bool use_state_server_, use_robot_server_, use_arm1_server_;
  int32_t data_timestamp_;
  int32_t data_timestamp_prev_;
  int32_t counter_;
  int32_t sns_trk_type_;
  char robot_status_;
  bool robot_reset_;
  uint32_t error_value_;            /* Error value coming from server via Tcp/IP */
  uint32_t error_value_prev_;       /* Memory of error value coming from server via Tcp/IP */   // VE_ADD for debug
  uint32_t error_value_clientcode_; /* Error value coming from server with client coding */
  bool packet_read_;
  bool verbose_;
  uint32_t invalidMsgCount_;
  geometry_msgs::msg::TransformStamped ee_transform_;
  sensor_msgs::msg::JointState urdf_joint_states_;
  tf2::Quaternion q;

  // publishers
  std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>> async_enable_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<comau_msgs::msg::ComauRobotStatus>> robot_status_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<comau_msgs::msg::ComauServerError>> server_error_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<comau_msgs::msg::ComauOperationMode>> server_operation_mode_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> ee_pose_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>> urdf_joint_states_pub_;
  std::unique_ptr<realtime_tools::RealtimePublisher<comau_msgs::msg::IOStates>> io_states_pub_;
  
  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr urdf_joint_states_sub_;

  // services
  rclcpp::Service<comau_msgs::srv::SetMoveFlyParams>::SharedPtr setMoveflyParams_service_;
  rclcpp::Service<comau_msgs::srv::SetIO>::SharedPtr setIO_service_;
  rclcpp::Service<comau_msgs::srv::SetArmState>::SharedPtr setArmState_service_;
  rclcpp::Service<comau_msgs::srv::OpenConnection>::SharedPtr thread_service_;

  bool req_prev_open_connection;
  bool print_server_not_connected;

  // comau action handlers
  const std::string execute_joint_server_name_     = "execute_joint_trajectory_handler";
  const std::string execute_cartesian_server_name_ = "execute_cartesian_trajectory_handler";
};

} // namespace comau_hardware_interface
