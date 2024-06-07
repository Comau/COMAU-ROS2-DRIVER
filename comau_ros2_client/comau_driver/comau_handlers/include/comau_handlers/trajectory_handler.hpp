/**
 * @file comau_hw_interface.h
 * @author Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 * @brief The ROS node that publishes the robot information
 * @version 0.1
 * @date 25-02-2020
 *
 * @copyright (c) 2020 Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 *
 */

#include <csignal>
#include <boost/scoped_ptr.hpp>
#include <comau_driver/comau_driver.hpp>
#include "comau_handlers/execute_joint_trajectory_handler.hpp"
#include "comau_handlers/execute_cartesian_trajectory_handler.hpp"

namespace trajectory_handler {
using CartTraj           = comau_msgs::action::ExecuteCartesianTrajectory;
using GoalHandleCartTraj = rclcpp_action::ClientGoalHandle<CartTraj>;
/**
 * @brief The ComauHardwareInterface class handles the interface between the ROS system and the main
 * driver. It contains the read and write methods of the main control loop and registers various ROS
 * topics and services.
 */
class TrajectoryHandler {
public:
  /**
   * @brief Construct a new Comau Hardware Interface object
   * @param nh Root level ROS node handle
   */
  TrajectoryHandler(rclcpp::Node::SharedPtr &nh);
  /**
   * @brief Default Destructor for the Comau Hardware Interface object
   */
  virtual ~TrajectoryHandler() = default;
  /**
   * @brief Handles the setup functionality for the ROS interface. This includes parsing ROS
   * parameters, creating interfaces, starting the main driver and advertising ROS services.
   *
   * @returns True, if the setup was performed successfully
   *
   */
  virtual bool init();
  /**
   * @brief Read method of the control loop. Reads a messages from the robot and handles and
   * publishes the information as needed.
   *
   * @param time Current time
   * @param period Duration of current control loop iteration
   */
  virtual void read();
  
  virtual char publishRobotStatus();
  
  virtual void write(double time, double period);

  bool holdConnection();
  
  void printVector(const std::vector<double> &vec);

  // Update funcion called with loop_hz_ rate
  void update();

  void sendCartTraj();

  void send_goal();

  void goal_response_callback(const GoalHandleCartTraj::SharedPtr & goal_handle);

  void feedback_callback(GoalHandleCartTraj::SharedPtr, const std::shared_ptr<const CartTraj::Feedback> feedback);

  void result_callback(const GoalHandleCartTraj::WrappedResult & result);
  
  /*void publishEndEffectorPose();
  
  void publishIOPins();
  
  void publishRobotStatus();
  
  void publishOperationMode();
  
  void publishErrorValue();
  
  void publishSnsTrkType();
  
  bool holdConnection();
  
  void printVector(const std::vector<double> &vec);
  
  bool ifZero(const std::vector<double> &vec);
  
  void copyVector(const std::vector<double> &src, std::vector<double> &dest);
  
  bool shouldResetControllers();
  
  virtual bool prepareSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                             const std::list<hardware_interface::ControllerInfo> &stop_list) override;
  
  virtual void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                        const std::list<hardware_interface::ControllerInfo> &stop_list) override;

  
  bool checkControllerClaims(const std::set<std::string> &claimed_resources);
  
  bool isRobotProgramRunning() const;
  
  void closeComauDriver();
  
  void errorParser(uint32_t error_value);*/
  std::unique_ptr<comau_action_handlers::ExecuteJointTrajectoryHandler>
      execute_joints_handler_ptr;

  boost::shared_ptr<comau_driver::ComauRobot> robot_ptr_; /**< Robot driver object pointer */

  std::unique_ptr<comau_action_handlers::ExecuteCartesianTrajectoryHandler>
      execute_cartesian_handler_ptr; /**< Object for asynchronous cartesian trajectory action server */

  rclcpp_action::Client<comau_msgs::action::ExecuteCartesianTrajectory>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

protected:
  
  /*bool setIO_routine(comau_msgs::SetIO::Request &req, comau_msgs::SetIO::Response &res);
  
  bool threadConnections(comau_msgs::OpenConnection::Request &req, comau_msgs::OpenConnection::Response &res);
  
  bool setMoveflyParams_routine(comau_msgs::SetMoveFlyParams::Request &req, comau_msgs::SetMoveFlyParams::Response &res);
  
  bool setSnsTrkParams_routine(comau_msgs::SetSnsTrkParams::Request &req, comau_msgs::SetSnsTrkParams::Response &res);
  
  bool setArmState_routine(comau_msgs::SetArmState::Request &req, comau_msgs::SetArmState::Response &res);*/

  std::string name_;                                      /**< Name of this class -> comau_hardware_interface */
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
  std::vector<int> pins_state_in_;
  std::vector<int> pins_out_;
  std::vector<int> pins_state_out_;
  // Commands
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocity_command_;
  std::vector<double> joint_effort_command_;
  std::vector<double> sensor_tracking_command_;
  // private
  uint32_t num_joints_;
  uint32_t num_robot_joints_;
  bool initialization_; 
  std::vector<std::string> joint_names_;
  std::vector<int> jnt_type_;
  uint32_t stsSelector_;
  uint64_t loop_hz_;
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

  // comau action handlers
  const std::string execute_joint_server_name_     = "execute_joint_trajectory_handler";
  const std::string execute_cartesian_server_name_ = "execute_cartesian_trajectory_handler";
};

} // namespace comau_hardware_interface
