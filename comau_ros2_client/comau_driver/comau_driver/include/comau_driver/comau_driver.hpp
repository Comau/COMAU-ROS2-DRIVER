/**
 * @file comau_driver.hpp
 * @author Comau Robotics S.p.A.
 * @brief The ROS2 node that publishes the robot information
 * @version 1.0
 * @date 02/07/2024
 *
 * @copyright (c) Comau Robotics S.p.A.
 *
 */

#ifndef COMAU_DRIVER_HPP
#define COMAU_DRIVER_HPP

#include "comau_tcp_interface/comau_robot_client.hpp"
#include "comau_tcp_interface/comau_state_client.hpp"
#include "comau_tcp_interface/comau_tcp_interface.hpp"
#include <vector>
#include <boost/scoped_ptr.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace comau_tcp_interface;
using namespace comau_tcp_interface::utils;

namespace comau_driver {
/*!
 * \brief Possible states for robot motion command's
 */
enum class ControlMode {
  MODE_POSITION,
  MODE_VELOCITY,
  MODE_JOG,
  MODE_SENSOR_TRACKING,
  MODE_JOINT_TRAJECTORY,
  MODE_CARTESIAN_TRAJECTORY
};


class MoveType{
public:
  inline static const uint32_t JOINT    = 10;
  inline static const uint32_t LINEAR   = 11;
  inline static const uint32_t CIRCULAR = 12;
  inline static const uint32_t SEG_VIA  = 13;
};

/**
 * @brief This is the main class for interfacing the Robot controller.
 * It sets up all the neacessary socket connections and handles the data exchange with the robot.
 */
class ComauRobot {
public:
  /**
   * @brief Construct a new Comau Robot object
   * @param nh global ROS NodeHandle
   */
  ComauRobot(rclcpp::Node::SharedPtr &nh);
  /**
   * @brief Destroy the Comau Robot object
   */
  ~ComauRobot();
  /**
   * @brief Initializes the robot driver
   * @param use_state_server if we are using the state server
   * @param use_motion_server if we are using the motion server
   * @return true If comau robot initialized correctly
   */
  bool initialize(bool use_state_server, bool use_robot_server, bool use_arm1_server);

  // ================ READ =================
  /**
   * @brief Getter for the robot Message Package
   * @return true
   * @return false
   */
  bool readMessagePackage();
  /**
   * @brief Get the default linear velocity
   * @param 
   */
  float getDefaultLinVel();
  /**
   * @brief Get the Joint State using the state client
   * @param joint_position current joint positions
   */
  void getJointPosition(std::vector<double> &joint_position, uint32_t &num_robot_joints, std::vector<int> &jnt_type);
  /**
   * @brief Get the End Effector Position using the state client
   * @param ee_position current end effector position
   */
  void getEePosition(std::vector<double> &ee_position);
  /**
   * @brief Get the DIN pins
   * @param pins_in vector holding the DIN pins
   */
  void getPinsIN(std::vector<int> &pins_in);
  /**
   * @brief Get the DIN pins state
   * @param pins_state_in vector holding the DIN pins state
   */
  void getPinsStatesIN(std::vector<int64_t> &pins_state_in);
  /**
   * @brief Get the DOUT pins
   * @param pins_out vector holding the DOUT pins
   */
  void getPinsOUT(std::vector<int> &pins_out);
  /**
   * @brief Get the DOUT pins state
   * @param pins_state_out vector holding the DOUT pins state
   */
  void getPinsStatesOUT(std::vector<int64_t> &pins_state_out);
  /**
   * @brief Get the current Timestamp
   *
   * @param timestamp current timestamp of the state message
   */
  void getTimeStamp(int32_t &timestamp);
  /**
   * @brief Get the current Robot Status
   *
   * @param status current Robot Status
   */
  void getStatus(char &status);
  /**
   * @brief Get the current Sensor Tracking Type
   *
   * @param sensor_type current Sensor Tracking Type
   */
  void getSensorType(int32_t &sensor_type);
    /**
   * @brief Get the current Robot Error value
   *
   * @param error current Robot Error value
   */
  void getError(uint32_t &error);
  /**
   * @brief Get Robot joint number
   *
   * @param num_joints Robot joint number
   */
  void getNumJoints(uint32_t &num_joints);
  /**
   * @brief Get joint type
   *
   * @param jnt_type Robot joint type
   */
  void getJointType(std::vector<int> &jnt_type);
  /**
   * @brief Get status selector
   *
   * @param stsSelector Robot joint type
   */
  void getStsSelector(uint32_t &stsSelector);
  // =================== WRITE =========================
  /**
   * @brief Write a joint command using the Motion Client
   *
   * @param joint_command desired joint positions
   * @param curr_time current time (used for MoveFly)
   * @param curr_period current period (used for MoveFly)
   * @param ControlMode desired control mode (sensor tracking or joint command)
   * @return true on successful write
   * @return false on failed write
   */
  bool writeCommand(const std::vector<double> &joint_command, double curr_time, double curr_period,
                    ControlMode ControlMode);
  /**
   * @brief Write a asynchronous joint trajectory command using the Motion Client
   *
   * @param joint_command desired joint positions
   * @param ControlMode desired control mode (joint or cartesian trajectory)
   * @return true on successful write
   * @return false on failed write
   */
  bool writeTrajectoryCommand(trajectoryf_t &trajectory, ControlMode ControlMode);
  bool writeJointTrajectoryCommand(joint_trajectoryf_t &trajectory, ControlMode ControlMode);
  /**
   * @brief initializes the robot server pdl program
   */
  bool initializePDL(bool with_verbose) {
    return robot_client_ptr_->sendInitializeMessage(with_verbose, sensor_type_, sensor_cnvrsn_, sensor_gain_,
                                                    sensor_time_, sensor_ofst_lim_trans_, sensor_ofst_lim_rot_,
                                                    din_pins_, dout_pins_);
  }
  /**
   * @brief starts the robot server pdl program
   */
  bool startPDL() {
    return robot_client_ptr_->sendStartMessage();
  }
  /**
   * @brief Terminates the pdl program
   */
  bool terminatePDL() {
    return robot_client_ptr_->sendTerminateMessage();
  }
  /**
   * @brief resets the robot state
   */
  bool resetPDL() {
    return robot_client_ptr_->sendResetMessage();
  }
  /**
   * @brief cancel the robot motion
   */
  bool cancelMotionPDL() {
    return robot_client_ptr_->sendCancelMessage();
  }


  bool isCommunicationInit()
  {
    return communication_initialized_;
  }

  void setCommunicationInit(bool isInit)
  {
    communication_initialized_ = isInit;
  }


  // ================ AUXILIARY =============================================
  /**
   * @brief Enables Async execution
   */
  void enableAllowAsync() {
    allow_async_ = true;
  }
  /**
   * @brief Desables Async execution
   */
  void desableAllowAsync() {
    allow_async_ = false;
  }
  /**
   * @brief Checks whether async execution is allowed
   * @return true if async execution is allowed
   * @return false if async execution is not allowed
   */
  bool checkAllowAsync() {
    return allow_async_;
  }
  /**
   * @brief Set's the Sensor Parameters of the Robot from a ros service
   */
  bool setSensorParams(int &sensor_type, int &sensor_cnvrsn, int &sensor_gain, int &sensor_time,
                       int &sensor_ofst_lim_trans, int &sensor_ofst_lim_rot);
  /**
   * @brief Set's the MoveFly Parameters of the Robot from a ros service
   */
  bool setMoveflyParams(double &threshold, double &lin_velocity, double &fly_dist);
  /**
   * @brief Set's the DOUT states of the Robot from a ros service
   */
  bool setIO(int &pin, bool &state);
  /**
   * @brief Set's the Robot state (lock/pause, unlock/resume, reset) from a ros service
   */
  bool setArmState(int &state);
  
public:
  boost::shared_ptr<comau_tcp_interface::RobotClient> robot_client_ptr_; /**< RobotClient object */
  boost::shared_ptr<comau_tcp_interface::RobotClient> arm1_client_ptr_;  /**< Arm1Client object */
  boost::shared_ptr<comau_tcp_interface::StateClient> state_client_ptr_; /**< StateClient object */
  std::vector<int> jnt_cmd_type_;                     /**< Joint TYPE for Command*/
  uint32_t num_cmd_joints_;
  
private:
  // Name of the driver
  std::string name_;              /**< Name of this class -> comau_driver */
  rclcpp::Node::SharedPtr nh_, nh_local_; /**< ROS NodeHandle objects required for parameters reading */

  comau_tcp_interface::utils::MessagePackage *msg;                       /**< State message */
  comau_tcp_interface::ComauTcpInterfaceParameters state_params_, robot_params_, arm1_params_; /**< Net parameters */
  bool parallel_link_fix_; /**< Parameter for parallel link transformation */
  bool allow_async_ = false;     /**< Parameter indicating if the robot is in a mode ready to receive asynchronous trajectories */
  bool verbose_; /**< Parameter for verbose messages */
  
  bool communication_initialized_ = false; /*ANDY VER if true the initialized and start commands are sent to the server*/
  // State parameters
  comau_tcp_interface::utils::vector10f_t joints_float_;        /**< Joint positions */
  comau_tcp_interface::utils::vector6f_t  ee_float_;            /**< End effector position */
  comau_tcp_interface::utils::vector6f_t  joint_command_float_; /**< Joint commands */
  comau_tcp_interface::utils::vector6f_t  sns_trk_cmd_;         /**< Sensor tracking commands */
  comau_tcp_interface::utils::vector6f_t  joint_cmd_;           /**< Joint commands */
  comau_tcp_interface::utils::vector6i_t  pins_state_in_;       /**< DIN states */
  comau_tcp_interface::utils::vector6i_t  pins_state_out_;      /**< DOUT states */
  comau_tcp_interface::utils::vector10i_t jnt_type_;            /**< Joint TYPE */
  std::vector<int64_t> din_pins_;                                   /**< DIN pins */
  std::vector<int64_t> dout_pins_;                                  /**< DOUT pins */

  // Joint/cartesian default linear velocity
  double default_linear_velocity_;

  // Move Fly Params
  double threshold_;
  double fly_lin_velocity_;
  double fly_dist_;
  double prev_time_ = 0;

  // Sensor Tracking default params
  int sensor_type_, sensor_cnvrsn_, sensor_gain_, sensor_time_, sensor_ofst_lim_trans_, sensor_ofst_lim_rot_;
  //
  bool use_state_server_, use_robot_server_, use_arm1_server_;
};

} // namespace comau_driver

#endif // COMAU_DRIVER_H
