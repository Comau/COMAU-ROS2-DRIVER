/**
 * @file comau_robot_client.hpp
 * @author Comau Robotics S.p.A.
 * @brief The ROS2 node send the robot data
 * @version 1.0
 * @date 02/07/2024
 *
 * @copyright (c) Comau Robotics S.p.A.
 *
 */

#ifndef COMAU_TCP_INTERFACE__COMAU_ROBOT_CLIENT_H
#define COMAU_TCP_INTERFACE__COMAU_ROBOT_CLIENT_H

#include <boost/shared_ptr.hpp>
#include <future>
#include <thread>

#include "comau_tcp_interface/comau_client_base.hpp"

namespace comau_tcp_interface {

class RobotClient : public ComauClientBase {
public:
  RobotClient() {}
  ~RobotClient();

  /**
   * @brief Read the params to connect with the server and initiates the receiving callback thread
   *
   * @param params ComauTcpInterfaceParameters for the State Server
   * @return true for success connection
   */
  bool initialize(const ComauTcpInterfaceParameters &params);
  /**
   * @brief Closes the connection with the server
   *
   */
  void close();
  /**
   * @brief Returns the connetion state
   *
   * @return true for connected
   * @return false for not connected
   */
  bool isConnected();
  /**
   * @brief get the last message of the robot
   *
   * @param msg
   * @return true
   * @return false
   */
  bool getLastMessage(utils::MessagePackage &msg);
  /**
   * @brief Sends the Joint trajectory message
   *
   * @param joint_values
   * @return true
   * @return false
   */
  bool sendJointTrajectoryMessage(utils::joint_trajectoryf_t joint_values);
  /**
   * @brief Sends the Joint targets
   *
   * @param joint_position_command
   * @return true
   * @return false
   */
  bool sendJointPositionMessage(utils::vector6f_t joint_position_command);
  /**
   * @brief  Sends the Movefly message
   * 
   * @param joint_position_command
   * @param lin_velocity
   * @param fly_dist
   * @return true
   * @return false
   */
  bool sendJointMoveFlyMessage(utils::vector6f_t joint_position_command, float lin_velocity, float fly_dist);
  /**
   * @brief  Sends the sensor tracking corrections
   *
   * @param sensor_tracking_command
   * @return true
   * @return false
   */
  bool sendSensorTrackingMessage(utils::vector6f_t sensor_tracking_command);
  /**
   * @brief  Sends the Cartesian trajectory
   *
   * @param joint_values
   * @return true
   * @return false
   */
  bool sendCartTrajectoryMessage(utils::trajectoryf_t trajectory);
  /**
   * @brief Sends the message to terminate the pdl program
   *
   * @param
   * @return true
   * @return false
   */
  bool sendTerminateMessage();
  /**
   * @brief  Sends the message to reset the robot state
   *
   * @param
   * @return true
   * @return false
   */
  bool sendResetMessage();
  /**
   * @brief  Sends the message to cancel the robot motion
   *
   * @param
   * @return true
   * @return false
   */
  bool sendCancelMessage();
  /**
   * @brief  Sends the message to disconnect states
   *
   * @param
   * @return true
   * @return false
   */
  bool sendDisconnectMessage();
  /**
   * @brief  Sends the robot to initialize the robot driver parameters
   *
   * @param
   * @return true
   * @return false
   */
  bool sendInitializeMessage(bool with_verbose, int &sensor_type, int &sensor_cnvrsn, int &sensor_gain,
                             int &sensor_time, int &sensor_ofst_lim_trans, int &sensor_ofst_lim_rot,
                             std::vector<int64_t> &din_pins, std::vector<int64_t> &dout_pins);
  /**
   * @brief  Sends the message to start the robot
   *
   * @param
   * @return true
   * @return false
   */
  bool sendStartMessage();
  /**
   * @brief Sends the Sensor Configuration message
   *
   * @param
   * @return true
   * @return false
   */
  bool sendSensorConfigurationMessage(int &sensor_type, int &sensor_cnvrsn, int &sensor_gain, int &sensor_time,
                                      int &sensor_ofst_lim_trans, int &sensor_ofst_lim_rot);
  /**
   * @brief Sends the IO message
   *
   * @param
   * @return true
   * @return false
   */
  bool sendIOMessage(int pin, bool state);
  /**
   * @brief Sends the Arm state message
   *
   * @param
   * @return true
   * @return false
   */
  bool sendArmStateMessage(int state);
  /**
   * @brief Returns the The Robot thread
   *
   * @return
   * @return
   */
  bool openRobotThread(bool openRobot, bool success); /* ANDY FOR TEST NODE */
  /**
   * @brief Returns the The Handler thread
   *
   * @return
   * @return
   */
  bool openHandlerThread(bool openHandler, bool success); /* ANDY FOR TEST NODE */

  bool openStateThread(bool openState, bool success); /* ANDY FOR TEST NODE */
  
private:
  /**
   * @brief Read the message and parse it into the MessagePackage last_recv_msg_
   *
   * @param timeout  TODO implement the timeout for receive
   */
  void receive(std::chrono::milliseconds timeout);
  /**
   * @brief Send the serialized message package
   *
   * @param msg
   */
  bool send(utils::MessagePackage &msg);
  /**
   * @brief Validate message based on timestamp
   *
   * @param none
   */
  bool validate();
  /**
   * @brief Callback function running in separate thread for non blocking receiving
   *
   * @param exit
   */
  void callback(std::future<void> exit_signal);

  boost::shared_ptr<ComauTcpInterfaceParameters> params_ptr_; /**< private pointer for client parameters*/
  bool is_connected_;                                         /**< private variable holding the connection info*/

  std::promise<void> robot_exit_promise_signal_; /**< */
  std::future<void> robot_future_obj_for_exit_;  /**< */

  boost::shared_ptr<std::thread> robot_receiving_thread_; /**<  */

  boost::shared_ptr<ComauTcpConnection>
      tcp_interface_ptr_; /**< ComauTcpConnection responsible for the TCP connection */

  /*Variables*/
  bool notRobotState;
  bool notHandlerState;
};

} // namespace comau_tcp_interface

#endif // COMAU_ROBOT_CLIENT_H
