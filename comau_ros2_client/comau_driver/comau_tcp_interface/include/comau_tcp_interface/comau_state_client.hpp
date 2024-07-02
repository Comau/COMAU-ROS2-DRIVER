/**
 * @file comau_state_client.hpp
 * @author Comau Robotics S.p.A.
 * @brief The ROS2 node recives the robot data
 * @version 1.0
 * @date 02/07/2024
 *
 * @copyright (c) Comau Robotics S.p.A.
 *
 */

#ifndef COMAU_TCP_INTERFACE__COMAU_STATE_CLIENT_H
#define COMAU_TCP_INTERFACE__COMAU_STATE_CLIENT_H

#include <boost/shared_ptr.hpp>
#include <future>
#include <thread>

#include "comau_tcp_interface/comau_client_base.hpp"

namespace comau_tcp_interface {

class StateClient : public comau_tcp_interface::ComauClientBase {
public:
  StateClient() {}
  ~StateClient();

  /**
   * @brief Read the params to connect with the server and initiates the receiving callback thread
   *
   * @param params ComauTcpInterfaceParameters for the State Server
   * @return true for success connection
   */
  bool initialize(const ComauTcpInterfaceParameters& params);
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
   * @brief Returns the The state thread
   *
   * @return
   * @return
   */
  bool openStateThread(bool openState, bool success);
  /**
   * @brief
   *
   * @param msg
   * @return true
   * @return false
   */
  bool getLastMessage(utils::MessagePackage& msg);

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
  bool send(utils::MessagePackage& msg);
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
 

  std::promise<void> exit_promise_signal_; /**< */
  std::future<void> future_obj_for_exit_;  /**< */

  boost::shared_ptr<std::thread> receiving_thread_; /**<  */
  boost::shared_ptr<ComauTcpConnection>
      tcp_interface_ptr_; /**< ComauTcpConnection responsible for the TCP connection */

  /*Variables*/
  bool notCloseState;

public:
  bool is_connected_;                                         /**< private variable holding the connection info*/
};

} // namespace comau_tcp_interface

#endif
