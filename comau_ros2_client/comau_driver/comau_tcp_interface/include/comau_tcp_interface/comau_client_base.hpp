/**
 * @file comau_client_base.h
 * @author Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 * @brief The ROS node that publishes the robot information
 * @version 0.1
 * @date 25-02-2020
 *
 * @copyright (c) 2020 Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 *
 */

#ifndef COMAU_TCP_INTERFACE__COMAU_CLIENT_BASE_HPP
#define COMAU_TCP_INTERFACE__COMAU_CLIENT_BASE_HPP

#include "comau_tcp_interface/comau_tcp_interface.hpp"
#include "comau_tcp_interface/utils/custom_data_type.hpp"
#include "comau_tcp_interface/utils/message_package.hpp"

namespace comau_tcp_interface {

class ComauClientBase {

protected:
  ComauClientBase() {}

public:
  virtual ~ComauClientBase(){}

  virtual bool initialize(const ComauTcpInterfaceParameters &params) = 0;
  virtual void close() = 0;
  virtual bool isConnected() = 0;
  virtual bool getLastMessage(utils::MessagePackage &msg) = 0;
  virtual bool openStateThread(bool openState, bool success) = 0; /*TEMP*/
  utils::vectorstr_t getRecvRecipe() {
    return incoming_msg_descr_;
  }
  utils::vectorstr_t getSendTerminateRecipe() {
    return terminate_msg_descr_;
  }
  utils::vectorstr_t getSendResetRecipe() {
    return reset_msg_descr_;
  }
  utils::vectorstr_t getSendDisconnectRecipe() {
    return disco_msg_descr_;
  }
  utils::vectorstr_t getSendInitializeRecipe() {
    return initialize_msg_descr_;
  }
  utils::vectorstr_t getSendStartRecipe() {
    return start_msg_descr_;
  }
  utils::vectorstr_t getSendTrajJointRecipe() {
    return traj_joint_msg_descr_;
  }
  utils::vectorstr_t getSendTrajCartRecipe() {
    return traj_cart_msg_descr_;
  }
  utils::vectorstr_t getSendSnsTrkRecipe() {
    return sns_trk_msg_descr_;
  }
  utils::vectorstr_t getSendPosRecipe() {
    return pos_msg_descr_;
  }
  utils::vectorstr_t getSendMoveFlyRecipe() {
    return movefly_msg_descr_;
  }
  utils::vectorstr_t getSendIORecipe() {
    return io_msg_descr_;
  }
  utils::vectorstr_t getSendConfigurationRecipe() {
    return configuration_msg_descr_;
  }
  utils::vectorstr_t getSendArmStateRecipe() {
    return arm_state_msg_descr_;
  }

protected:
  virtual void receive(std::chrono::milliseconds timeout) = 0;
  virtual bool send(utils::MessagePackage &msg) = 0;

  std::unique_ptr<utils::MessagePackage> last_recv_msg_;
  std::vector<std::string> incoming_msg_descr_;
  std::vector<std::string> incoming_msg_motion_descr_;
  std::vector<std::string> terminate_msg_descr_;
  std::vector<std::string> initialize_msg_descr_;
  std::vector<std::string> start_msg_descr_;
  std::vector<std::string> reset_msg_descr_;
  std::vector<std::string> disco_msg_descr_;
  std::vector<std::string> traj_joint_msg_descr_;
  std::vector<std::string> traj_cart_msg_descr_;
  std::vector<std::string> sns_trk_msg_descr_;
  std::vector<std::string> pos_msg_descr_;
  std::vector<std::string> movefly_msg_descr_;
  std::vector<std::string> configuration_msg_descr_;
  std::vector<std::string> io_msg_descr_;
  std::vector<std::string> arm_state_msg_descr_;
};

} // namespace comau_tcp_interface

#endif //COMAU_TCP_INTERFACE__COMAU_CLIENT_BASE_HPP
