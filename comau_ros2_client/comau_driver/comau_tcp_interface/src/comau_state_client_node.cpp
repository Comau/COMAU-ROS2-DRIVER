/**
 * @file comau_state_client_node.cpp
 * @author Comau Robotics S.p.A.
 * @brief The ROS2 node that composes TCP message
 * @version 1.0
 * @date 02/07/2024
 *
 * @copyright (c) Comau Robotics S.p.A.
 *
 */

#include <csignal>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "comau_tcp_interface/comau_client_base.hpp"
#include "comau_tcp_interface/comau_state_client.hpp"

using namespace comau_tcp_interface;

[[noreturn]] void signalHandler(int signum) {

  RCLCPP_WARN_STREAM(rclcpp::get_logger("comau_state_client_node"), "  Interrupt signal (" << signum << ") received.\n");

  exit(signum);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("comau_state_client_node");
  auto nh_local = rclcpp::Node::make_shared("_");

  // register signal SIGINT and signal handler
  signal(SIGINT, signalHandler);

  ComauTcpInterfaceParameters params;

  sleep(3);
  // Read parameters through rclcpp parameter server
  nh->declare_parameter("robot_ip", "192.168.56.2");
  params.server_ip_address = nh->get_parameter("robot_ip").as_string();
  nh->declare_parameter("state_server_port", "1104");
  params.server_port = nh->get_parameter("state_server_port").as_string();
  params.log_tag = "[comau_state_client_node] ";

  pluginlib::ClassLoader<ComauClientBase> client_loader("comau_tcp_interface", "comau_tcp_interface::ComauClientBase");
  bool try2connect;
  bool successOrNot;
  successOrNot = false;
  try {
    std::shared_ptr<ComauClientBase> state_client = client_loader.createSharedInstance("comau_tcp_interface::StateClient");

    if (state_client->initialize(params)) {
      try2connect = true;
      //utils::MessagePackage msg(state_client->getRecvRecipe());
      //utils::MessagePackage *msg;
      successOrNot = state_client->openStateThread(try2connect, successOrNot);
      RCLCPP_INFO_STREAM(rclcpp::get_logger("comau_state_client_node"),successOrNot);
      //msg = dynamic_cast<comau_tcp_interface::utils::MessagePackage *>(new comau_tcp_interface::utils::MessagePackage(state_client->getRecvRecipe()));
      
      while (rclcpp::ok()) {
        //if (state_client->getLastMessage(msg)) 
        {
          rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.1).to_chrono<std::chrono::nanoseconds>());
          while(state_client->isConnected())
          {//RICORDARSI DI CAMBIARE openStateThread() in base e robotClient
            rclcpp::sleep_for(rclcpp::Duration::from_seconds(2).to_chrono<std::chrono::nanoseconds>());
            //if (state_client->getLastMessage(*msg)) 
            {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("comau_state_client_node"),"Still Connected.");
            } 
           // else 
            /*{
              RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_state_client_node")," Could not get Last Message Package");
            }*/
          }
          //RCLCPP_INFO_STREAM(msg.toString());
        }
      }

      state_client->close();
    } else {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_state_client_node"), " Error at state client initialize");
    }
  } catch (pluginlib::PluginlibException &e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_state_client_node"), e.what());
  }

  rclcpp::shutdown();

  return 0;
}
