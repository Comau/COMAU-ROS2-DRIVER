/**
 * @file comau_state_client_node.cpp
 * @author Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 * @brief The ROS node that publishes the robot information
 * @version 0.1
 * @date 25-02-2020
 *
 * @copyright (c) 2020 Laboratory for Manufacturing Systems & Automation (LMS) - University of Patras
 *
 */

#include <csignal>
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include <boost/shared_ptr.hpp>

#include "comau_tcp_interface/comau_client_base.hpp"

using namespace comau_tcp_interface;

[[noreturn]] void signalHandler(int signum) {

  ComauTcpInterfaceParameters params;
  params.log_tag = "[comau_state_client_node] ";
  RCLCPP_WARN_STREAM(rclcpp::get_logger(params.log_tag), " Interrupt signal (" << signum << ") received.\n");

  exit(signum);
}

int main(int argc, char **argv) {
  //ros::init(argc, argv, "comau_state_client_node", ros::init_options::NoRosout);
  //ros::NodeHandle nh("");

  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node("comau_state_client_node");

  // register signal SIGINT and signal handler
  signal(SIGINT, signalHandler);

  ComauTcpInterfaceParameters params;

  sleep(3);
  // Read parameters through ros parameter server
  nh.declare_parameter("robot_ip", "192.168.56.2");
  params.server_ip_address = nh.get_parameter("robot_ip").as_string();
  std::vector<rclcpp::Parameter> new_ip_address{rclcpp::Parameter("robot_ip", "192.168.56.2")};
  nh.set_parameters(new_ip_address);
  
  nh.declare_parameter("state_server_port", "1104");
  params.server_port = nh.get_parameter("state_server_port").as_string();
  std::vector<rclcpp::Parameter> new_server_port{rclcpp::Parameter("state_server_port", "1104")};
  nh.set_parameters(new_server_port);
    
  params.log_tag = "[comau_state_client_node] ";

  pluginlib::ClassLoader<ComauClientBase> client_loader("comau_tcp_interface", "comau_tcp_interface::ComauClientBase");

  try {
    std::shared_ptr<ComauClientBase> state_client = client_loader.createSharedInstance("comau_tcp_interface::StateClient");

    if (state_client->initialize(params)) {
      comau_tcp_interface::utils::vectorstr_t getRecvRecipe = state_client->getRecvRecipe();     
      utils::MessagePackage msg(getRecvRecipe);
      while (rclcpp::ok()) 
      {
        /*if (state_client->getLastMessage(msg)) {
          rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.1).to_chrono<std::chrono::nanoseconds>()); //0.1 ROS Melodic
          RCLCPP_INFO_STREAM(rclcpp::get_logger(params.log_tag), msg.toString());
        }*/
      }

      state_client->close();
    } else {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger(params.log_tag), " Error at state client initialize");
    }
  } catch (pluginlib::PluginlibException &e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger(params.log_tag), e.what());
  }

  rclcpp::shutdown();

  return 0;
}
