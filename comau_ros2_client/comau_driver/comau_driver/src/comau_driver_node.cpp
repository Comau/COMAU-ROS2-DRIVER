/**
 * @file comau_driver_node.cpp
 * @author Comau Robotics S.p.A.
 * @brief The ROS2 node that publishes the robot information
 * @version 1.0
 * @date 02/07/2024
 *
 * @copyright (c) Comau Robotics S.p.A.
 *
 */

#include <csignal>

#include <comau_driver/comau_driver.hpp>

using namespace comau_tcp_interface;
using namespace comau_tcp_interface::utils;

boost::shared_ptr<comau_driver::ComauRobot> robot_ptr_;
bool use_state_server;
bool use_robot_server;
bool use_arm1_server;

int exitParam;
bool initialization_;
int32_t data_timestamp_;
char robot_status_;
uint32_t error_value_;
int32_t sns_trk_type_;
uint32_t num_robot_joints_;
uint32_t num_joints_;
std::vector<double> ee_position_     = {0,0,0,0,0,0};
std::vector<int> pins_in_            = {0,0,0,0,0,0};
std::vector<int64_t> pins_state_in_  = {0,0,0,0,0,0};
std::vector<int> pins_out_           = {0,0,0,0,0,0};
std::vector<int64_t> pins_state_out_ = {0,0,0,0,0,0};
uint32_t stsSelector_;
std::vector<int> jnt_type_           = {0,0,0,0,0,0};
std::vector<double> joint_position_  = {0.0,0.0,0.0,0.0,0.0,0.0};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> nh = std::make_shared<rclcpp::Node>("comau_driver_node");
  rclcpp::Rate loop_rate(500);

  initialization_ = false;
  num_joints_ = 6;

  // Read parameters through rclcpp parameter server  
    nh->declare_parameter("use_state_server", true);
    use_state_server = nh->get_parameter("use_state_server").as_bool();
    nh->declare_parameter("use_robot_server", true);
    use_robot_server = nh->get_parameter("use_robot_server").as_bool();
    nh->declare_parameter("use_arm1_server", true);
    use_arm1_server = nh->get_parameter("use_arm1_server").as_bool();

    nh->declare_parameter("exit",0);

    // Initialize Robot driver
    try {
      robot_ptr_.reset(new comau_driver::ComauRobot(nh));
      if (!robot_ptr_->initialize(use_state_server, use_robot_server, use_arm1_server)) {
        RCLCPP_ERROR_STREAM(nh->get_logger()," Failed to initialize robot driver");
        rclcpp::shutdown();
      }
      bool state;
      state = false;
      bool robot;
      robot = false;
      bool arm;
      arm = false;
      state = robot_ptr_->state_client_ptr_->openStateThread(true,  state);
      rclcpp::sleep_for(rclcpp::Duration::from_seconds(1).to_chrono<std::chrono::nanoseconds>());
      robot = robot_ptr_->robot_client_ptr_->openRobotThread(true,  robot);
      rclcpp::sleep_for(rclcpp::Duration::from_seconds(1).to_chrono<std::chrono::nanoseconds>());
      arm = robot_ptr_->arm1_client_ptr_->openHandlerThread(true, arm);
      rclcpp::sleep_for(rclcpp::Duration::from_seconds(1).to_chrono<std::chrono::nanoseconds>());

      while (rclcpp::ok()) 
      {
        if(robot_ptr_->state_client_ptr_->isConnected() && robot_ptr_->robot_client_ptr_->isConnected() && robot_ptr_->arm1_client_ptr_->isConnected())
        {
          RCLCPP_INFO_STREAM(nh->get_logger(),"Connected");
          rclcpp::sleep_for(rclcpp::Duration::from_seconds(2).to_chrono<std::chrono::nanoseconds>());

          exitParam = nh->get_parameter("exit").as_int();
          RCLCPP_INFO(nh->get_logger(), "Exit [%d]", exitParam);
          if(exitParam == true)
          {
            if (use_robot_server) {
              RCLCPP_ERROR(nh->get_logger()," Terminating PDL Programs");
              robot_ptr_->terminatePDL();
            }
            if (use_state_server) {
              robot_ptr_->state_client_ptr_->close();
              robot_ptr_->state_client_ptr_.reset();
            }
            if (use_robot_server) {
              robot_ptr_->robot_client_ptr_->close();
              robot_ptr_->robot_client_ptr_.reset();
            }
            if (use_arm1_server) {
              robot_ptr_->arm1_client_ptr_->close();
              robot_ptr_->arm1_client_ptr_.reset();
            }
            rclcpp::shutdown();
          }

          if (robot_ptr_->readMessagePackage())
          {

            robot_ptr_->getTimeStamp(data_timestamp_);
            robot_ptr_->getStatus(robot_status_);
            RCLCPP_INFO(nh->get_logger(), "robot_status_ [%c]", robot_status_);
            robot_ptr_->getSensorType(sns_trk_type_);
            robot_ptr_->getEePosition(ee_position_);
            robot_ptr_->getPinsIN(pins_in_);
            robot_ptr_->getPinsStatesIN(pins_state_in_);
            robot_ptr_->getPinsOUT(pins_out_);
            robot_ptr_->getPinsStatesOUT(pins_state_out_);
            robot_ptr_->getError(error_value_);

            RCLCPP_INFO(nh->get_logger(), "DATA:");
            robot_ptr_->getNumJoints(num_robot_joints_);
            RCLCPP_INFO(nh->get_logger(), "Num_robot_joints_ [%d]", num_robot_joints_);
            num_joints_ = num_robot_joints_;
            robot_ptr_->getStsSelector(stsSelector_);
            RCLCPP_INFO(nh->get_logger(), "stsSelector_ [%d]", stsSelector_);
            robot_ptr_->getJointType(jnt_type_);            
            RCLCPP_INFO(nh->get_logger(), "Jnt_type_:");
            std::cout << "[ ";
            for (uint32_t i = 0; i < num_joints_;i++)
            {
              std::cout << jnt_type_.at(i) << " ";
            }
            std::cout << "]" << std::endl;
            RCLCPP_INFO(nh->get_logger(), "joint_position_:");
            std::cout << "[ ";
            robot_ptr_->getJointPosition(joint_position_, num_robot_joints_, jnt_type_);
            for (uint32_t i = 0; i < num_joints_;i++)
            {
              std::cout << joint_position_.at(i) << " ";
            }
            std::cout << "]" << std::endl;

            if(num_robot_joints_ > 0 && !initialization_)
            {
              if (num_robot_joints_ != num_joints_)
                RCLCPP_WARN_STREAM(nh->get_logger(),"Warning - mismatch between real robot and urdf number of joints: " << "[" << num_robot_joints_ << ", " << num_joints_ << "]");
      
              robot_ptr_->jnt_cmd_type_   = jnt_type_;
              robot_ptr_->num_cmd_joints_ = num_robot_joints_;
              initialization_ = true;
            }
      
            if ((robot_status_ == 'T') || (robot_status_ == 'C') || (robot_status_ == 'R') || (robot_status_ == 'M') ||
                (robot_status_ == 'I') || (robot_status_ == 'P') || (robot_status_ == 'S') || (robot_status_ == 'E') )
            {
              /*execute_joints_handler_ptr->set_status(robot_status_);
              execute_joints_handler_ptr->set_allow_async(robot_ptr_->checkAllowAsync());*/
              /*execute_cartesian_handler_ptr->set_status(robot_status_);
              execute_cartesian_handler_ptr->set_allow_async(robot_ptr_->checkAllowAsync());*/  
            }
            else
            {
              RCLCPP_WARN_STREAM(nh->get_logger(),"Invalid state msg: " << robot_status_);
            }
          }
        }
        
        rclcpp::spin_some(nh);
        loop_rate.sleep();
      }
    } catch (...) {
      RCLCPP_ERROR_STREAM(nh->get_logger()," Failed to initialize robot driver");
      rclcpp::shutdown();
    }
    RCLCPP_INFO_STREAM(nh->get_logger(),"END.");

  return 0;
}
