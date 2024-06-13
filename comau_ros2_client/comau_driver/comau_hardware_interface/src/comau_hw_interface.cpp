#include <sstream>

#include "comau_hardware_interface/comau_hw_helpers.hpp"
#include "comau_hardware_interface/comau_hw_interface.hpp"
#include <string>
#include <vector>

namespace comau_hardware_interface
{
CallbackReturn ComauHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  // Get Comau Hardware Interface parameters
  const std::size_t NUM_JOINTS_MAX = 10;
  const std::size_t cart_pose_size = 6;
  
  name_ = "comau_hardware_interface"; 
  nh_ = rclcpp::Node::make_shared(name_);
  nh_priv_ = rclcpp::Node::make_shared(nh_->get_name(), name_);
  position_controller_running_ = false;
  //sensor_tracking_controller_running_ = false;
  controllers_initialized_ = false;

  nh_->declare_parameter("use_state_server", true);
  use_state_server_ = nh_->get_parameter("use_state_server").as_bool();
  nh_->declare_parameter("use_robot_server", true);
  use_robot_server_ = nh_->get_parameter("use_robot_server").as_bool();
  nh_->declare_parameter("use_arm1_server", true);
  use_arm1_server_ = nh_->get_parameter("use_arm1_server").as_bool();
  nh_->declare_parameter("verbose", true);
  verbose_ = nh_->get_parameter("verbose").as_bool();

  // robot has 6 joints and 2 interfaces
  joint_position_.assign(NUM_JOINTS_MAX, 0);
  joint_velocities_.assign(NUM_JOINTS_MAX, 0);
  joint_position_command_.assign(cart_pose_size, 0);
  joint_velocities_command_.assign(cart_pose_size, 0);

  // force sensor has 6 readings
  /*ft_states_.assign(6, 0);
  ft_command_.assign(6, 0);*/

  // services
  thread_service_ = nh_->create_service<comau_msgs::srv::OpenConnection>("tcpip_conn_manager", [&](const comau_msgs::srv::OpenConnection::Request::SharedPtr req, 
                                 comau_msgs::srv::OpenConnection::Response::SharedPtr resp)
                                 {
                                    if(req->open_connection != req_prev->open_connection) {
                                      robot_ptr_->state_client_ptr_->openStateThread( req->open_connection, resp->success);
                                      robot_ptr_->robot_client_ptr_->openRobotThread( req->open_connection, resp->success);
                                      robot_ptr_->arm1_client_ptr_->openHandlerThread(req->open_connection, resp->success);
                                      resp->success = true;
                                    }
                                    req_prev->open_connection = req->open_connection;
                                    resp->success;
                                 });
  setMoveflyParams_service_ = nh_->create_service<comau_msgs::srv::SetMoveFlyParams>("set_movefly_params", [&](const comau_msgs::srv::SetMoveFlyParams::Request::SharedPtr req, 
                                 comau_msgs::srv::SetMoveFlyParams::Response::SharedPtr resp)
                                 {
                                    if (use_robot_server_){
                                      robot_ptr_->setMoveflyParams(req->threshold, req->lin_velocity, req->fly_dist);
                                    }
                                    resp->success = true;
                                 });
  setIO_service_ = nh_->create_service<comau_msgs::srv::SetIO>("set_io", [&](const comau_msgs::srv::SetIO::Request::SharedPtr req, 
                                 comau_msgs::srv::SetIO::Response::SharedPtr resp)
                                 {
                                    if (use_robot_server_){
                                      resp->success = robot_ptr_->setIO(req->pin, req->state);
                                    }
                                    resp->success;
                                 });
  /*setSnsTrkParams_service_  =
      nh_.advertiseService("/set_sensor_tracking_params", &ComauHardwareInterface::setSnsTrkParams_routine, this);*/
  setArmState_service_ = nh_->create_service<comau_msgs::srv::SetArmState>("set_arm_state", [&](const comau_msgs::srv::SetArmState::Request::SharedPtr req, 
                                 comau_msgs::srv::SetArmState::Response::SharedPtr resp)
                                 {
                                    if (use_robot_server_)
                                      robot_ptr_->setArmState(req->arm_state);
                                    resp->success = true;
                                 });

  // Initialize Robot driver
  try {
    robot_ptr_.reset(new comau_driver::ComauRobot(nh_));
    if (!robot_ptr_->initialize(use_state_server_, use_robot_server_, use_arm1_server_)) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_hw_interface")," Failed to initialize robot driver");
      return CallbackReturn::ERROR;
    }
  } catch (...) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_hw_interface")," Failed to initialize robot driver");
    return CallbackReturn::ERROR;
  }
  
  // comau action handlers : execute_joint_trajectory_handler
  try {
    execute_joints_handler_ptr.reset(new comau_action_handlers::ExecuteJointTrajectoryHandler(
        nh_, nh_priv_, execute_joint_server_name_, robot_ptr_));
    if (!execute_joints_handler_ptr->initialize(use_state_server_, use_robot_server_, use_arm1_server_)) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_hw_interface"),"Execute Joint Trajectory Handler could not initialized ");
      return CallbackReturn::ERROR;
    }
  } catch (...) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_hw_interface"),"Execute Joint Trajectory Handler error");
    return CallbackReturn::ERROR;
  }

  for (const auto & joint : info_.joints)
  {
    for (const auto & interface : joint.state_interfaces)
    {
      joint_interfaces[interface.name].push_back(joint.name);
    }
    execute_joints_handler_ptr->urdf_number_of_joints_ += 1;
  }

  RCLCPP_INFO_STREAM(rclcpp::get_logger("comau_hw_interface"),"of joint within URDF file is: " << execute_joints_handler_ptr->urdf_number_of_joints_);
  num_joints_ = execute_joints_handler_ptr->urdf_number_of_joints_;//joint_names_.size();/*ANDY*/

  for(size_t i = 0; i < num_joints_; i++)
  {
    std::stringstream str_temp;
    str_temp << "joint_" << i+1;
    std::string res = str_temp.str();
    joint_names_.push_back(res);
  }

  // Resize vectors
  joint_position_.resize(NUM_JOINTS_MAX); // num_joints_
  joint_velocities_.resize(NUM_JOINTS_MAX);
  //joint_effort_.resize(NUM_JOINTS_MAX);
  joint_position_command_.resize(NUM_JOINTS_MAX);
  joint_velocities_command_.resize(NUM_JOINTS_MAX);
  //joint_effort_command_.resize(NUM_JOINTS_MAX);
  //sensor_tracking_command_.resize(cart_pose_size);
  ee_position_.resize(cart_pose_size);

  initialization_ = false;
  invalidMsgCount_ = 0;
  robot_reset_ = false;
  data_timestamp_prev_ = 0;
  counter_ = 0;

  // comau action handlers : execute_cartesian_trajectory_handler
  try {
    execute_cartesian_handler_ptr.reset(new comau_action_handlers::ExecuteCartesianTrajectoryHandler(
        nh_, nh_priv_, execute_cartesian_server_name_, robot_ptr_));
    if (!execute_cartesian_handler_ptr->initialize(use_state_server_, use_robot_server_, use_arm1_server_)) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_hw_interface"),"Execute Cartesian Trajectory Handler could not initialized ");
      return CallbackReturn::ERROR;
    }
  } catch (...) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("comau_hw_interface"),"Execute Cartesian Trajectory Handler error");
    return CallbackReturn::ERROR;
  }

  //publishers
  try {
    robot_status_pub_.reset(
      new realtime_tools::RealtimePublisher<comau_msgs::msg::ComauRobotStatus>(nh_->create_publisher<comau_msgs::msg::ComauRobotStatus>(
    "robot_status", rclcpp::SystemDefaultsQoS())));
    } catch (const std::exception & e) {
      fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }
  try {
    server_error_pub_.reset(
      new realtime_tools::RealtimePublisher<comau_msgs::msg::ComauServerError>(nh_->create_publisher<comau_msgs::msg::ComauServerError>(
        "server_error", rclcpp::SystemDefaultsQoS())));
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }
  try {
    server_operation_mode_pub_.reset(
      new realtime_tools::RealtimePublisher<comau_msgs::msg::ComauOperationMode>(nh_->create_publisher<comau_msgs::msg::ComauOperationMode>(
        "robot_operation_mode", rclcpp::SystemDefaultsQoS())));
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }
  try {
    ee_pose_pub_.reset(
      new realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>(nh_->create_publisher<tf2_msgs::msg::TFMessage>(
      "tf", rclcpp::SystemDefaultsQoS())));
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }
  try {
    async_enable_pub_.reset(
      new realtime_tools::RealtimePublisher<std_msgs::msg::Bool>(
      nh_->create_publisher<std_msgs::msg::Bool>(
      "async_enable", rclcpp::SystemDefaultsQoS())));
  
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }
  try {
    io_states_pub_.reset(
      new realtime_tools::RealtimePublisher<comau_msgs::msg::IOStates>(
        nh_->create_publisher<comau_msgs::msg::IOStates>("io_states", rclcpp::SystemDefaultsQoS())));
    io_states_pub_->msg_.digital_in_states.resize(6);
    io_states_pub_->msg_.digital_out_states.resize(6);
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }
  /*try {
    sns_trk_type_pub_ = std::make_unique<std_msgs::msg::Int32>(
                        nh_->create_publisher<std_msgs::msg::Int32>("sensor_tracking_type", rclcpp::SystemDefaultsQoS()));
  } catch (const std::exception & e) {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }*/

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ComauHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
  }

  /*state_interfaces.emplace_back("tcp_fts_sensor", "force.x", &ft_states_[0]);
  state_interfaces.emplace_back("tcp_fts_sensor", "force.y", &ft_states_[1]);
  state_interfaces.emplace_back("tcp_fts_sensor", "force.z", &ft_states_[2]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.x", &ft_states_[3]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.y", &ft_states_[4]);
  state_interfaces.emplace_back("tcp_fts_sensor", "torque.z", &ft_states_[5]);*/

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ComauHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_interfaces["position"])
  {
    command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
  }

  ind = 0;
  for (const auto & joint_name : joint_interfaces["velocity"])
  {
    command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
  }

  /*command_interfaces.emplace_back("tcp_fts_sensor", "force.x", &ft_command_[0]);
  command_interfaces.emplace_back("tcp_fts_sensor", "force.y", &ft_command_[1]);
  command_interfaces.emplace_back("tcp_fts_sensor", "force.z", &ft_command_[2]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.x", &ft_command_[3]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.y", &ft_command_[4]);
  command_interfaces.emplace_back("tcp_fts_sensor", "torque.z", &ft_command_[5]);*/

  return command_interfaces;
}

return_type ComauHardwareInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // TODO(pac48) set sensor_states_ values from subscriber
  RCLCPP_WARN_STREAM(rclcpp::get_logger("comau_hw_interface"),"READ");/* ANDY */
  for (auto i = 0ul; i < joint_velocities_command_.size(); i++)
  {
    joint_velocities_[i] = joint_velocities_command_[i];
    joint_position_[i] += joint_velocities_command_[i] * period.seconds();
  }

  for (auto i = 0ul; i < joint_position_command_.size(); i++)
  {
    joint_position_[i] = joint_position_command_[i];
  }

  if (!(use_state_server_ || use_robot_server_ || use_arm1_server_)) 
  {
    if (position_controller_running_)
      copyVector(joint_position_command_, joint_position_);
    return return_type::OK;
  }

  if (use_state_server_ && robot_ptr_->state_client_ptr_->is_connected_) {
    if (robot_ptr_->readMessagePackage()) {
      robot_ptr_->getTimeStamp(data_timestamp_);
      robot_ptr_->getStatus(robot_status_);
      robot_ptr_->getSensorType(sns_trk_type_);
      robot_ptr_->getEePosition(ee_position_);
      robot_ptr_->getPinsIN(pins_in_);
      robot_ptr_->getPinsStatesIN(pins_state_in_);
      robot_ptr_->getPinsOUT(pins_out_);
      robot_ptr_->getPinsStatesOUT(pins_state_out_);
      robot_ptr_->getError(error_value_);
      robot_ptr_->getNumJoints(num_robot_joints_);
      robot_ptr_->getJointType(jnt_type_);
      robot_ptr_->getJointPosition(joint_position_, num_robot_joints_, jnt_type_);
      robot_ptr_->getStsSelector(stsSelector_);

      if(num_robot_joints_ > 0 && !initialization_)
      {
        if (num_robot_joints_ != num_joints_)
          RCLCPP_WARN_STREAM(rclcpp::get_logger("comau_hw_interface"),"Warning - mismatch between real robot and urdf number of joints: " << "[" << num_robot_joints_ << ", " << num_joints_ << "]");

        robot_ptr_->jnt_cmd_type_   = jnt_type_;
        robot_ptr_->num_cmd_joints_ = num_robot_joints_;
        initialization_ = true;
      }

      if ((robot_status_ == 'T') || (robot_status_ == 'C') || (robot_status_ == 'R') || (robot_status_ == 'M') ||
          (robot_status_ == 'I') || (robot_status_ == 'P') || (robot_status_ == 'S') || (robot_status_ == 'E') )
      {
        packet_read_ = true;
        if (error_value_ != error_value_prev_)
        {
          RCLCPP_WARN_STREAM(rclcpp::get_logger("comau_hw_interface"),"Error from server: " << error_value_);
          errorParser(error_value_);
          publishErrorValue();

          /*  Get server reset */
          if (error_value_prev_ > 0 && error_value_prev_ < KI_ERR_STATE_MAX && error_value_ == 0)
            robot_reset_ = true;

          error_value_prev_ = error_value_;          
        }
        publishRobotStatus();
        publishEndEffectorPose();
        publishIOPins();
        //publishSnsTrkType();
        publishOperationMode();
        execute_joints_handler_ptr->set_status(robot_status_);
        execute_joints_handler_ptr->set_allow_async(robot_ptr_->checkAllowAsync());
        execute_cartesian_handler_ptr->set_status(robot_status_);
        execute_cartesian_handler_ptr->set_allow_async(robot_ptr_->checkAllowAsync());

        invalidMsgCount_ = 0;   
      }
      else
      {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("comau_hw_interface"),"Invalid state msg: " << robot_status_);
        //std::cout << "Server error:      " << error_value_  << std::endl;
        invalidMsgCount_ ++; // counter to restart the connection
        if (robot_ptr_->state_client_ptr_->is_connected_ && invalidMsgCount_ > 2) // VE_ADD : sistema la visibilit� di is_connected_ e state_client_ptr_ che � stata cambiata
        {
          RCLCPP_WARN_STREAM(rclcpp::get_logger("comau_hw_interface"),"Restart state_client");
          robot_ptr_->state_client_ptr_->is_connected_ = false;
        }
      }     
    }
  }

  return return_type::OK;
}

return_type ComauHardwareInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (use_robot_server_ && robot_ptr_->robot_client_ptr_->isConnected())
  {
    if (!robot_ptr_->isCommunicationInit())
    {
      // Initialize PDL through driver
      robot_ptr_->initializePDL(verbose_);
      rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.3).to_chrono<std::chrono::nanoseconds>());
      // Enables Motion
      robot_ptr_->startPDL();
      rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.3).to_chrono<std::chrono::nanoseconds>());
      //robot_ptr_->cancelMotionPDL();
      //ros::Duration(0.3).sleep();
      robot_ptr_->setCommunicationInit(true);
    }

    if (robot_status_ == comau_tcp_interface::RobotStatus::ERROR && robot_reset_)
    {
      RCLCPP_ERROR_STREAM(nh_->get_logger(),"Send reset command after an Error is returned from the server.");
      robot_ptr_->resetPDL();
      robot_reset_ = false;
    }
      
  }
  
  //if (!(use_state_server_ || use_robot_server_ || use_arm1_server_)) {
  //  if (sensor_tracking_controller_running_)
  //    printVector(sensor_tracking_command_);
  //}
  if (use_arm1_server_) {

    if ((robot_status_ == comau_tcp_interface::RobotStatus::READY ||
         robot_status_ == comau_tcp_interface::RobotStatus::MOVING) &&
        packet_read_) {
      /*if (sensor_tracking_controller_running_) {
        robot_ptr_->writeCommand(sensor_tracking_command_, time, period,
                                 comau_driver::ControlMode::MODE_SENSOR_TRACKING);
      } else*/ 
      if (position_controller_running_) {
        robot_ptr_->writeCommand(joint_position_command_, time.nanoseconds(), period.nanoseconds(),
                                 comau_driver::ControlMode::MODE_POSITION);
      } else {
        holdConnection();
      }
      packet_read_ = false;
    }
  }

  return return_type::OK;
}

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(comau_hardware_interface::ComauHardwareInterface, 
                       hardware_interface::SystemInterface)