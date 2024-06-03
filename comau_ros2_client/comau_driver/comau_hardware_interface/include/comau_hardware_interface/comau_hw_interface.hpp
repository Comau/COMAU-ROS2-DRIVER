#ifndef ROS2_COMAU_HARDWARE_INTERFACE__R6BOT_HARDWARE_HPP_
#define ROS2_COMAU_HARDWARE_INTERFACE__R6BOT_HARDWARE_HPP_

#pragma once

#include <boost/scoped_ptr.hpp>
#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include "unordered_map"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.h"

#include <comau_driver/comau_driver.hpp>
#include "comau_handlers/execute_joint_trajectory_handler.hpp"
#include "comau_handlers/execute_cartesian_trajectory_handler.hpp"
#include <comau_msgs/msg/comau_robot_status.hpp>
#include <comau_msgs/msg/comau_server_error.hpp>
#include <comau_msgs/msg/comau_operation_mode.hpp>
#include <comau_msgs/msg/digital.hpp>
#include <comau_msgs/msg/io_states.hpp>
//#include <dynamic_reconfigure/server.h>
#include <std_msgs/msg/bool.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <comau_msgs/srv/open_connection.hpp>
#include <comau_msgs/srv/set_move_fly_params.hpp>
#include <comau_msgs/srv/set_arm_state.hpp>
#include <comau_msgs/srv/set_io.hpp>
#include <comau_msgs/srv/set_sns_trk_params.hpp>
#include <std_msgs/msg/int32.hpp>

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

using hardware_interface::return_type;

namespace comau_hardware_interface
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HARDWARE_INTERFACE_PUBLIC ComauHardwareInterface : public hardware_interface::SystemInterface
{
  public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
    return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  
    return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    void publishEndEffectorPose();
  
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

    bool checkControllerClaims(const std::set<std::string> &claimed_resources);

    bool isRobotProgramRunning() const;

    void closeComauDriver();

    void errorParser(uint32_t error_value);
  
  protected:
  
    void setIO_routine(const std::shared_ptr<comau_msgs::srv::SetIO::Request> req,
                             std::shared_ptr<comau_msgs::srv::SetIO::Response> res);

    /*void threadConnections(const std::shared_ptr<comau_msgs::srv::OpenConnection::Request> req, 
                                 std::shared_ptr<comau_msgs::srv::OpenConnection::Response> res);*/

    void setMoveflyParams_routine(const std::shared_ptr<comau_msgs::srv::SetMoveFlyParams::Request> req,
                                        std::shared_ptr<comau_msgs::srv::SetMoveFlyParams::Response> res);

    void setArmState_routine(const std::shared_ptr<comau_msgs::srv::SetArmState::Request> req, 
                                   std::shared_ptr<comau_msgs::srv::SetArmState::Response> res);

    std::string name_;                                      /**< Name of this class -> comau_hardware_interface */
    std::shared_ptr<rclcpp::Node> nh_, nh_priv_;                  /**< ROS NodeHandle objects required for parameters reading */
    boost::shared_ptr<comau_driver::ComauRobot> robot_ptr_; /**< Robot driver object pointer */

    // Configuration
    bool position_controller_running_;
    bool velocity_controller_running_;
    bool controllers_initialized_;
    bool robot_program_running_ = true; /* TODO */
      
    // States
    std::vector<double> joint_position_;
    std::vector<double> joint_velocities_; // joint_velocity_
    std::vector<double> ft_states_;
    std::vector<double> ee_position_;
    std::vector<int> pins_in_;
    std::vector<int> pins_state_in_;
    std::vector<int> pins_out_;
    std::vector<int> pins_state_out_;
    // Commands
    std::vector<double> joint_position_command_;
    std::vector<double> joint_velocities_command_; // joint_velocity_command_
    //std::vector<double> ft_command_; // sensor_tracking_command_

    std::unordered_map<std::string, std::vector<std::string>> joint_interfaces = {
    {"position", {}}, {"velocity", {}}};

    // private
    size_t num_joints_;
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
    geometry_msgs::msg::TransformStamped ee_transform_; // TODO : change timestamp to ros::Time
    tf2::Quaternion q;

    // publishers
    std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Bool>> async_enable_pub_;
    using StatePublisher = realtime_tools::RealtimePublisher<comau_msgs::msg::ComauRobotStatus>;
    rclcpp::Publisher<comau_msgs::msg::ComauRobotStatus>::SharedPtr robot_sts_pub_;
    std::unique_ptr<StatePublisher> robot_status_pub_; /**< ROS status publisher see ComauRobotStatus.msg */
    std::unique_ptr<realtime_tools::RealtimePublisher<comau_msgs::msg::ComauServerError>>
        server_error_pub_; /**< ROS error publisher see ComauServerError.msg */
    std::unique_ptr<realtime_tools::RealtimePublisher<comau_msgs::msg::ComauOperationMode>>
        server_operation_mode_pub_; /**< ROS robot status publisher see ComauOperationMode.msg */
    std::unique_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>> ee_pose_pub_;
    std::unique_ptr<realtime_tools::RealtimePublisher<comau_msgs::msg::IOStates>>
        io_states_pub_; /**< ROS status publisher see ComauRobotStatus.msg */
    /*std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Int32>>
        sns_trk_type_pub_;*/ /**< ROS status publisher see ComauRobotStatus.msg */
    // services
    rclcpp::Service<comau_msgs::srv::SetMoveFlyParams>::SharedPtr setMoveflyParams_service_;
    rclcpp::Service<comau_msgs::srv::SetIO>::SharedPtr setIO_service_;
    //rclcpp::Service<comau_msgs::srv::SetSnsTrkParams>::SharedPtr setSnsTrkParams_service_;
    rclcpp::Service<comau_msgs::srv::SetArmState>::SharedPtr setArmState_service_;
    rclcpp::Service<comau_msgs::srv::OpenConnection>::SharedPtr thread_service_;

    std::shared_ptr<comau_msgs::srv::OpenConnection::Request> req_prev;

    // comau action handlers
    const std::string execute_joint_server_name_ = "execute_joint_trajectory_handler";
    std::unique_ptr<comau_action_handlers::ExecuteJointTrajectoryHandler>
                    execute_joints_handler_ptr; /**< Object for asynchronous joint trajectory action server */
    const std::string execute_cartesian_server_name_ = "execute_cartesian_trajectory_handler";
    std::unique_ptr<comau_action_handlers::ExecuteCartesianTrajectoryHandler>
                    execute_cartesian_handler_ptr; /**< Object for asynchronous cartesian trajectory action server */
};

}  // namespace ros2_control_demo_example_7

#endif  // ROS2_CONTROL_DEMO_EXAMPLE_7__R6BOT_HARDWARE_HPP_
