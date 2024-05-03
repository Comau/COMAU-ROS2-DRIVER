#include "rclcpp/rclcpp.hpp"

class MainNode : public rclcpp::Node {
  public: 
  MainNode() : Node("load_param_node", rclcpp::NodeOptions()) {
     // declare parameter and default value config/transforms
     declare_parameter("parallel_joint_fix", false); 

     // declare parameter and default value config/net
     declare_parameter("robot_ip", "0.0.0.0");            // this the the IP of the Robot
     declare_parameter("state_server_port", 0);           // the port of TCP state server
     declare_parameter("robot_server_port", 0);           // the port of TCP robot/executor server
     declare_parameter("arm1_server_port", 0);            // the port of TCP for Arm1 server

     // declare parameter and default value config/driver 
     declare_parameter("loop_hz", 0);
     declare_parameter("cycle_time_error_threshold", 0.0);
     declare_parameter("use_state_server", false);
     declare_parameter("use_robot_server", false);
     declare_parameter("use_arm1_server", false);
     declare_parameter("verbose", false);
     declare_parameter("default_linear_velocity", 0.0);
     declare_parameter("fly_lin_velocity", 0.0);
     declare_parameter("fly_dist", 0.0);
     declare_parameter("threshold", 0.0);
     declare_parameter("sensor_type", 0);
     declare_parameter("sensor_cnvrsn", 0);
     declare_parameter("sensor_gain", 0);
     declare_parameter("sensor_time", 0);
     declare_parameter("sensor_ofst_lim_trans", 0);
     declare_parameter("sensor_ofst_lim_rot", 0);
     declare_parameter("din_pins", std::vector<int64_t>(6, 1));
     declare_parameter("dout_pins", std::vector<int64_t>(6, 1));
    //comau_point_follower
     declare_parameter("with_ori", false);
    // joint_state_controller
     declare_parameter("type", "default");
     declare_parameter("publish_rate", 0);
    //sensor_tracking_relative_controller
     declare_parameter("state_publish_rate", 0);
     declare_parameter("joints", std::vector<std::string>(6, "default"));
     declare_parameter("dead_man_timeout", 0.0);
     declare_parameter("ee_vel_limit", 0.0);
     declare_parameter("with_plot", false);
     declare_parameter("stop_trajectory_duration", 0.0);
     declare_parameter("action_monitor_rate", 0);
     //pos_joint_traj_controller 
     declare_parameter("goal_time", 0.0);
     declare_parameter("stopped_velocity_tolerance", 0.0);

     // Get parameter values one by one config/transforms
     auto p0 = get_parameter("parallel_joint_fix").as_bool();
     // Get parameter values one by one config/net
     auto p1 = get_parameter("robot_ip").as_string();
     auto p2 = get_parameter("state_server_port").as_int();
     auto p3 = get_parameter("robot_server_port").as_int();
     auto p4 = get_parameter("arm1_server_port").as_int();
     // Get parameter values one by one config/driver 
     auto p5 = get_parameter("loop_hz").as_int();
     auto p6 = get_parameter("cycle_time_error_threshold").as_double();
     auto p7 = get_parameter("use_state_server").as_bool();
     auto p8 = get_parameter("use_robot_server").as_bool();
     auto p9 = get_parameter("use_arm1_server").as_bool();
     auto p10 = get_parameter("verbose").as_bool();
     auto p11 = get_parameter("default_linear_velocity").as_double();
     auto p12 = get_parameter("fly_lin_velocity").as_double();
     auto p13 = get_parameter("fly_dist").as_double();
     auto p14 = get_parameter("threshold").as_double();
     auto p15 = get_parameter("sensor_type").as_int();
     auto p16 = get_parameter("sensor_cnvrsn").as_int();
     auto p17 = get_parameter("sensor_gain").as_int();
     auto p18 = get_parameter("sensor_time").as_int();
     auto p19 = get_parameter("sensor_ofst_lim_trans").as_int();
     auto p20 = get_parameter("sensor_ofst_lim_rot").as_int();
     auto p21 = get_parameter("din_pins").as_integer_array();
     auto p22 = get_parameter("dout_pins").as_integer_array();
    //comau_point_follower
     auto p23 = get_parameter("with_ori").as_bool();
    //joint_state_controller:
     auto p24 = get_parameter("publish_rate").as_int();
     auto p26 = get_parameter("type").as_string();
     //sensor_tracking_relative_controller   ****
     auto p27 = get_parameter("state_publish_rate").as_int();
     auto p28 = get_parameter("joints").as_string_array();
     auto p29 = get_parameter("dead_man_timeout").as_double();
     auto p30 = get_parameter("ee_vel_limit").as_double();
     auto p31 = get_parameter("with_plot").as_bool();
     auto p32 = get_parameter("stop_trajectory_duration").as_double();    
     auto p34 = get_parameter("action_monitor_rate").as_int();                       
     //pos_joint_traj_controller   
     auto p35 = get_parameter("goal_time").as_double();
     auto p36 = get_parameter("stopped_velocity_tolerance").as_double();

     // Print:

    RCLCPP_INFO(get_logger(), "String vector parameter  joints [0]: %s", p28[0].c_str());
    RCLCPP_INFO(get_logger(), "Boolean parameter parallel_joint_fix : %d", p0);
    RCLCPP_INFO(get_logger(), "String  parameter robot_ip:%s", p1.c_str());
    RCLCPP_INFO(get_logger(), "Integer parameter state_server_port: %ld", p2);
    RCLCPP_INFO(get_logger(), "Integer parameter robot_server_port: %ld", p3);
    RCLCPP_INFO(get_logger(), "Integer parameter arm1_server_port: %ld", p4);
    RCLCPP_INFO(get_logger(), "Integer parameter loop_hz: %ld", p5);
    RCLCPP_INFO(get_logger(), "Double parameter cycle_time_error_threshold: %f", p6);
    RCLCPP_INFO(get_logger(), "Boolean parameter use_state_server : %d", p7);
    RCLCPP_INFO(get_logger(), "Boolean parameter use_robot_server : %d", p8);
    RCLCPP_INFO(get_logger(), "Boolean parameter use_arm1_server : %d", p9);
    RCLCPP_INFO(get_logger(), "Boolean parameter verbose : %d", p10);
    RCLCPP_INFO(get_logger(), "Double parameter default_linear_velocity: %f", p11);
    RCLCPP_INFO(get_logger(), "Double parameter fly_lin_velocity: %f", p12);
    RCLCPP_INFO(get_logger(), "Double parameter fly_dist: %f", p13);
    RCLCPP_INFO(get_logger(), "Double parameter threshold: %f", p14);
    RCLCPP_INFO(get_logger(), "Integer parameter sensor_type: %ld", p15);
    RCLCPP_INFO(get_logger(), "Integer parameter sensor_cnvrsn: %ld", p16);
    RCLCPP_INFO(get_logger(), "Integer parameter sensor_gain: %ld", p17);
    RCLCPP_INFO(get_logger(), "Integer parameter sensor_time: %ld", p18);
    RCLCPP_INFO(get_logger(), "Integer parameter sensor_ofst_lim_trans: %ld", p19);
    RCLCPP_INFO(get_logger(), "Integer parameter sensor_ofst_lim_rot: %ld", p20);
    RCLCPP_INFO(get_logger(), "Integer vector parameter [0]din_pins: %d", static_cast<int>(p21[0]));
    RCLCPP_INFO(get_logger(), "Integer vector parameter [0]dout_pins: %d",static_cast<int>(p22[0]));             
    RCLCPP_INFO(get_logger(), "Boolean parameter  with_ori : %d", p23);
    RCLCPP_INFO(get_logger(), "Integer parameter  publish_rate: %ld", p24);
    RCLCPP_INFO(get_logger(), "String  parameter type :%s", p26.c_str());
    RCLCPP_INFO(get_logger(), "Integer parameter state_publish_rate: %ld", p27);
    RCLCPP_INFO(get_logger(), "Double parameter goal_time: %f", p35);
    RCLCPP_INFO(get_logger(), "Double parameter stopped_velocity_tolerance: %f", p36);
    RCLCPP_INFO(get_logger(), "Double parameter dead_man_timeout: %f", p29);
    RCLCPP_INFO(get_logger(), "Double parameter ee_vel_limit: %f", p30);
    RCLCPP_INFO(get_logger(), "Boolean parameter with_plot : %d", p31);
    RCLCPP_INFO(get_logger(), "Double parameter stop_trajectory_duration: %f", p32);
    RCLCPP_INFO(get_logger(), "Integer parameter action_monitor_rate: %ld", p34);
         
  };
};
  int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MainNode>());
  rclcpp::shutdown();
  return 0;
  };
