#include <thread>
#include <memory>

#include <comau_hardware_interface/comau_hw_control_loop.hpp>
#include <comau_hardware_interface/comau_hw_interface.hpp>
#include <csignal>

// ROS includes
#include "controller_manager/controller_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/thread_priority.hpp"

boost::shared_ptr<comau_hardware_control_loop::ComauHWControlLoop> c_hw_control_loop_ptr;

void signalHandler(int signum) {

  RCLCPP_WARN_STREAM(rclcpp::get_logger("comau_robot_interface"),"Interrupt signal (" << signum << ") received.\n");
  rclcpp::sleep_for(rclcpp::Duration::from_seconds(0.2).to_chrono<std::chrono::nanoseconds>());
  c_hw_control_loop_ptr.reset();

  exit(signum);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> nh_ = std::make_shared<rclcpp::Node>("comau_hardware_control_loop");
  nh_->declare_parameter("loop_hz", 500.0);
  nh_->declare_parameter("cycle_time_error_threshold", 0.025);
  /*c_hw_control_loop_ptr->loop_hz_ = nh_->get_parameter("loop_hz").as_double();
  c_hw_control_loop_ptr->cycle_time_error_threshold_ = nh_->get_parameter("cycle_time_error_threshold").as_double();*/

  // create executor
  std::shared_ptr<rclcpp::Executor> e = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  // create controller manager instance
  auto controller_manager = std::make_shared<controller_manager::ControllerManager>(e, "controller_manager");

  // control loop thread
  std::thread control_loop([controller_manager]() {
    if (!realtime_tools::configure_sched_fifo(50)) {
      RCLCPP_WARN(controller_manager->get_logger(), "Could not enable FIFO RT scheduling policy");
    }

    // for calculating sleep time
    auto const period = std::chrono::nanoseconds(1'000'000'000 / controller_manager->get_update_rate());
    auto const cm_now = std::chrono::nanoseconds(controller_manager->now().nanoseconds());
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> next_iteration_time{ cm_now };

    // for calculating the measured period of the loop
    rclcpp::Time previous_time = controller_manager->now();

    /*ANDY UPDATE PERIOD AS DESIRED */

    while (rclcpp::ok()) {
      // calculate measured period
      auto const current_time = controller_manager->now();
      auto const measured_period = current_time - previous_time;
      previous_time = current_time;

      // execute update loop
      controller_manager->read(controller_manager->now(), measured_period);
      //controller_manager->update(controller_manager->now(), measured_period);
      controller_manager->write(controller_manager->now(), measured_period);
      RCLCPP_WARN(controller_manager->get_logger(), "Cycle");
      // wait until we hit the end of the period
      next_iteration_time += period;
      std::this_thread::sleep_until(next_iteration_time);
    }
  });

  // spin the executor with controller manager node
  e->add_node(controller_manager);
  e->spin();

  // wait for control loop to finish
  control_loop.join();

  // shutdown
  rclcpp::shutdown();

  return 0;
}