cmake_minimum_required(VERSION 3.8)
project(comau_handlers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

# check c++17 / c++11 / c++0x
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    message(STATUS "C++11 support: OK (-std=c++17)")
elseif(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
    message(STATUS "C++11 support: OK (-std=c++14)")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    message(STATUS "C++11 support: OK (-std=c++11)")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    message(STATUS "C++11 support: OK (-std=c++0x)")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(comau_driver REQUIRED)
find_package(joint_limits REQUIRED)
find_package(ros2_control REQUIRED)
find_package(tf2_ros REQUIRED)
#rosparam_shortcuts
find_package(urdf REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(comau_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(nav2_util REQUIRED)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
)

## Comau Action Handlers
add_library(comau_handlers
  src/execute_joint_trajectory_handler.cpp
  src/execute_cartesian_trajectory_handler.cpp
)
ament_target_dependencies(
  comau_handlers
  rclcpp
  rclcpp_action
  rclcpp_components
  nav2_util
  comau_driver
  tf2_ros
  std_msgs
  geometry_msgs
  tf2_geometry_msgs
  urdf
  comau_msgs
  Boost
  hardware_interface
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(comau_handlers
  ${catkin_LIBRARIES}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION share/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)
ament_export_include_directories(include)
ament_export_libraries(comau_handlers)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  rclcpp_components
  nav2_util
  comau_driver
  joint_limits
  ros2_control
  tf2_ros
  #rosparam_shortcuts
  urdf
  std_msgs
  geometry_msgs
  tf2_geometry_msgs
  comau_msgs
  Boost
)
install(TARGETS comau_handlers
        DESTINATION lib/${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION bin/${PROJECT_NAME}
)
ament_package()