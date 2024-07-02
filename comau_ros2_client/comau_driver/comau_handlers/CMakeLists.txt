# Copyright 2024 Comau Robotics S.p.A.

cmake_minimum_required(VERSION 3.8)
project(comau_handlers)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-unused-parameter -Werror)
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
find_package(ros2_control REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(comau_msgs REQUIRED)
find_package(realtime_tools REQUIRED)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
)
set(THIS_PACKAGE_INCLUDE_DEPENDS
rclcpp
rclcpp_action
kdl_parser
comau_driver
ros2_control
tf2
tf2_ros
std_msgs
comau_msgs
geometry_msgs
tf2_geometry_msgs
geometry2
hardware_interface
urdf
rclcpp_components
realtime_tools
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## Comau Action Handlers
add_library(comau_handlers
  src/execute_joint_trajectory_handler.cpp
  src/execute_cartesian_trajectory_handler.cpp
  src/trajectory_handler.cpp
)
ament_export_targets(comau_handlers HAS_LIBRARY_TARGET)

target_link_libraries(comau_handlers
                      ${catkin_LIBRARIES}
)

ament_target_dependencies(comau_handlers 
                          rclcpp
                          ${${PROJECT_NAME}_EXPORTED_TARGETS} 
                          ${catkin_EXPORTED_TARGETS}
                          ${THIS_PACKAGE_INCLUDE_DEPENDS}
)



install(TARGETS comau_handlers
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS comau_handlers
  EXPORT comau_handlers
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
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
ament_export_dependencies(rclcpp Boost)
#ament_export_libraries( rclcpp
#                        rclcpp_action
#                        comau_driver
#                        ros2_control
#                        tf2
#                        tf2_ros
#                        tf2_geometry_msgs
#                        std_msgs
#                        comau_msgs
#                        geometry_msgs
#)


add_executable(execute_trajectory_handler_node 
src/execute_trajectory_handler_node.cpp
src/execute_joint_trajectory_handler.cpp
src/execute_cartesian_trajectory_handler.cpp
src/trajectory_handler.cpp
)
ament_target_dependencies(execute_trajectory_handler_node rclcpp ament_cmake pluginlib Boost ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(
  TARGETS
  execute_trajectory_handler_node
  comau_handlers
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()