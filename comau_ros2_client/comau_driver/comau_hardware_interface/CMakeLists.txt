# Copyright 2024 Comau Robotics S.p.A.

cmake_minimum_required(VERSION 3.8)
project(comau_hardware_interface)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-unused-parameter -Werror -fPIC)
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

find_package(comau_driver REQUIRED)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${Boost_INCLUDE_DIRS}
 ${HW_IF_INCLUDE_DEPENDS}
)

# find dependencies
set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
  kdl_parser
  rclcpp
  controller_interface
  controller_manager
  controller_manager_msgs
  realtime_tools
  trajectory_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  urdf
  std_msgs
  comau_msgs
  comau_driver
  comau_handlers
  Boost
  #comau_controllers
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  comau_hardware_interface
  SHARED
  src/comau_robot_interface_node.cpp
  src/comau_hw_interface.cpp
  src/comau_control.cpp
)
ament_export_targets(comau_hardware_interface HAS_LIBRARY_TARGET)
target_link_libraries(comau_hardware_interface
  ${catkin_LIBRARIES}
)

ament_target_dependencies(comau_hardware_interface
  rclcpp
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${HW_IF_INCLUDE_DEPENDS}
)

install(TARGETS comau_hardware_interface
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(
  TARGETS comau_hardware_interface
  EXPORT comau_hardware_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY include/comau_hardware_interface
  DESTINATION include/${PROJECT_NAME}/)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN ".svn" EXCLUDE
)

## EXPORTS
ament_export_include_directories(include)
ament_export_libraries(comau_hardware_interface)
ament_export_dependencies(rclcpp)

## COMPILE
add_executable(comau_robot_interface_node 
  src/comau_robot_interface_node.cpp
  src/comau_control.cpp
  src/comau_hw_interface.cpp
)
ament_target_dependencies(comau_robot_interface_node
  ${HW_IF_INCLUDE_DEPENDS}
)
target_link_libraries(comau_robot_interface_node
  comau_hardware_interface
  #comau_hardware_control_loop
  ${catkin_LIBRARIES}
)

install(
  TARGETS 
  comau_robot_interface_node
  comau_hardware_interface
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/comau_hardware_interface
)

install(
  DIRECTORY 
    src
    config
   # test
  DESTINATION share/${PROJECT_NAME})

#target_compile_features(comau_hardware_interface PUBLIC cxx_std_17)
#target_include_directories(comau_hardware_interface PUBLIC
#$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
#$<INSTALL_INTERFACE:include>
#)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "comau_hardware_interface_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface comau_hardware_interface.xml)
# Export controller plugins
pluginlib_export_plugin_description_file(controller_interface comau_hardware_interface.xml)

# INSTALL
## Mark cpp header files for installation

ament_package()