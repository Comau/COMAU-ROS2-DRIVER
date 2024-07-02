# Copyright 2024 Comau Robotics S.p.A.

cmake_minimum_required(VERSION 3.8)
project(comau_tcp_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
    message(STATUS "C++17 support: OK (-std=c++17)")
elseif(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
    message(STATUS "C++14 support: OK (-std=c++14)")
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    message(STATUS "C++11 support: OK (-std=c++11)")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    message(STATUS "C++00 support: OK (-std=c++0x)")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(comau_msgs REQUIRED)
find_package(pluginlib REQUIRED)

#add_executable(comau_tcp_interface src/comau_state_client_node.cpp)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

###########
## Build ##
###########

include_directories(
  include
  include/comau_tcp_interface
  include/comau_tcp_interface/utils
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  comau_msgs
  pluginlib
  rclcpp
  rcutils
  Boost
)

add_library(comau_tcp_interface
  SHARED
  src/${PROJECT_NAME}/utils/message_package.cpp
  src/${PROJECT_NAME}/comau_tcp_interface.cpp
  src/${PROJECT_NAME}/comau_state_client.cpp
  src/${PROJECT_NAME}/comau_robot_client.cpp
)

target_include_directories(
  comau_tcp_interface PUBLIC
<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
<INSTALL_INTERFACE:include>)

target_link_libraries(
  comau_tcp_interface
  ${catkin_LIBRARIES}
)

ament_target_dependencies(
  comau_tcp_interface
  pluginlib
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(comau_tcp_interface comau_tcp_client_plugins.xml)

target_compile_features(comau_tcp_interface PRIVATE cxx_std_17)

add_executable(comau_tcp_interface_node src/comau_state_client_node.cpp)
ament_target_dependencies(comau_tcp_interface_node ${THIS_PACKAGE_INCLUDE_DEPENDS})
install(TARGETS comau_tcp_interface_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  comau_tcp_interface_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES comau_tcp_client_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include)
  install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
) 

ament_export_libraries(comau_tcp_interface)
ament_export_dependencies(
  rclcpp
  pluginlib
  rcutils
  comau_msgs 
  Boost
)

ament_export_include_directories(
  include
)

if(BUILD_TESTING)
find_package(ament_cmake_gtest REQUIRED)
ament_add_gtest(${PROJECT_NAME}_tutorial_test test/tutorial_test.cpp)
target_include_directories(${PROJECT_NAME}_tutorial_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_tutorial_test
  std_msgs
)
target_link_libraries(${PROJECT_NAME}_tutorial_test comau_tcp_interface)
endif()

ament_package()
