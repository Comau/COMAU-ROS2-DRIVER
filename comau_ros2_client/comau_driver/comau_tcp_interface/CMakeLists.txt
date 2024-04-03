cmake_minimum_required(VERSION 3.8)
project(comau_tcp_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-std=c++11)
#endif()

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

#ament_target_dependencies(
#  comau_tcp_interface
#  ${THIS_PACKAGE_INCLUDE_DEPENDS}
#)

ament_target_dependencies(
  comau_tcp_interface
  pluginlib
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

pluginlib_export_plugin_description_file(comau_driver/comau_tcp_interface comau_tcp_client_plugins.xml)

target_compile_features(comau_tcp_interface PRIVATE c_std_99 cxx_std_17)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
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
ament_export_include_directories(include)
ament_export_libraries(comau_tcp_interface)
ament_export_dependencies(
  rclcpp
  pluginlib
  rcutils
  comau_msgs 
  Boost
)
ament_package()
