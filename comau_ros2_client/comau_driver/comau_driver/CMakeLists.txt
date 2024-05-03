cmake_minimum_required(VERSION 3.8)
project(comau_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wno-unused-parameter)
endif()

# check c++17 / c++11 / c++0x / c++14

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++17" COMPILER_SUPPORTS_CXX17)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)

if(COMPILER_SUPPORTS_CXX17)
  message(STATUS "CCompile with C++17 support")
  add_compile_options(-std=c++17)
elseif(COMPILER_SUPPORTS_CXX14)
  message(STATUS "Compile with c++14 support")
  add_compile_options(-std=c++14) 
elseif(COMPILER_SUPPORTS_CXX11)
  message(STATUS "Compile with c++11 support")
  add_compile_options(-std=c++11)
elseif(COMPILER_SUPPORTS_CXX0X)
  message(STATUS "Compile with c++0x support")
  add_compile_options(-std=c++0x)
else()
  message(FATAL_ERROR "Il compilatore ${CMAKE_CXX_COMPILER} has no C++ support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(comau_tcp_interface REQUIRED) 
#find_package(rosparam_shortcuts REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
# Comau Driver

add_library(
    ${PROJECT_NAME}
    SHARED
    src/comau_driver.cpp)
 
target_include_directories(
       ${PROJECT_NAME}
       PUBLIC
       include   
 	 ${catkin_INCLUDE_DIRS}
 	 ${Boost_INCLUDE_DIRS}
 	 ${comau_tcp_interface_INCLUDE_DIRS})
 
#necessario per compilazione 	        
ament_target_dependencies( 
      ${PROJECT_NAME} 
      rclcpp
      pluginlib
      comau_tcp_interface
#     rosparam_shortcuts
      Boost) 

ament_export_include_directories(
      include)
      
ament_export_libraries(
      ${PROJECT_NAME})
      
ament_export_dependencies(   
      rclcpp
      pluginlib 
      include
      comau_tcp_interface
      Boost)
#   rosparam_shortcuts
 
target_link_libraries(
  ${PROJECT_NAME}
  ${rclcpp_LIBRARIES}
  ${catkin_LIBRARIES}
  ${comau_tcp_interface_LIBRARIES})

install(
        TARGETS
        ${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
  	LIBRARY DESTINATION lib/${PROJECT_NAME}
  	RUNTIME DESTINATION bin/${PROJECT_NAME})
 
install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp")

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
   #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
 # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
