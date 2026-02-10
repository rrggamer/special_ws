cmake_minimum_required(VERSION 3.8)
project(custom_action_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(custom_action_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)

# ---------------------------------------------------------
# 1. SERVER (Stays as a Component Library)
# ---------------------------------------------------------
add_library(action_server SHARED
  src/fibonacci_action_server.cpp)

target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(action_server
  PRIVATE "CUSTOM_ACTION_CPP_BUILDING_DLL")

ament_target_dependencies(action_server
  "custom_action_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")

rclcpp_components_register_node(action_server 
  PLUGIN "custom_action_cpp::FibonacciActionServer" 
  EXECUTABLE fibonacci_action_server)

install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

# ---------------------------------------------------------
# 2. CLIENT (Changed to Standalone Executable)
# ---------------------------------------------------------
# We use the name 'manual_fibonacci_client' as you requested
add_executable(manual_fibonacci_client src/fibonacci_action_client.cpp)

target_include_directories(manual_fibonacci_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Use ament_target_dependencies instead of target_link_libraries
# It automatically handles includes and linker flags for ROS packages
ament_target_dependencies(manual_fibonacci_client
  "custom_action_interfaces"
  "rclcpp"
  "rclcpp_action"
)

install(TARGETS
  manual_fibonacci_client
  DESTINATION lib/${PROJECT_NAME}
) 

ament_package()