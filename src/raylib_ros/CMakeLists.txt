cmake_minimum_required(VERSION 3.8)
project(raylib_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) # <-- Add this
find_package(raylib REQUIRED)

add_executable(pid_tuner src/main.cpp)

target_include_directories(pid_tuner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src> # <-- Add this so it finds raygui.h
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(pid_tuner rclcpp sensor_msgs geometry_msgs) # <-- Add geometry_msgs
target_link_libraries(pid_tuner raylib)

install(TARGETS pid_tuner
  DESTINATION lib/${PROJECT_NAME})


ament_package()
