cmake_minimum_required(VERSION 3.8)
project(assign2_660610842)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(a3_sub src/a3_sub.cpp)
add_executable(talker src/publisher_lambda_function.cpp)
add_executable(listener1 src/subscriber_lambda_function_1.cpp)
add_executable(listener2 src/subscriber_lambda_function_2.cpp)

ament_target_dependencies(a3_sub rclcpp sensor_msgs)
ament_target_dependencies(talker rclcpp std_msgs)
ament_target_dependencies(listener1 rclcpp std_msgs)
ament_target_dependencies(listener2 rclcpp std_msgs)


install(TARGETS
 talker
 listener1
 listener2
 a3_sub
 DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})

ament_package()
