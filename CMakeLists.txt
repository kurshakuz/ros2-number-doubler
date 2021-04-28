cmake_minimum_required(VERSION 3.5)
project(doubler_pubsub)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(client src/publisher_client_member_function.cpp)
ament_target_dependencies(client rclcpp std_msgs)
add_executable(service src/subscriber_service_member_function.cpp)
ament_target_dependencies(service rclcpp std_msgs)


install(TARGETS
  client
  service
  DESTINATION lib/${PROJECT_NAME})

ament_package()
