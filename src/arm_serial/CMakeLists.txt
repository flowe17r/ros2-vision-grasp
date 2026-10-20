cmake_minimum_required(VERSION 3.8)
project(arm_serial)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(arm_interfaces REQUIRED)

add_executable(serial_bridge src/serial_bridge.cpp)
target_include_directories(serial_bridge PRIVATE include)
ament_target_dependencies(serial_bridge
  rclcpp
  sensor_msgs
  std_srvs
  arm_interfaces
)

install(TARGETS serial_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frame_codec test/test_frame_codec.cpp)
  target_include_directories(test_frame_codec PRIVATE include)
endif()

ament_package()
