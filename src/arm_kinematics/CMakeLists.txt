cmake_minimum_required(VERSION 3.8)
project(arm_kinematics)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(arm_interfaces REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(kinematics_node src/kinematics_node.cpp)
ament_target_dependencies(kinematics_node
  rclcpp
  geometry_msgs
  sensor_msgs
  arm_interfaces
  kdl_parser
  urdf
)
target_include_directories(kinematics_node PRIVATE ${orocos_kdl_INCLUDE_DIRS})
target_link_libraries(kinematics_node ${orocos_kdl_LIBRARIES} Eigen3::Eigen)

install(TARGETS kinematics_node DESTINATION lib/${PROJECT_NAME})

ament_package()
