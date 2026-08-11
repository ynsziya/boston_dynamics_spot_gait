cmake_minimum_required(VERSION 3.8)
project(robot_dog_gait)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(robot_dog_gait_core
  src/robot_dog_model.cpp
  src/leg_kinematics.cpp
  src/gait_engine.cpp
  src/trajectory_generator.cpp
  src/body_pose_controller.cpp
)
target_include_directories(robot_dog_gait_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(robot_dog_gait_core
  rclcpp
  std_msgs
  geometry_msgs
)

add_executable(robot_dog_controller src/robot_dog_controller_node.cpp)
target_link_libraries(robot_dog_controller robot_dog_gait_core)
ament_target_dependencies(robot_dog_controller
  rclcpp
  std_msgs
  geometry_msgs
)

install(TARGETS robot_dog_gait_core robot_dog_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
