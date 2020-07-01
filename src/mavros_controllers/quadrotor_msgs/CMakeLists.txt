cmake_minimum_required(VERSION 2.8.3)
project(quadrotor_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-O3)

find_package(catkin REQUIRED COMPONENTS geometry_msgs nav_msgs message_generation std_msgs)


add_message_files(
  DIRECTORY msg
  FILES
  TrajectoryPoint.msg
)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(
  CATKIN_DEPENDS 
  geometry_msgs 
  nav_msgs 
  message_runtime 
  std_msgs
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
