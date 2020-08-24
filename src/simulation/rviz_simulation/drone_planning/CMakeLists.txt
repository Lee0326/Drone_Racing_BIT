
cmake_minimum_required(VERSION 2.8.3)
project(drone_planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  message_runtime
  visualization_msgs
  tf
  pose_utils
  quadrotor_msgs
)

generate_messages()

catkin_package()

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(drone_planning 
src/gate_detector.cpp
src/RapidTrajectoryGenerator.cpp
src/SingleAxisTrajectory.cpp
src/drone_planning_node.cpp
src/trajectory_generator.cpp
)

target_link_libraries(drone_planning
   ${catkin_LIBRARIES}
   pose_utils
)


