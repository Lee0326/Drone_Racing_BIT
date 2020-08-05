cmake_minimum_required(VERSION 2.8.3)
project(trajectory_generator)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  quadrotor_msgs
)

generate_messages()


find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp quadrotor_msgs
  DEPENDS Eigen3 system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
    src/trajectory_generator_node.cpp
    src/RapidTrajectoryGenerator.cpp
    src/SingleAxisTrajectory.cpp
)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)
