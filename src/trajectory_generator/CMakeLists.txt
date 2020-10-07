cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)
project(minimum_snap_trajectory)

find_package(catkin REQUIRED 
    COMPONENTS
    roscpp
    message_generation
    message_runtime
    quadrotor_msgs
    controller_msgs 
)
generate_messages()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp quadrotor_msgs controller_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

include_directories("/usr/include/eigen3/")

add_executable(${PROJECT_NAME}_node
    src/trajectory_point.cpp
    src/minimum_snap_trajectories_node.cpp
#    src/minimum_snap_office_node.cpp
    src/constrained_polynomial_trajectories.cpp
    src/minimum_snap_trajectories.cpp
    src/polynomial_trajectories_common.cpp
    src/polynomial_trajectory.cpp
    src/geometry_eigen_conversions.cpp
    src/parameter_helper.cpp
    src/global_trajectory.cpp
)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)

