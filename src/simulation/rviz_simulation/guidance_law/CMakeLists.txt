cmake_minimum_required(VERSION 2.8.3)
project(guidance_law)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  quadrotor_msgs
)

generate_messages()

catkin_package()

find_package(Eigen3 REQUIRED)

include_directories(${EIGEN_INCLUDE_DIR})


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
    src/guidance_law_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)
