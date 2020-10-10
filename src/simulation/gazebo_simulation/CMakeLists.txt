cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)
project(drone_racing_simulation)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    cv_bridge
    image_transport)

find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)

find_package(gazebo REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

catkin_package()

add_executable(${PROJECT_NAME}_node
    src/dataset_collector_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

