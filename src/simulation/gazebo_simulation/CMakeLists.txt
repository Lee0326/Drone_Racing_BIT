cmake_minimum_required(VERSION 2.8.3)
add_compile_options(-std=c++11)
project(drone_racing_simulation)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    gazebo_ros
    std_msgs
    nav_msgs
    cv_bridge
    image_transport
    camera_info_manager
    sensor_msgs
)

find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)

find_package(gazebo REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
    DEPENDS
        roscpp
        gazebo_ros
        image_transport
        camera_info_manager
        sensor_msgs
)
include_directories(
    include)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(
    realsense_gazebo_plugin
    src/RealSensePlugin.cpp
    src/gazebo_ros_realsense.cpp
    )
target_link_libraries(realsense_gazebo_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(realsense_gazebo_plugin ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node
    src/dataset_collector_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)



