cmake_minimum_required(VERSION 3.0.2)
project(composite_cbf)

set(CMAKE_CXX_FLAGS "-O3")
add_compile_options(-std=c++17)

find_package(Eigen3 REQUIRED)
find_package(qpOASES REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    nav_msgs
    mavros_msgs
)

catkin_package(
    LIBRARIES composite_cbf
    CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs nav_msgs mavros_msgs
)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${qpOASES_INCLUDE_DIR}
)

add_executable(composite_cbf_node src/CompositeCbfNode.cpp src/CbfSafetyFilter.cpp)
target_link_libraries(
    composite_cbf_node
    ${catkin_LIBRARIES}
    ${qpOASES_LIBRARY_DIR}/libqpOASES.a
)

message(foo=${qpOASES_LIBRARY_DIR})
