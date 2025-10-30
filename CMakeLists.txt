cmake_minimum_required(VERSION 3.16)
project(composite_cbf)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(composite_cbf_node src/CompositeCbfNode.cpp src/CbfSafetyFilter.cpp)

ament_target_dependencies(composite_cbf_node
  rclcpp
  geometry_msgs
  sensor_msgs
  mavros_msgs
  nav_msgs
)

install(TARGETS composite_cbf_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_package()
