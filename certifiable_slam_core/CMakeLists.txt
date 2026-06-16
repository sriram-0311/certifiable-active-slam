cmake_minimum_required(VERSION 3.8)
project(certifiable_slam_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(GTSAM REQUIRED)

include_directories(include)

add_executable(build_graph 
  src/main.cpp
  src/RosbagParser.cpp
)

target_link_libraries(build_graph gtsam)
ament_target_dependencies(build_graph
  rclcpp
  rosbag2_cpp
  rosbag2_storage
  nav_msgs
)

install(TARGETS build_graph
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()