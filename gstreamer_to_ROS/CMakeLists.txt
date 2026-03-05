cmake_minimum_required(VERSION 3.8)
project(gstreamer_to_ROS)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED
  gstreamer-1.0
  gstreamer-app-1.0
)

add_executable(gstreamer_to_ROS_node
  src/gstreamer_to_ROS_node.cpp
)

target_include_directories(gstreamer_to_ROS_node PUBLIC
  include
  ${GST_INCLUDE_DIRS}
)

target_link_libraries(gstreamer_to_ROS_node
  ${GST_LIBRARIES}
)

ament_target_dependencies(gstreamer_to_ROS_node
  rclcpp
  sensor_msgs
)

install(TARGETS
  gstreamer_to_ROS_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()