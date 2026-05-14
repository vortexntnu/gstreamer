cmake_minimum_required(VERSION 3.8)
project(gstreamer_to_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED
  gstreamer-1.0
  gstreamer-app-1.0
)

add_library(gstreamer_to_ros_component SHARED
  src/gstreamer_to_ros_node.cpp
)

target_include_directories(gstreamer_to_ros_component PUBLIC
  include
  ${GST_INCLUDE_DIRS}
)

target_link_libraries(gstreamer_to_ros_component
  ${GST_LIBRARIES}
)

ament_target_dependencies(gstreamer_to_ros_component
  rclcpp
  rclcpp_components
  sensor_msgs
)

rclcpp_components_register_node(gstreamer_to_ros_component
  PLUGIN "gstreamer_to_ros::GStreamerToROS"
  EXECUTABLE gstreamer_to_ros_node
)

install(TARGETS gstreamer_to_ros_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
