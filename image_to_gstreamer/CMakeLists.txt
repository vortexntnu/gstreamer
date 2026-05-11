cmake_minimum_required(VERSION 3.8)
project(image_to_gstreamer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vortex_utils_ros REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED
  gstreamer-1.0
  gstreamer-app-1.0
)

add_library(image_to_gstreamer_component SHARED
  src/image_to_gstreamer_node.cpp
)

target_include_directories(image_to_gstreamer_component PUBLIC
  include
  ${GST_INCLUDE_DIRS}
)

target_link_libraries(image_to_gstreamer_component
  ${GST_LIBRARIES}
)

ament_target_dependencies(image_to_gstreamer_component
  rclcpp
  rclcpp_components
  sensor_msgs
  vortex_utils_ros
)

rclcpp_components_register_node(image_to_gstreamer_component
  PLUGIN "image_to_gstreamer::ImageToGStreamer"
  EXECUTABLE image_to_gstreamer_node
)

install(TARGETS image_to_gstreamer_component
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
