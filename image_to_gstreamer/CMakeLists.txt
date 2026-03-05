cmake_minimum_required(VERSION 3.8)
project(image_to_gstreamer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED
  gstreamer-1.0
  gstreamer-app-1.0
)

add_executable(image_to_gstreamer_node
  src/image_to_gstreamer_node.cpp
)

target_include_directories(image_to_gstreamer_node PUBLIC
  include
  ${GST_INCLUDE_DIRS}
)

target_link_libraries(image_to_gstreamer_node
  ${GST_LIBRARIES}
)

ament_target_dependencies(image_to_gstreamer_node
  rclcpp
  sensor_msgs
)

install(TARGETS
  image_to_gstreamer_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()