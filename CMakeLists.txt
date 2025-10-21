cmake_minimum_required(VERSION 3.8)
project(imx_stereo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)

# Include headers
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${GST_INCLUDE_DIRS}
)

link_directories(${GST_LIBRARY_DIRS})
add_definitions(${GST_CFLAGS_OTHER})

# Add executable
add_executable(stereo_sync_node
  src/main.cpp
  src/dual_camera.cpp
)

ament_target_dependencies(stereo_sync_node
  rclcpp
  cv_bridge
  image_transport
  sensor_msgs
  OpenCV
)

target_link_libraries(stereo_sync_node
  ${GST_LIBRARIES}
)

# Install
install(TARGETS stereo_sync_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  cv_bridge
  image_transport
  sensor_msgs
  OpenCV
)

ament_package()
