cmake_minimum_required(VERSION 3.8)
project(bluerov2_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# GStreamer via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
)

# --- ROV Controller Node ---
add_executable(rov_controller
  src/rov_controller_node.cpp
  src/rov_controller_main.cpp
)
ament_target_dependencies(rov_controller
  rclcpp std_msgs sensor_msgs geometry_msgs mavros_msgs
)
target_include_directories(rov_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# --- Camera Streamer Node ---
add_executable(camera_streamer
  src/camera_streamer_node.cpp
  src/camera_streamer_main.cpp
)
ament_target_dependencies(camera_streamer
  rclcpp sensor_msgs image_transport cv_bridge
)
target_include_directories(camera_streamer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${GST_INCLUDE_DIRS}
)
target_link_libraries(camera_streamer ${GST_LIBRARIES} ${OpenCV_LIBRARIES})

# --- Install ---
install(TARGETS rov_controller camera_streamer
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
