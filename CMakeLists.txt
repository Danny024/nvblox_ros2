cmake_minimum_required(VERSION 3.18)
project(nvblox_ros2 LANGUAGES CXX CUDA)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(Eigen3 REQUIRED)

add_subdirectory(src/nvblox)

add_executable(nvblox_ros2_node src/nvblox_ros2_node.cpp)
target_include_directories(nvblox_ros2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/nvblox/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(nvblox_ros2_node
  nvblox_lib
  Eigen3::Eigen)
ament_target_dependencies(nvblox_ros2_node
  rclcpp sensor_msgs geometry_msgs visualization_msgs nav_msgs tf2_ros message_filters)

install(TARGETS nvblox_ros2_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
