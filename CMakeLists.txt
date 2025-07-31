cmake_minimum_required(VERSION 3.8)
project(turtlmap)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(dvl_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)
find_package(GTSAM REQUIRED)
find_package(yaml-cpp REQUIRED)

# Generate service interfaces
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveTrajectory.srv"
)

# Get typesupport target for linking
rosidl_get_typesupport_target(
  rosidl_typesupport_cpp_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

# Library target containing all implementation files
add_library(${PROJECT_NAME}_lib
  src/Posegraph.cpp
  src/PosegraphBackendOnline.cpp
  src/VelocityIntegrationFactor.cpp # NOTE: Do not include the executable here, it will be linked in the executable
  src/BluerovBarometerFactor.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
  dvl_msgs
)

# Link with core and unstable GTSAM, YAML, and TBB, plus ROS typesupport
target_link_libraries(${PROJECT_NAME}_lib
  ${rosidl_typesupport_cpp_target}
  gtsam
  gtsam_unstable
  yaml-cpp
  TBB::tbb
)

# Node executable
add_executable(posegraph_backend_online_node
  src/PosegraphBackendOnlineNode.cpp
)
ament_target_dependencies(posegraph_backend_online_node
  rclcpp
)
target_link_libraries(posegraph_backend_online_node
  ${PROJECT_NAME}_lib
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}_lib
  posegraph_backend_online_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
