cmake_minimum_required(VERSION 3.8)
project(convoy_traj)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robot_localization REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(apriltag_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(launch REQUIRED)
# find_package(launch_ros REQUIRED)

# Build as a shared library (composable component)
add_library(convoy_traj_component SHARED
  src/convoy_traj_node.cpp)
target_include_directories(convoy_traj_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(convoy_traj_component PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  convoy_traj_component
  "rclcpp"
  "rclcpp_components"
  "robot_localization"
  "message_filters"
  "apriltag_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "tf2_geometry_msgs"
  "tf2"
  "tf2_ros"
)

# Register the component and generate the convoy_traj_node executable from it.
# rclcpp_components_register_node creates the executable target, so no separate
# add_executable is needed (that would produce a duplicate-target CMake error).
rclcpp_components_register_node(convoy_traj_component
  PLUGIN "convoy_traj::ConvoyTrajNode"
  EXECUTABLE convoy_traj_node
)

# Install the library
install(TARGETS convoy_traj_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/calibrate_tags.py
    scripts/test_tag_fallback.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
