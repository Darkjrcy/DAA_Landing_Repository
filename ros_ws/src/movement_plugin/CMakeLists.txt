cmake_minimum_required(VERSION 3.8)
project(movement_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_gz_bridge REQUIRED)
# Dependencies for the plugin:
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(gz-sim7 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

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


# Install the plugin of 3d movement:
# Inlcude the headers:
include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)
# Add the plugin source file
add_library(movement_3d_plugin SHARED
  src/movement_3d_plugin.cpp)
# Link the plugin witht he needed libraries:
ament_target_dependencies(movement_3d_plugin
  "rclcpp"
  "geometry_msgs"
  "tf2_ros"
  "gz-sim7"
  "gz-plugin2"
  "nav_msgs"
  "gz-common5"
  "gz-math7"
)
# Install shared library to lib/
install(TARGETS movement_3d_plugin
  LIBRARY DESTINATION lib
)

ament_package()
