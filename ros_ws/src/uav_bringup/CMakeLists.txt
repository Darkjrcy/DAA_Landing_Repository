cmake_minimum_required(VERSION 3.8)
project(uav_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(gnss_multipath_plugin REQUIRED) 
find_package(gz-math7 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


# Add the ROS2 executable for saving the data:
add_executable(save_uav_info_node src/save_uav_info.cpp)
# Link teh dependencies:
ament_target_dependencies(save_uav_info_node
  "rclcpp"
  "nav_msgs"
  "std_msgs"
  "rosgraph_msgs"
  "gz-math7"
  "gnss_multipath_plugin"
)
# Intall the executable:
install(TARGETS save_uav_info_node
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


# Install the launch folders and the models inside the 
install(
  DIRECTORY
    launch
    worlds
    models
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
