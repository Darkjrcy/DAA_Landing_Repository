cmake_minimum_required(VERSION 3.8)
project(uav_dynamics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gnss_multipath_plugin REQUIRED) 
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)


# Generate the custom messgaes:
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AvoidanceStates.msg"
  DEPENDENCIES gnss_multipath_plugin  
)

# Add the include directory:
include_directories(include)

# Install the Fixed wiing guidance logic:
# COmpile both the Geometric_guidance and the fixed_wing model executbales:
add_executable(fixed_wing_dynamics_node
  src/fixed_wing_model.cpp
  src/geometric_guidance.cpp
)
# Add the local messages:
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(fixed_wing_dynamics_node "${cpp_typesupport_target}")
# Add the dependencies:
ament_target_dependencies(fixed_wing_dynamics_node
  rclcpp
  std_msgs
  geometry_msgs
  gnss_multipath_plugin
)
# Link the C++ libraries
target_include_directories(fixed_wing_dynamics_node PUBLIC ${EIGEN3_INCLUDE_DIRS})
target_link_libraries(fixed_wing_dynamics_node Eigen3::Eigen)
# Intall the executable:
install(TARGETS fixed_wing_dynamics_node
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
