cmake_minimum_required(VERSION 3.8)
project(robot_control_ssm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(angles REQUIRED)
find_package(ur_msgs REQUIRED)

#find_package(robot_model REQUIRED)
#find_package(robot_model_loader REQUIRED)
#find_package(robot_state REQUIRED)

# robot_speed_separation.cpp
add_executable(robot_speed_separation src/robot_speed_separation.cpp)
target_include_directories(robot_speed_separation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(robot_speed_separation PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  robot_speed_separation
  "moveit_ros_planning_interface"
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "PCL"
  "pcl_conversions"
  "angles"
  "ur_msgs"
)

# min_distance.cpp
add_executable(min_distance src/min_distance.cpp)
target_include_directories(min_distance PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(min_distance PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  min_distance
  "moveit_ros_planning_interface"
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "PCL"
  "pcl_conversions"
  "angles"
  "ur_msgs"
)


install(TARGETS 
  robot_speed_separation
  min_distance
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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
