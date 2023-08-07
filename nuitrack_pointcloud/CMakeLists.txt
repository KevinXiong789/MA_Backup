cmake_minimum_required(VERSION 3.8)
project(nuitrack_pointcloud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(NUITRACK_SDK_INCLUDE_DIR /home/student/Nuitrack/nuitrack-sdk-master/Nuitrack/include)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
#find_package(OpenCV REQUIRED)
#find_package(cv_bridge REQUIRED)
#find_package(nuitrack REQUIRED) # Assuming there's a FindNuitrack.cmake or NuitrackConfig.cmake available

include_directories(
  ${NUITRACK_SDK_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)
link_directories("/usr/local/lib/nuitrack")

add_executable(nuitrack_skeleton_pointcloud src/nuitrack_skeleton_pointcloud.cpp)

ament_target_dependencies(nuitrack_skeleton_pointcloud 
  rclcpp 
  std_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs)
target_link_libraries(nuitrack_skeleton_pointcloud nuitrack ${OpenCV_LIBS})

#Don't forget add executable name here
install(TARGETS
    nuitrack_skeleton_pointcloud
    DESTINATION lib/${PROJECT_NAME}) 



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
