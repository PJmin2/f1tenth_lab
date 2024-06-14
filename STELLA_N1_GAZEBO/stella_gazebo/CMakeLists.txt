cmake_minimum_required(VERSION 3.8)
project(stella_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(stella_description REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(imu_to_ahrs src/imu_to_ahrs.cpp)
add_executable(odom_ahrs_fusion src/odom_ahrs_fusion.cpp)

ament_target_dependencies(imu_to_ahrs rclcpp sensor_msgs std_msgs)
ament_target_dependencies(odom_ahrs_fusion rclcpp nav_msgs sensor_msgs std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

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

install(TARGETS imu_to_ahrs
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS odom_ahrs_fusion
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY
    launch
    worlds
    models
  DESTINATION
    share/${PROJECT_NAME}/
)

ament_package()
