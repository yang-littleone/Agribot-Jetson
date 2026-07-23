cmake_minimum_required(VERSION 3.8)
project(turn_on_32chassis)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)  # 添加对serial包的依赖
find_package(sensor_msgs REQUIRED)  # 添加对sensor_msgs包的依赖
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  # Legacy chassis sources are not uncrustify-clean. Keep that pre-existing
  # formatting out of this functional change; new files are checked directly.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_wheel_trust_monitor
    test/test_wheel_trust_monitor.cpp
  )
  target_include_directories(test_wheel_trust_monitor PUBLIC include)
  add_launch_test(
    test/test_slip_aware_odometry_launch.py
    TIMEOUT 30
  )
endif()

include_directories(
  include
  ${serial_INCLUDE_DIRS}  # 添加serial包的包含目录
)

# 添加可执行文件
add_executable(
  turn_on_32chassis_node
  src/turn_on_32chassis.cpp
  src/Quaternion_Solution.cpp
)
add_executable(imu_source_selector_node src/imu_source_selector.cpp)
add_executable(slip_aware_odometry_node src/slip_aware_odometry.cpp)

# 链接rclcpp库和serial库
ament_target_dependencies(turn_on_32chassis_node
  rclcpp
  serial
  sensor_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
)

ament_target_dependencies(imu_source_selector_node
  rclcpp
  sensor_msgs
)

ament_target_dependencies(slip_aware_odometry_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS
  turn_on_32chassis_node
  imu_source_selector_node
  slip_aware_odometry_node
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY
#   include
#   DESTINATION include/
# )
install(DIRECTORY
  launch
  urdf
  config
  DESTINATION share/${PROJECT_NAME}
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
