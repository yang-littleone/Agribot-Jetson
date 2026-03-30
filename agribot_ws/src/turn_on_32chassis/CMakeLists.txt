cmake_minimum_required(VERSION 3.8)
project(turn_on_32chassis)

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
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


include_directories(
  include
  ${serial_INCLUDE_DIRS}  # 添加serial包的包含目录
  
  )

# 添加可执行文件
add_executable(turn_on_32chassis_node src/turn_on_32chassis.cpp src/Quaternion_Solution.cpp)


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

install(TARGETS
  turn_on_32chassis_node
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

ament_package()