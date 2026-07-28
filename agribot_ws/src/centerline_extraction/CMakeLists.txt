cmake_minimum_required(VERSION 3.8)
project(centerline_extraction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置CMP0074策略以处理PCL_ROOT变量
cmake_policy(SET CMP0074 NEW)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# 包含PCL头文件目录
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  )

# 链接PCL库
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
# add_executable(centerline_extraction_node  src/centerline_extraction_node.cpp)
# ament_target_dependencies(centerline_extraction_node rclcpp sensor_msgs nav_msgs visualization_msgs pcl_conversions )
# target_link_libraries(centerline_extraction_node
#   ${PCL_LIBRARIES}
# )

# install(TARGETS
#   centerline_extraction_node 
#   DESTINATION lib/${PROJECT_NAME}
# )
# add_executable(pcl_learning  src/pcl_learning.cpp)
# ament_target_dependencies(pcl_learning rclcpp sensor_msgs pcl_conversions visualization_msgs)
# target_link_libraries(pcl_learning
#   ${PCL_LIBRARIES}
# )

# add_executable(centerline_extract  src/centerline_extract.cpp)
# ament_target_dependencies(centerline_extract rclcpp sensor_msgs pcl_conversions visualization_msgs)
# target_link_libraries(centerline_extract
#   ${PCL_LIBRARIES}
# )

add_executable(corn_row_detector_projection  src/corn_row_detector_projection.cpp)
ament_target_dependencies(corn_row_detector_projection 
  rclcpp 
  nav_msgs
  std_msgs
  sensor_msgs 
  pcl_conversions 
  visualization_msgs
  tf2
  tf2_geometry_msgs)
target_link_libraries(corn_row_detector_projection
  ${PCL_LIBRARIES}
)


# add_executable(corn_row_detector  src/corn_row_detector.cpp)
# ament_target_dependencies(corn_row_detector rclcpp sensor_msgs pcl_conversions visualization_msgs nav_msgs)
# target_link_libraries(corn_row_detector
#   ${PCL_LIBRARIES}
# )

add_executable(cornfield_navigation_node
  # src/corn_row_detector.cpp
  src/pure_pursuit_controller.cpp
  src/obstacle_detector.cpp
  src/pid_controller.cpp
  src/main.cpp
)

ament_target_dependencies(cornfield_navigation_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  pcl_conversions
  tf2
  tf2_geometry_msgs
  PCL
  pcl_conversions
  visualization_msgs
)
target_link_libraries(cornfield_navigation_node
  ${PCL_LIBRARIES}
)

# 在没有玉米行检测输入时，为 PID 跟踪提供可重复的室内测试中心线。
add_executable(test_path_publisher src/test_path_publisher.cpp)
ament_target_dependencies(test_path_publisher
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  cornfield_navigation_node corn_row_detector_projection test_path_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pid_tracking_evaluator.py
  scripts/field_trial_logger.py
  scripts/analyze_field_trials.py
  scripts/prepare_ground_truth.py
  scripts/record_field_bag.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  include
  DESTINATION include/
)
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY docs
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(field_analysis_test test/test_field_analysis.py)
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
