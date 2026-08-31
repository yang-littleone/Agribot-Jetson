cmake_minimum_required(VERSION 3.8)
project(thesis_field_experiments)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/indoor_headland_simulator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
