cmake_minimum_required(VERSION 3.0.2)
project(waypoint_global_planner)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package()

add_library(${PROJECT_NAME}
  src/waypoint_global_planner.cpp
)

target_link_libraries(${PROJECT_NAME} PRIVATE
  ${catkin_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
)