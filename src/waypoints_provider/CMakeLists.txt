cmake_minimum_required(VERSION 3.0.2)
project(waypoints_provider)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} 
  src/waypoints_provider.cpp
)

target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
)