cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_adapter)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)

catkin_package()

catkin_install_python(
  PROGRAMS
    src/pointcloud_adapter.py
  DESTINATION 
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)
