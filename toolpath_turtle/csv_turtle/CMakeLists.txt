cmake_minimum_required(VERSION 3.0.2)
project(csv_turtle)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  turtlesim
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs turtlesim
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/csv_turtle.py
  scripts/turtle_plan_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
