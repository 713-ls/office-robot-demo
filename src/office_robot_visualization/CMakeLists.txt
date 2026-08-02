cmake_minimum_required(VERSION 3.0.2)
project(office_robot_visualization)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  visualization_msgs
  geometry_msgs
)

catkin_package(
#  CATKIN_DEPENDS rospy std_msgs visualization_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
