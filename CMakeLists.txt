cmake_minimum_required(VERSION 2.8.3)
project(tritech_micron)

find_package(catkin REQUIRED COMPONENTS
  rospy
  genmsg
  std_msgs
  actionlib
  sensor_msgs
  geometry_msgs
  actionlib_msgs
)

add_action_files(
  FILES
  Scan.action
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
)
