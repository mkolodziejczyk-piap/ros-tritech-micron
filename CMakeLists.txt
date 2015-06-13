cmake_minimum_required(VERSION 2.8.3)
project(tritech_micron)

find_package(catkin REQUIRED COMPONENTS
  rospy
  genmsg
  std_msgs
  sensor_msgs
  geometry_msgs
  dynamic_reconfigure
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/Scan.cfg
)

catkin_package(
  CATKIN_DEPENDS
  rospy
)
