cmake_minimum_required(VERSION 3.5)
project(tritech_micron)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

# add_message_files(FILES TritechMicronConfig.msg)
# generate_messages(DEPENDENCIES std_msgs)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TritechMicronConfig.msg"
  DEPENDENCIES builtin_interfaces std_msgs
 )

 ament_export_dependencies(rosidl_default_runtime)

# catkin_install_python(
#   PROGRAMS
#     scripts/csv_to_pointcloud.py
#     scripts/scan.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Install Python executables
# install(PROGRAMS
#   scripts/scan.py
#   DESTINATION lib/${PROJECT_NAME}
# )
install(PROGRAMS scripts/scan.py DESTINATION bin)
ament_package()