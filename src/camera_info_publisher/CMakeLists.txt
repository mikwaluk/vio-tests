cmake_minimum_required(VERSION 3.0.2)
project(camera_info_publisher)


find_package(catkin REQUIRED COMPONENTS rospy std_msgs)


catkin_package(
  CATKIN_DEPENDS geometry_msgs rospy sensor_msgs nav_msgs tf2_ros visualization_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS src/publisher.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
