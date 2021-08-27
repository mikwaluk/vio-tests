cmake_minimum_required(VERSION 3.0.2)
project(feature_tracker_evaluation)


find_package(catkin REQUIRED COMPONENTS rospy std_msgs)


catkin_package(
  CATKIN_DEPENDS geometry_msgs rospy sensor_msgs nav_msgs tf2_ros visualization_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_install_python(PROGRAMS src/evaluator.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
