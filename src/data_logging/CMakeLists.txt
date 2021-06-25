cmake_minimum_required(VERSION 3.0.2)
project(data_logging)


find_package(catkin REQUIRED COMPONENTS rospy std_msgs)


catkin_package(
  CATKIN_DEPENDS geometry_msgs rospy sensor_msgs nav_msgs tf2_ros visualization_msgs logging_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})
catkin_python_setup()
catkin_install_python(PROGRAMS src/data_logging/odom_logger.py src/data_logging/bias_logger.py src/data_logging/imu_logger.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
