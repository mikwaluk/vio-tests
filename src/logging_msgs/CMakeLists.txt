cmake_minimum_required(VERSION 3.0.2)
project(logging_msgs)


find_package(catkin REQUIRED COMPONENTS std_msgs)
find_package(genmsg REQUIRED)
find_package(geometry_msgs REQUIRED)

add_message_files(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} FILES msg/Bias.msg msg/TrackedFeatures.msg)
# Do code generation, specifying which other message packages we depend on.
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  CATKIN_DEPENDS geometry_msgs sensor_msgs nav_msgs tf2_ros visualization_msgs std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})


