cmake_minimum_required(VERSION 2.8.3)
project(pupil_ros_plugin)
find_package(catkin REQUIRED roscpp rospy std_msgs geometry_msgs message_generation)

add_message_files( FILES gaze.msg pupil.msg gaze_positions.msg pupil_positions.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime roscpp rospy std_msgs geometry_msgs)



